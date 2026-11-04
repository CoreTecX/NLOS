// ============================================================================
// camera_service —— PS 侧采集进程（方案 §7 第 1 条 / Task 10 Step 1~2）
//
// 职责（只有这些）：
//   * 打开相机（GenTL/Spinnaker），设置曝光、ROI、像素格式、关闭所有自动项；
//   * 采 RAW，带上帧号、曝光、增益、时间戳、stride；
//   * 帧号连续性检查（§4.4：「连续采集 10 min 无帧序号跳变」）；
//   * 把「哪一帧已经落在 DDR」的事实以 FrameReady 消息发出。
//
// **不做**：不预处理、不重建、不碰 NumPy/OpenCV（方案 §7 明确禁止 CPU 做在线重建）。
// 这一条在代码里的体现是：本文件里没有任何矩阵运算，也没有图像处理 —— 采集到的
// 原始 16 bit 像素原样搬走。
//
// 硬件边界：GenTL/Spinnaker 调用全部封在 gentl_camera.hpp 的 ps_gentl_* 里。
// 本进程在主机上默认跑桩件源（--source sim），用于验证帧号、stride、12 bit 有效位、
// 超时与掉线重开这些**与相机型号无关**的逻辑。
//
// 用法：
//   ps_camera --selftest
//   ps_camera --frames 4 --reset                 # 产生 4 帧并写信箱
//   ps_camera --frames 8 --drop-stride 3         # 注入丢帧，验证跳变检测
//   ps_camera --dump-raw                         # 把末帧落到暂存目录（ASCII 路径）
//
// 所有落盘路径都走 ps::scratch_path()：主机上 $env:TEMP 若含中文会自动退到
// 相对目录 ./ps_tmp，避免窄字符 fopen 乱码（见 ps_platform.hpp 头注释）。
// ============================================================================

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "../ps_ipc.hpp"
#include "../ps_platform.hpp"
#include "gentl_camera.hpp"

namespace {

struct Options {
    bool        selftest;
    int         frames;
    bool        reset;
    bool        dump_raw;
    uint64_t    drop_stride;
    int         source;          // ps::kCameraSourceSim / GenTL
    int64_t     exposure_us;
    int         burst_frames;
    bool        rgb;
    std::string mailbox;

    Options()
        : selftest(false), frames(3), reset(false), dump_raw(false),
          drop_stride(0), source(ps::kCameraSourceSim), exposure_us(100000),
          burst_frames(1), rgb(true), mailbox(ps::ps_mailbox_default_path()) {}
};

void usage() {
    std::printf(
        "camera_service —— PenumbraSight PS 侧采集进程\n"
        "  --selftest              跑主机自检（默认动作）\n"
        "  --frames N              采集 N 帧（默认 3）\n"
        "  --reset                 先截断信箱（等价板上重置环形缓冲）\n"
        "  --dump-raw              把末帧写入暂存目录并打印指纹\n"
        "  --drop-stride N         每 N 帧注入一次帧号跳变（测跳变检测）\n"
        "  --source sim|camera     sim = 桩件源（默认），camera = GenTL 实机\n"
        "  --exposure-us N         曝光（微秒），默认 100000\n"
        "  --burst 1|4|8           采集批大小（累积在 PL 做）\n"
        "  --mono                  单色（默认 RGB/Bayer 彩色）\n"
        "  --mailbox PATH          信箱路径，默认 <scratch>/ps_penumbrasight.mb\n");
}

// 把一帧 RAW 落盘（裸 uint16 小端、无头，与 X12 的 file 源同格式）。
int dump_raw_frame(const std::string& path, const uint16_t* data, size_t pixels) {
    if (!ps::require_ascii_path(path.c_str(), "camera_service")) return -1;
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) {
        std::fprintf(stderr, "[camera_service] 无法写 %s\n", path.c_str());
        return -1;
    }
    const size_t w = std::fwrite(data, sizeof(uint16_t), pixels, f);
    std::fclose(f);
    return (w == pixels) ? 0 : -1;
}

// ---------------------------------------------------------------- 自检
int run_selftest(const Options& opt) {
    ps::PsTest t("camera_service");

    // 1) 设置自检：合法设置通过，四类错误设置必须被拒。
    ps::CameraSettings s;
    s.exposure_us = opt.exposure_us;
    s.burst_frames = opt.burst_frames;
    s.rgb = opt.rgb;
    t.check(ps::ps_camera_settings_validate(s) == ps::kPsCamOk, "B 档设置通过校验");

    ps::CameraSettings bad_auto = s;
    bad_auto.disable_gamma = false;   // 漏关 gamma：最典型的静默错误
    t.check(ps::ps_camera_settings_validate(bad_auto) == ps::kPsCamBadSettings,
            "漏关 gamma 被拒（自动项必须全关）");

    ps::CameraSettings bad_buf = s;
    bad_buf.buffer_count = 4;         // 方案 §3.2 要求 >= 8
    t.check(ps::ps_camera_settings_validate(bad_buf) == ps::kPsCamBadSettings,
            "缓冲队列 < 8 被拒");

    ps::CameraSettings bad_roi = s;
    bad_roi.offset_x = 600;           // 600+1920 > 2448
    t.check(ps::ps_camera_settings_validate(bad_roi) == ps::kPsCamBadSettings,
            "ROI 越界被拒");

    // 2) 与 X12 契约对齐：桩件流的墙网格边长必须是 B 档的 96。
    ps::CameraDevice dev;
    int rc = dev.open(s, ps::kCameraSourceSim, nullptr);
    t.check(rc == ps::kPsCamOk, "桩件源打开成功");
    const ps::RawStreamSpec spec = dev.stream_spec();
    const int spec_rc = ps::ps_validate_spec(spec, 96, 16);
    t.check(spec_rc == ps::kPsOk, "RawStreamSpec 通过 ps_validate_spec(.,96,16)");
    t.check(ps::ps_wall_side(spec) == 96, "墙网格边长 = 96");

    rc = dev.start();
    t.check(rc == ps::kPsCamOk, "start 成功");

    // 3) 连采 8 帧：帧号必须连续，元数据字段必须自洽。
    const size_t frame_pixels =
        static_cast<size_t>(s.roi_width) * static_cast<size_t>(s.roi_height);
    std::vector<uint16_t> frame(frame_pixels, 0);
    ps::PsFrameMeta meta;
    bool ids_ok = true;
    bool meta_ok = true;
    uint64_t max_pixel = 0;
    for (int i = 0; i < 8; ++i) {
        const int gr = dev.grab(frame.data(), frame.size(), &meta, 2000);
        if (gr != ps::kPsCamOk) { ids_ok = false; break; }
        if (meta.frame_id != static_cast<uint64_t>(i + 1)) ids_ok = false;
        if (meta.bytes != frame_pixels * 2u ||
            meta.stride_bytes != static_cast<uint32_t>(s.roi_width * 2) ||
            meta.bit_depth != 12 || meta.exposure_us != s.exposure_us) {
            meta_ok = false;
        }
        for (size_t k = 0; k < frame_pixels; ++k) {
            if (frame[k] > max_pixel) max_pixel = frame[k];
        }
    }
    t.check(ids_ok, "连采 8 帧帧号连续且从 1 开始");
    t.check(meta_ok, "帧元数据（bytes/stride/bit_depth/exposure）自洽");
    t.check(dev.stats().gap_events == 0, "无丢帧时 gap_events = 0");
    // packed 口径（右对齐 12 bit）：像素上界必须是 4095，而不是 65535。
    // 若哪天相机是左移 4 位，这条会先炸，提醒按 fixed_point_b.yaml 的
    // if_fullscale_then_widen 把差分通路加宽到 26 bit。
    t.check(max_pixel <= 4095u, "12 bit packed 口径：像素 <= 4095");

    // 4) 丢帧注入 -> 跳变检测必须报出来（§4.4 的判据在实物到之前就得能测）。
    dev.close();
    dev.set_stub_drop_stride(3);
    dev.open(s, ps::kCameraSourceSim, nullptr);
    dev.start();
    for (int i = 0; i < 9; ++i) {
        if (dev.grab(frame.data(), frame.size(), &meta, 2000) != ps::kPsCamOk) break;
    }
    t.check(dev.stats().gap_events >= 1, "注入丢帧后 gap_events >= 1");
    t.check(dev.stats().frames_dropped >= 1, "丢帧计数 >= 1");

    // 5) 掉线重开路径（Task 10 Step 2）必须可走通且被计数。
    const uint64_t reopen_before = dev.stats().reopen_count;
    rc = dev.reopen();
    t.check(rc == ps::kPsCamOk && dev.stats().reopen_count == reopen_before + 1,
            "reopen 成功并计数");

    // 6) 真实相机源在实物未到时必须**明确不可用**，而不是返回假帧。
    ps::CameraDevice real;
    const int real_rc = real.open(s, ps::kCameraSourceGenTL, nullptr);
    t.check(real_rc == ps::kPsCamUnavailable,
            "GenTL 实机源返回 Unavailable（不返回假数据）");

    // 7) 信箱往返：FrameReady 的载荷必须逐字节还原。
    //    这一小节先关掉丢帧注入（前面测跳变时开的），否则帧号本来就不连续，
    //    断言会变成在检查「注入是否生效」而不是「载荷是否还原」。
    dev.set_stub_drop_stride(0);
    const std::string mb = ps::scratch_path("ps_camera_selftest.mb");
    t.check(ps::is_ascii_path(mb.c_str()), "信箱路径为纯 ASCII（窄字符 fopen 安全）");
    ps::ps_mailbox_reset(mb);
    dev.close();
    dev.open(s, ps::kCameraSourceSim, nullptr);
    dev.start();
    uint32_t seq = 0;
    bool send_ok = true;
    for (int i = 0; i < 3; ++i) {
        if (dev.grab(frame.data(), frame.size(), &meta, 2000) != ps::kPsCamOk) {
            send_ok = false;
            break;
        }
        ps::PsMsg m = ps::ps_msg_make(ps::kMsgFrameReady, ps::kProcCamera,
                                      ps::kProcBroadcast, seq++);
        m.arg0 = meta.frame_id;
        if (!ps::ps_msg_set_payload(m, meta) || ps::ps_mailbox_send(mb, m) != 0) {
            send_ok = false;
            break;
        }
    }
    t.check(send_ok, "3 条 FrameReady 写入信箱");
    t.check(ps::ps_mailbox_count(mb) == 3, "信箱记录数 = 3");

    uint64_t cursor = 0;
    ps::PsMsg got;
    int n_read = 0;
    bool payload_ok = true;
    while (ps::ps_mailbox_poll(mb, &cursor, &got) == 1) {
        ps::PsFrameMeta pm;
        if (!ps::ps_msg_get_payload(got, &pm)) { payload_ok = false; break; }
        if (pm.frame_id != static_cast<uint64_t>(n_read + 1)) payload_ok = false;
        if (pm.bytes != frame_pixels * 2u) payload_ok = false;
        ++n_read;
    }
    t.check(n_read == 3 && payload_ok, "按游标回读 3 条且载荷逐字段一致");

    // 8) 裸帧落盘（走 ASCII 暂存路径）后能被原样读回。
    if (opt.dump_raw) {
        const std::string raw_path = ps::scratch_path("ps_camera_selftest.raw");
        const int wrc = dump_raw_frame(raw_path, frame.data(), frame.size());
        t.check(wrc == 0, "裸帧落盘到暂存目录");
    }

    dev.close();
    return t.finish();
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    bool want_selftest = false;

    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        if (std::strcmp(a, "--selftest") == 0) {
            want_selftest = true;
        } else if (std::strcmp(a, "--frames") == 0 && i + 1 < argc) {
            opt.frames = std::atoi(argv[++i]);
        } else if (std::strcmp(a, "--reset") == 0) {
            opt.reset = true;
        } else if (std::strcmp(a, "--dump-raw") == 0) {
            opt.dump_raw = true;
        } else if (std::strcmp(a, "--drop-stride") == 0 && i + 1 < argc) {
            opt.drop_stride = static_cast<uint64_t>(std::strtoul(argv[++i], nullptr, 10));
        } else if (std::strcmp(a, "--source") == 0 && i + 1 < argc) {
            const char* v = argv[++i];
            opt.source = (std::strcmp(v, "camera") == 0) ? ps::kCameraSourceGenTL
                                                         : ps::kCameraSourceSim;
        } else if (std::strcmp(a, "--exposure-us") == 0 && i + 1 < argc) {
            opt.exposure_us = static_cast<int64_t>(std::atoll(argv[++i]));
        } else if (std::strcmp(a, "--burst") == 0 && i + 1 < argc) {
            opt.burst_frames = std::atoi(argv[++i]);
        } else if (std::strcmp(a, "--mono") == 0) {
            opt.rgb = false;
        } else if (std::strcmp(a, "--mailbox") == 0 && i + 1 < argc) {
            opt.mailbox = argv[++i];
        } else if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "未知参数：%s\n", a);
            usage();
            return 2;
        }
    }

    if (opt.frames <= 0) {
        std::fprintf(stderr, "--frames 必须为正\n");
        return 2;
    }

    std::printf("=== camera_service（桩件；硬件边界见 gentl_camera.hpp）===\n");
    std::printf("暂存目录：%s\n", ps::scratch_dir().c_str());
    std::printf("本进程只采集 RAW + metadata，不参与预处理与重建（方案 §7）\n\n");

    if (want_selftest) return run_selftest(opt);

    // ---------------------------------------------------------------- 采集
    ps::CameraSettings s;
    s.exposure_us = opt.exposure_us;
    s.burst_frames = opt.burst_frames;
    s.rgb = opt.rgb;

    ps::CameraDevice dev;
    dev.set_stub_drop_stride(opt.drop_stride);

    int rc = dev.open(s, opt.source, nullptr);
    if (rc != ps::kPsCamOk) {
        std::fprintf(stderr,
                     "[camera_service] 打开失败（rc=%d）。若 --source camera，"
                     "请先实现 ps_gentl_open（TODO 见 gentl_camera.hpp）\n", rc);
        return 1;
    }

    // 与 X12 契约对齐：墙网格边长必须是 96，否则 M1 输出与几何配置错开。
    const ps::RawStreamSpec spec = dev.stream_spec();
    const int spec_rc = ps::ps_validate_spec(spec, 96, 16);
    if (spec_rc != ps::kPsOk) {
        std::fprintf(stderr, "[camera_service] 流规格不合法（rc=%d）\n", spec_rc);
        return 1;
    }

    rc = dev.start();
    if (rc != ps::kPsCamOk) {
        std::fprintf(stderr, "[camera_service] start 失败（rc=%d）\n", rc);
        return 1;
    }

    if (opt.reset) {
        if (ps::ps_mailbox_reset(opt.mailbox) != 0) {
            std::fprintf(stderr, "[camera_service] 信箱重置失败：%s\n", opt.mailbox.c_str());
            return 1;
        }
        std::printf("信箱已重置：%s\n", opt.mailbox.c_str());
    }

    const size_t frame_pixels =
        static_cast<size_t>(s.roi_width) * static_cast<size_t>(s.roi_height);
    std::vector<uint16_t> frame(frame_pixels, 0);
    ps::PsFrameMeta meta;

    std::printf("%-6s %-12s %-10s %-9s %-10s %-8s %s\n",
                "frame", "exposure_us", "bytes", "stride", "max_pixel", "fingerprint", "gap");
    uint32_t seq = 1;
    bool last_frame_ok = false;
    for (int i = 0; i < opt.frames; ++i) {
        rc = dev.grab(frame.data(), frame.size(), &meta, 2000);
        if (rc != ps::kPsCamOk) {
            std::fprintf(stderr, "第 %d 帧取流失败（rc=%d）\n", i, rc);
            break;
        }

        uint64_t max_pixel = 0;
        for (size_t k = 0; k < frame_pixels; ++k) {
            if (frame[k] > max_pixel) max_pixel = frame[k];
        }
        const uint64_t fp = ps::fnv1a64(frame.data(), frame.size() * sizeof(uint16_t));

        // FrameReady 只声明「这一帧已可用」，载荷是元数据（不含像素指针多少字节都行）。
        ps::PsMsg m = ps::ps_msg_make(ps::kMsgFrameReady, ps::kProcCamera,
                                      ps::kProcBroadcast, seq++);
        m.arg0 = meta.frame_id;
        if (!ps::ps_msg_set_payload(m, meta) || ps::ps_mailbox_send(opt.mailbox, m) != 0) {
            std::fprintf(stderr, "[camera_service] 写信箱失败：%s\n", opt.mailbox.c_str());
            break;
        }

        std::printf("%-6llu %-12lld %-10u %-9u %-10llu %016llx %s\n",
                    static_cast<unsigned long long>(meta.frame_id),
                    static_cast<long long>(meta.exposure_us),
                    static_cast<unsigned>(meta.bytes),
                    static_cast<unsigned>(meta.stride_bytes),
                    static_cast<unsigned long long>(max_pixel),
                    static_cast<unsigned long long>(fp),
                    (meta.frame_id == static_cast<uint64_t>(i + 1) ? "-" : "跳变"));
        last_frame_ok = true;
    }

    if (opt.dump_raw && last_frame_ok) {
        const std::string raw_path = ps::scratch_path("ps_frame_last.raw");
        if (dump_raw_frame(raw_path, frame.data(), frame.size()) == 0) {
            std::printf("末帧已写入：%s（%llu 字节）\n", raw_path.c_str(),
                        static_cast<unsigned long long>(frame.size() * 2u));
        }
    }

    dev.stop();
    const ps::CameraStats& st = dev.stats();
    std::printf("\n统计：交付 %llu 帧，丢帧 %llu，帧号跳变事件 %llu，超时 %llu，重开 %llu\n",
                static_cast<unsigned long long>(st.frames_delivered),
                static_cast<unsigned long long>(st.frames_dropped),
                static_cast<unsigned long long>(st.gap_events),
                static_cast<unsigned long long>(st.timeout_count),
                static_cast<unsigned long long>(st.reopen_count));
    std::printf("信箱：%s（当前 %lld 条）\n", opt.mailbox.c_str(),
                static_cast<long long>(ps::ps_mailbox_count(opt.mailbox)));

    // §4.4 判据：正常采集不该有跳变。桩件注入丢帧时这条会如实报出来。
    if (st.gap_events > 0) {
        std::printf("警告：检测到帧号跳变 %llu 次 —— 板端应按 Task 10 Step 2 "
                    "记录并自动重开设备，不允许静默丢帧。\n",
                    static_cast<unsigned long long>(st.gap_events));
    }
    dev.close();
    return 0;
}
