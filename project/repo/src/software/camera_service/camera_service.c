// ============================================================================
// camera_service.c —— 取流实现的四个后端（file / synthetic / spinnaker / gentl）
//
// 本文件把「怎么取一帧」这件事压在函数指针表后面，上层只认 ps_camera_pump()。
//
// ★ RAW Bayer 红线（方案 §7）★
// ---------------------------------------------------------------------------
// 相机只允许输出 **RAW Bayer 单通道马赛克**（RAW12 装在 16 bit 容器里）。
// 绝不允许让 SDK 先做 8bit RGB 转换（解马赛克）：
//   * 那样进 M1 的就不是 RAW12 而是三通道 8bit，「彩色先解交织再分块」的口径
//     作废，墙网格边长会算错一倍；
//   * 8bit 会丢掉 4 bit 动态范围，而本项目正好靠低比特差分成像，压掉不可逆；
//   * 解马赛克搬到 SDK/CPU 侧，也违背方案 §7「CPU 不得调用 NumPy/OpenCV 完成
//     在线重建」与「强化 FPGA 核心性」的取向。
// 落实：
//   1) 编译期红线 —— 见下面的 #error；
//   2) 运行期检查 —— ps_camera_format_check()；
//   3) 打开成功时把「本路流的格式已按 RAW Bayer 口径确认」记在
//      confirmed_raw_bayer 里（自检会断言它为 1）。
//
// 为什么 synthetic 用「位置无关哈希」而不是有状态的 LCG
// ---------------------------------------------------------------------------
// 有状态生成器一旦被分段 pull（PL 流水上必然如此）就无法从中间续起，只能
// rewind 重跑。位置无关的哈希让 pull 可以在任意偏移处独立取值，与真实流一致。
// 本文件的 ps_camera_synthetic_pixel() 与 src/software/ps_sources.hpp 的
// ps_sim_pixel() **逐值相同**（同 splitmix64、同 [dark, 满量程] 映射），
// host_selftest 里有交叉比对 —— 两条路分叉了就必须当场发现。
//
// 板端替换点
// ---------------------------------------------------------------------------
//   ps_cam_spinnaker_open / ps_cam_gentl_open 两个留桩函数体里有逐步 TODO
//   （PixelFormat=BayerRG12、ROI、帧号、时间戳、曝光）。其余代码板上原样复用。
// ============================================================================

#include "camera_service.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------- 编译期红线
//
// 方案 §7 的硬约束：相机 SDK 不得先做 8bit RGB 转换。
// 谁要是为了让画面「好看一点」而在编译命令里打开这个宏，这里直接编译失败，
// 而不是等到板上发现 M1 边长对不上、动态范围不够时再回头找原因。
#if defined(PS_CAMERA_ALLOW_SDK_RGB_CONVERSION)
#  error "红线：相机 SDK 不得先做 8bit RGB 转换，只保存 RAW Bayer（见 camera_service.h 头注释）"
#endif

// ---------------------------------------------------------------- 平台小件

static int ps_cam_path_is_ascii(const char* s) {
    if (!s) return 0;
    for (; *s != '\0'; ++s) {
        const unsigned char c = (unsigned char)*s;
        if (c < 0x20u || c > 0x7Eu) return 0;
    }
    return 1;
}

static void ps_cam_put_reason(char* reason, size_t cap, const char* msg) {
    if (!reason || cap == 0u) return;
    size_t i = 0u;
    if (msg) {
        while (msg[i] != '\0' && i + 1u < cap) {
            reason[i] = msg[i];
            ++i;
        }
    }
    reason[i] = '\0';
}

// ---------------------------------------------------------------- 名字

const char* ps_camera_impl_name(int impl) {
    switch (impl) {
        case PS_CAM_IMPL_FILE:      return "file";
        case PS_CAM_IMPL_SYNTHETIC: return "synthetic";
        case PS_CAM_IMPL_SPINNAKER: return "spinnaker";
        case PS_CAM_IMPL_GENTL:     return "gentl";
        default:                    return "unknown";
    }
}

int ps_camera_impl_from_name(const char* name) {
    if (!name) return -1;
    if (strcmp(name, "file") == 0)      return PS_CAM_IMPL_FILE;
    if (strcmp(name, "synthetic") == 0) return PS_CAM_IMPL_SYNTHETIC;
    if (strcmp(name, "synth") == 0)     return PS_CAM_IMPL_SYNTHETIC;
    if (strcmp(name, "spinnaker") == 0) return PS_CAM_IMPL_SPINNAKER;
    if (strcmp(name, "gentl") == 0)     return PS_CAM_IMPL_GENTL;
    return -1;
}

// ---------------------------------------------------------------- 格式检查

int ps_camera_format_check(const ps_camera_format_t* f, char* reason, size_t cap) {
    if (!f) {
        ps_cam_put_reason(reason, cap, "格式描述为空");
        return PS_CAM_ERR_ARG;
    }
    if (f->width == 0u || f->height == 0u) {
        ps_cam_put_reason(reason, cap, "宽/高为 0");
        return PS_CAM_ERR_FORMAT;
    }
    if (f->block == 0u) {
        ps_cam_put_reason(reason, cap, "分块边长为 0");
        return PS_CAM_ERR_FORMAT;
    }
    if (f->frames == 0u) {
        ps_cam_put_reason(reason, cap, "帧累积数为 0");
        return PS_CAM_ERR_FORMAT;
    }

    // ---- RAW Bayer 红线的三条运行期检查 ----
    if (f->interleaved_rgb != 0u) {
        ps_cam_put_reason(reason, cap,
                          "格式标记为已解交织的 RGB：SDK 不得先做 8bit RGB 转换，"
                          "只保存 RAW Bayer");
        return PS_CAM_ERR_FORMAT;
    }
    if (f->bytes_per_pixel != 2u) {
        ps_cam_put_reason(reason, cap,
                          "每像素字节数不是 2（RAW12 必须装在 16 bit 容器里；"
                          "1 字节 = 8bit 截断，3 字节 = RGB8 解马赛克）");
        return PS_CAM_ERR_FORMAT;
    }
    if (f->container_bits != 8u && f->container_bits != 12u &&
        f->container_bits != 16u) {
        ps_cam_put_reason(reason, cap, "容器位深不在 {8,12,16}");
        return PS_CAM_ERR_FORMAT;
    }
    if (f->bayer_phase != 0u && f->bayer_phase != 1u) {
        ps_cam_put_reason(reason, cap, "Bayer 相位不在 {0=RGGB, 1=BGGR}");
        return PS_CAM_ERR_FORMAT;
    }

    // ---- 几何自洽（与 ps_contract.hpp 的 ps_validate_spec 同判据）----
    {
        const uint32_t cs = ps_camera_crop_side(f);
        if (cs == 0u || cs > f->width || cs > f->height) {
            ps_cam_put_reason(reason, cap, "裁剪边长非法（应 <= min(宽,高)）");
            return PS_CAM_ERR_FORMAT;
        }
        if ((cs % 2u) != 0u) {
            // RAW Bayer 解交织要求偶数边长，奇数边长意味着最后一行/列会被丢掉，
            // 丢掉之后墙网格就与几何配置对不上了。
            ps_cam_put_reason(reason, cap, "裁剪边长必须是偶数（解交织要求）");
            return PS_CAM_ERR_FORMAT;
        }
        // 彩色口径：unit = 2 * block。不整除意味着 M1 会静默取整。
        const uint32_t unit = 2u * f->block;
        if ((cs % unit) != 0u) {
            ps_cam_put_reason(reason, cap,
                              "裁剪边长不能被 2*分块边长整除（M1 会静默取整，"
                              "墙网格随之对不上）");
            return PS_CAM_ERR_FORMAT;
        }
    }
    ps_cam_put_reason(reason, cap, "RAW Bayer 格式合法");
    return PS_CAM_OK;
}

int ps_camera_geometry_check(const ps_camera_format_t* f, uint32_t wall_n,
                             char* reason, size_t cap) {
    const int rc = ps_camera_format_check(f, reason, cap);
    if (rc != PS_CAM_OK) return rc;
    if (ps_camera_wall_side(f) != wall_n) {
        // 与 ps_contract.hpp 的 kPsSideMismatch 同一件事：规格自洽但配错了档位
        // （例如拿 B 档规格去配官方 126 的墙网格）。
        ps_cam_put_reason(reason, cap,
                          "推出的墙网格边长与几何配置不符"
                          "（彩色档：crop/2/block 必须等于 wall.n）");
        return PS_CAM_ERR_FORMAT;
    }
    return PS_CAM_OK;
}

// ---------------------------------------------------------------- synthetic

static uint64_t ps_cam_splitmix64(uint64_t seed, uint64_t k) {
    uint64_t x = seed ^ (k * 0x9E3779B97F4A7C15ull);
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

static uint64_t ps_cam_fullscale(uint32_t container_bits) {
    if (container_bits == 0u || container_bits >= 32u) return 0u;
    return ((uint64_t)1u << container_bits) - 1u;
}

uint16_t ps_camera_synthetic_pixel(const ps_camera_format_t* f, uint64_t seed,
                                   uint64_t k) {
    if (!f) return 0u;
    const uint64_t fs = ps_cam_fullscale(f->container_bits);
    const uint64_t lo = (f->dark_level <= fs) ? f->dark_level : 0u;
    const uint64_t span = (fs >= lo) ? (fs - lo) : 0u;
    const uint64_t h = ps_cam_splitmix64(seed, k);
    uint64_t v = lo + ((span > 0u) ? (h % (span + 1u)) : 0u);
    if (v > fs) v = fs;
    return (uint16_t)v;
}

void ps_camera_synthetic_fill(const ps_camera_format_t* f, uint64_t seed,
                              uint64_t first_index, uint16_t* dst,
                              uint64_t n_pixels) {
    if (!dst) return;
    uint64_t k;
    for (k = 0u; k < n_pixels; ++k) {
        dst[k] = ps_camera_synthetic_pixel(f, seed, first_index + k);
    }
}

// ---------------------------------------------------------------- 上下文

// 四个实现共用一份上下文（ps_camera_open 里 malloc，ps_camera_close 里 free）。
//   file      : f != NULL，pos/total_pixels 来自文件长度
//   synthetic : f == NULL，pos/total_pixels 来自帧数与幅面
//   留桩两种  : 不成功打开，上下文随错误一起释放
typedef struct {
    ps_camera_capture_t* cap;     // 回指句柄（fmt / seed 从这里读）
    FILE*    f;                   // 仅 file 实现使用
    int64_t  exposure_us;         // 主机实现只「记账」；真实值来自相机 SDK
    uint64_t pos;                 // 读游标（像素）
    uint64_t total_pixels;
} ps_cam_ctx_t;

// ---------------------------------------------------------------- 取流内核

// file 与 synthetic 共用这一个 pull：差别只在 pixels 从文件读还是现算。
static long long ps_cam_read_pixels(void* ctx, uint16_t* dst, long long max_pixels) {
    ps_cam_ctx_t* c = (ps_cam_ctx_t*)ctx;
    if (!c || !c->cap || !dst) return PS_CAM_ERR_ARG;
    if (max_pixels <= 0) return PS_CAM_ERR_ARG;
    if (c->pos >= c->total_pixels) return 0;      // 流已尽
    uint64_t n = c->total_pixels - c->pos;
    if (n > (uint64_t)max_pixels) n = (uint64_t)max_pixels;
    if (c->f) {
        const size_t want = (size_t)n * sizeof(uint16_t);
        const size_t got = fread(dst, 1, want, c->f);
        if (got != want) return PS_CAM_ERR_IO;    // 半帧 = 出错，不静默补零
    } else {
        // synthetic：按**绝对下标**（c->pos 起）现算，不做任何色彩转换（红线见文件头）。
        // 这里必须带 c->pos：不带的话分段 pull 的第二段会从 0 重新开始，
        // 表现为「每帧都是第一帧」，而且不报任何错。
        ps_camera_synthetic_fill(&c->cap->fmt, c->cap->seed, c->pos, dst, n);
    }
    c->pos += n;
    return (long long)n;
}

static int ps_cam_rewind_impl(void* ctx) {
    ps_cam_ctx_t* c = (ps_cam_ctx_t*)ctx;
    if (!c) return PS_CAM_ERR_ARG;
    if (c->f) fseek(c->f, 0, SEEK_SET);
    c->pos = 0u;
    return PS_CAM_OK;
}

static int ps_cam_set_exposure(void* ctx, int64_t us) {
    ps_cam_ctx_t* c = (ps_cam_ctx_t*)ctx;
    if (!c || us < 0) return PS_CAM_ERR_ARG;
    c->exposure_us = us;   // 主机只记账；板上写在 Spinnaker/GenTL 的曝光节点
    return PS_CAM_OK;
}

static int64_t ps_cam_get_exposure(void* ctx) {
    ps_cam_ctx_t* c = (ps_cam_ctx_t*)ctx;
    return c ? c->exposure_us : (int64_t)PS_CAM_UNAVAILABLE;
}

static int ps_cam_file_open(void* ctx, const ps_camera_format_t* fmt,
                            const char* dev) {
    ps_cam_ctx_t* c = (ps_cam_ctx_t*)ctx;
    if (!c || !c->cap || !dev) return PS_CAM_ERR_ARG;
    if (!ps_cam_path_is_ascii(dev)) {
        fprintf(stderr,
                "[camera_service] 拒绝非 ASCII 路径（窄字符 fopen 会乱码）：%s\n",
                dev);
        return PS_CAM_ERR_IO;
    }
    c->f = fopen(dev, "rb");
    if (!c->f) {
        fprintf(stderr, "[camera_service] 打不开裸帧文件 %s\n", dev);
        return PS_CAM_ERR_IO;
    }
    if (fseek(c->f, 0, SEEK_END) != 0) {
        fclose(c->f);
        c->f = NULL;
        return PS_CAM_ERR_IO;
    }
    const long bytes = ftell(c->f);
    fseek(c->f, 0, SEEK_SET);
    if (bytes < 0) {
        fclose(c->f);
        c->f = NULL;
        return PS_CAM_ERR_IO;
    }
    const uint64_t total = (uint64_t)bytes / 2ull;   // uint16 小端
    const uint64_t per_frame = ps_camera_frame_pixels(fmt);
    if (per_frame == 0u || (total % per_frame) != 0u) {
        fprintf(stderr, "[camera_service] 裸帧文件长度不是整数帧：%lld 像素\n",
                (long long)total);
        fclose(c->f);
        c->f = NULL;
        return PS_CAM_ERR_IO;
    }
    c->total_pixels = total;
    c->cap->total_pixels = total;
    return PS_CAM_OK;
}

static void ps_cam_file_close(void* ctx) {
    ps_cam_ctx_t* c = (ps_cam_ctx_t*)ctx;
    if (c && c->f) {
        fclose(c->f);
        c->f = NULL;
    }
}

// ---------------------------------------------------------------- 留桩实现

static int ps_cam_spinnaker_open(void* ctx, const ps_camera_format_t* fmt,
                                 const char* dev) {
    (void)ctx;
    (void)fmt;
    // TODO(板端)：Spinnaker SDK 的接入顺序（每一步都在这一步就断言，别往下猜）
    //   1) Spinnaker::SystemPtr sys = System::GetInstance();
    //      CameraList list = sys->GetCameras(); 按序列号（dev）挑机；
    //   2) 关掉自动：ExposureAuto=Off, GainAuto=Off —— 本项目靠帧间差分测量，
    //      自动曝光会把「帧间差」抹掉；
    //   3) ★ 像素格式必须设成 RAW Bayer 单通道 ★
    //        PixelFormat = BayerRG12（或 BayerRG12Packed）；
    //      **绝对不允许** RGB8 / BGR8 / Mono8 —— 那等于让 SDK 先做 8bit RGB
    //      转换/截断，动态范围与解交织口径同时作废（见本文件头注释）。
    //      设完必须回读 PixelFormat 并断言它是 Bayer*，不是就报错退出；
    //   4) ROI：Width/Height/OffsetX/OffsetY 设成与 ps_camera_format_t 一致；
    //   5) 曝光：ExposureTime = exposure_us，回读确认（相机有量化步长）；
    //   6) 取流：BeginAcquisition()，GetNextImage() 拿到的是**单通道马赛克**，
    //      直接按 16 bit 容器装进 uint16（12bit 右对齐），不做任何转换；
    //   7) 帧号与时间戳：image->GetFrameID() / GetTimeStamp()，回传给 telemetry
    //      （帧号跳变 = 丢帧）；时间戳单调性与主机时钟的差别要显式标注；
    //   8) 收尾：EndAcquisition() + camera->DeInit() + System::ReleaseInstance()。
    fprintf(stderr,
            "[camera_service] Spinnaker SDK 未接入（实物未到）：设备 %s 不可用。\n"
            "        接入步骤见 camera_service.c 的 TODO（★ 必须 BayerRG12，"
            "禁止 RGB8）。\n",
            dev ? dev : "(null)");
    return PS_CAM_UNAVAILABLE;
}

static int ps_cam_gentl_open(void* ctx, const ps_camera_format_t* fmt,
                             const char* dev) {
    (void)ctx;
    (void)fmt;
    // TODO(板端)：GenTL/GenICam 路径（厂商无关，但字段名要看相机 XML）
    //   1) 打开 .cti 生产商库 -> 枚举 Interface/Device -> 按序列号（dev）打开；
    //   2) DeviceReg 里设 PixelFormat = BayerRG12（★ 理由同 Spinnaker：禁止 RGB8），
    //      并**回读断言**；
    //   3) 设 Width/Height/OffsetX/OffsetY 与 ExposureTime，同样回读断言；
    //   4) 开流：DSStartAcquisition + buffers，等 NewBuffer 事件；
    //   5) 取回的 buffer 必须是单通道 RAW；12bit 数据右对齐装进 uint16 容器；
    //   6) 帧号/时间戳优先用 chunk（ChunkFrameID / ChunkTimestamp）；没开 chunk
    //      就用主机单调时钟并**显式标注**（两套时钟不能混用）。
    fprintf(stderr,
            "[camera_service] GenTL 未接入（实物未到）：设备 %s 不可用。\n"
            "        接入步骤见 camera_service.c 的 TODO（★ 必须 BayerRG12）。\n",
            dev ? dev : "(null)");
    return PS_CAM_UNAVAILABLE;
}

static long long ps_cam_unavailable_pull(void* ctx, uint16_t* dst,
                                         long long max_pixels) {
    (void)ctx;
    (void)dst;
    (void)max_pixels;
    return PS_CAM_UNAVAILABLE;   // 实物未到：如实返回不可用，不假装成功
}

// ---------------------------------------------------------------- 打开/关闭

int ps_camera_open(ps_camera_capture_t* c, int impl, const ps_camera_format_t* fmt,
                   const char* dev) {
    if (!c || !fmt) return PS_CAM_ERR_ARG;
    memset(c, 0, sizeof(*c));
    c->impl = impl;
    c->name = ps_camera_impl_name(impl);
    c->fmt = *fmt;
    c->seed = 0x20260926ull;
    c->confirmed_raw_bayer = 0;

    // RAW Bayer 约束：任何实现都先过这一关（含留桩实现 —— 免得将来有人在
    // SDK 那边偷偷设成 RGB8，一直等板上发现边长对不上才暴露）。
    char reason[192];
    reason[0] = '\0';
    const int frc = ps_camera_format_check(fmt, reason, sizeof(reason));
    if (frc != PS_CAM_OK) {
        fprintf(stderr, "[camera_service] 格式被拒（%s）：%s\n",
                ps_camera_impl_name(impl), reason);
        return frc;
    }
    c->confirmed_raw_bayer = 1;

    ps_cam_ctx_t* ctx = (ps_cam_ctx_t*)malloc(sizeof(ps_cam_ctx_t));
    if (!ctx) return PS_CAM_ERR_IO;
    memset(ctx, 0, sizeof(*ctx));
    ctx->cap = c;

    int rc = PS_CAM_OK;
    switch (impl) {
        case PS_CAM_IMPL_FILE:
            c->open = ps_cam_file_open;
            c->close = ps_cam_file_close;
            c->pull = ps_cam_read_pixels;
            c->rewind = ps_cam_rewind_impl;
            c->set_exposure_us = ps_cam_set_exposure;
            c->get_exposure_us = ps_cam_get_exposure;
            rc = c->open(ctx, fmt, dev);
            if (rc == PS_CAM_OK) ctx->total_pixels = c->total_pixels;
            break;

        case PS_CAM_IMPL_SYNTHETIC:
            c->open = NULL;   // 无需打开动作
            c->close = NULL;
            c->pull = ps_cam_read_pixels;
            c->rewind = ps_cam_rewind_impl;
            c->set_exposure_us = ps_cam_set_exposure;
            c->get_exposure_us = ps_cam_get_exposure;
            c->total_pixels = ps_camera_frame_pixels(fmt) * (uint64_t)fmt->frames;
            ctx->total_pixels = c->total_pixels;
            break;

        case PS_CAM_IMPL_SPINNAKER:
        case PS_CAM_IMPL_GENTL:
            c->open = (impl == PS_CAM_IMPL_SPINNAKER) ? ps_cam_spinnaker_open
                                                      : ps_cam_gentl_open;
            c->close = NULL;
            c->pull = ps_cam_unavailable_pull;
            c->rewind = NULL;
            c->set_exposure_us = NULL;
            c->get_exposure_us = NULL;
            rc = c->open(ctx, fmt, dev);
            break;

        default:
            rc = PS_CAM_ERR_ARG;
            break;
    }

    if (rc != PS_CAM_OK) {
        free(ctx);
        c->confirmed_raw_bayer = 0;
        return rc;
    }
    c->ctx = ctx;
    return PS_CAM_OK;
}

void ps_camera_close(ps_camera_capture_t* c) {
    if (!c) return;
    if (c->close && c->ctx) c->close(c->ctx);
    free(c->ctx);
    c->ctx = NULL;
}

// ---------------------------------------------------------------- 取流

long long ps_camera_pump(ps_camera_capture_t* c, unsigned n_frames, uint16_t* dst) {
    if (!c || !c->pull || !dst) return PS_CAM_ERR_ARG;
    const uint64_t per_frame = ps_camera_frame_pixels(&c->fmt);
    if (per_frame == 0u) return PS_CAM_ERR_ARG;

    uint64_t got_total = 0u;
    unsigned f;
    for (f = 0u; f < n_frames; ++f) {
        uint64_t got = 0u;
        while (got < per_frame) {
            const uint64_t remain = per_frame - got;
            const long long ask = (long long)((remain > 65536u) ? 65536u : remain);
            const long long r = c->pull(c->ctx, dst + got_total + got, ask);
            if (r < 0) {
                // 中途出错/不可用：如实返回错误码，已取到的部分不回滚但也不计帧
                if (got > 0u) c->dropped_frames++;
                return r;
            }
            if (r == 0) {
                // 流已尽：这一帧取不满 -> 记丢帧，返回已取到的像素数
                if (got > 0u || f > 0u) c->dropped_frames++;
                return (long long)got_total;
            }
            got += (uint64_t)r;
        }
        got_total += per_frame;
        c->frames_served++;
    }
    return (long long)got_total;
}

int ps_camera_rewind(ps_camera_capture_t* c) {
    if (!c || !c->rewind) return PS_CAM_UNAVAILABLE;
    return c->rewind(c->ctx);
}

// ---------------------------------------------------------------- 工具

int ps_camera_write_raw_frames(const char* path, const uint16_t* data,
                               uint64_t n_pixels) {
    if (!path || !data) return PS_CAM_ERR_ARG;
    if (!ps_cam_path_is_ascii(path)) {
        fprintf(stderr, "[camera_service] 拒绝非 ASCII 路径：%s\n", path);
        return PS_CAM_ERR_IO;
    }
    FILE* f = fopen(path, "wb");
    if (!f) return PS_CAM_ERR_IO;
    const size_t w = fwrite(data, sizeof(uint16_t), (size_t)n_pixels, f);
    fclose(f);
    return (w == (size_t)n_pixels) ? PS_CAM_OK : PS_CAM_ERR_IO;
}
