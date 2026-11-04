// ============================================================================
// X12 验收测试：PS 数据契约与三条数据源
//
// 编译（在 src/software 目录下）：
//   g++ -O2 -std=c++11 -I../hardware test_x12_contract.cpp -o t_x12
// 运行：
//   ./t_x12
//
// 判据（X12 = 1.0 人日，「三条数据源共用同一接口」）
//   T1 流规格校验：把 X1「M1 输出边长必须等于墙网格边长」变成可执行检查
//   T2 三源同一接口：相机 / 文件 / 仿真都能被同一个 ps_pump_frames 驱动
//   T3 sim ↔ file 互换：同一份数据经两条路径取出**逐值相同**
//   T4 AXI 拍打包契约：打包/解包往返精确、补零正确、TLAST 位置正确
//   T5 端到端互换：两条源驱动同一条 M1 链，输出逐值相等，且边长 == 126
//   T6 Gram 搬出预算：复核 X12 判据里的 DDR 核算（1.36 MB/候选、0.66 s）
//
// 注意：临时文件一律用**相对路径**。C++ 窄字符 fopen 走系统 ANSI 码页，
// 绝对路径里的中文（本工程路径含「学习」「26秋嵌入式」）会乱码——这是本项目
// 已踩过的坑（X8/X9/X17 都用 ./hwdata/ 这种相对前缀），相对路径是纯 ASCII。
// ============================================================================
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "preprocess.hpp"      // M1
#include "ps_contract.hpp"
#include "ps_sources.hpp"      // X12 三条数据源

static int g_pass = 0;
static int g_fail = 0;

static void check(const char* tag, bool ok, const std::string& detail = "") {
    std::printf("  [%s] %-44s %s\n", ok ? "ok  " : "FAIL", tag, detail.c_str());
    if (ok) ++g_pass; else ++g_fail;
}

static ps::RawStreamSpec mk_spec(int w, int h, int cbits, int color, int phase,
                                 int crop, int block, int frames, uint64_t dark) {
    ps::RawStreamSpec s;
    std::memset(&s, 0, sizeof(s));
    s.width = w; s.height = h; s.container_bits = cbits;
    s.color = color; s.bayer_phase = phase;
    s.crop = crop; s.block = block; s.frames = frames; s.dark_level = dark;
    return s;
}

// ---------------------------------------------------------------- T1

static void t1_spec_validation() {
    std::printf("\n=== T1 流规格校验（X1 的边长约束）===\n");

    // 官方档：crop 2016、block 8、彩色 → 2016/2/8 = 126
    const ps::RawStreamSpec official = mk_spec(2016, 2016, 16, 1, 0, 2016, 8, 1, 0);
    const int side = ps::ps_wall_side(official);
    check("官方档推出的墙网格边长 == 126", side == 126,
          "得到 " + std::to_string(side));
    check("官方档校验通过（wall_n = 126）",
          ps::ps_validate_spec(official, 126, 16) == ps::kPsOk);

    // B 档：crop 1920、block 10、彩色 → 960/10 = 96。两档的 crop 不同，不得混用
    const ps::RawStreamSpec b = mk_spec(2016, 2016, 16, 1, 0, 1920, 10, 4, 0);
    const int side_b = ps::ps_wall_side(b);
    check("B 档推出的墙网格边长 == 96", side_b == 96,
          "得到 " + std::to_string(side_b));
    check("B 档校验通过（wall_n = 96）",
          ps::ps_validate_spec(b, 96, 16) == ps::kPsOk);

    // 用 B 档规格去配官方 126 的墙网格 → 必须被拒
    check("B 档配 126 墙网格被拒（侧长不符）",
          ps::ps_validate_spec(b, 126, 16) == ps::kPsSideMismatch);

    // 裁剪边长不能被 2*block 整除 → 静默取整会让墙网格对不上，必须拒
    const ps::RawStreamSpec bad =
        mk_spec(2016, 2016, 16, 1, 0, 2015, 8, 1, 0);
    check("裁剪边长不整除被拒（bad spec）",
          ps::ps_validate_spec(bad, 126, 16) == ps::kPsBadSpec, "crop=2015");

    // 容器位宽超过编译期输入位宽 → 进 M1 前会被截断
    check("容器位宽 > 编译期 in_bits 被拒",
          ps::ps_validate_spec(official, 126, 12) == ps::kPsContainerTooLarge,
          "container=16, in_bits=12");

    // 黑白口径：不解交织，直通
    const ps::RawStreamSpec mono = mk_spec(1008, 1008, 16, 0, 0, 1008, 8, 1, 0);
    check("黑白档不解交织（1008/8 = 126）",
          ps::ps_wall_side(mono) == 126 &&
          ps::ps_validate_spec(mono, 126, 16) == ps::kPsOk);

    // 12 bit 相机口径：容器 12 位也要能过（双口径）
    const ps::RawStreamSpec cam12 = mk_spec(1920, 1200, 12, 1, 0, 960, 8, 16, 16);
    check("12 bit 相机口径可过（960/2/8 = 60）",
          ps::ps_validate_spec(cam12, 60, 16) == ps::kPsOk);
}

// ---------------------------------------------------------------- T2 / T3

static void t2_three_sources_one_interface() {
    std::printf("\n=== T2 三源共用同一接口 ===\n");

    const ps::RawStreamSpec small =
        mk_spec(64, 64, 12, 1, 0, 64, 8, 2, 16);
    const long long per_frame = (long long)small.width * small.height;
    const long long total = per_frame * small.frames;

    // --- 仿真源 ---
    ps::SimRawCtx sc;
    sc.spec = small; sc.seed = 0x20260925ULL; sc.total_pixels = total; sc.pos = 0;
    ps::RawSource sim = ps::ps_make_sim_source(&sc);
    std::vector<uint16_t> buf_sim((size_t)total, 0);
    const long long got_sim =
        ps::ps_pump_frames(sim, per_frame, small.frames, buf_sim.data());

    // --- 文件源 ---
    const char* TMP = "./x12_tmp_raw.bin";
    const int wst = ps::ps_write_raw_frames(TMP, buf_sim.data(), total);
    ps::FileRawCtx fc;
    const int ost = ps::ps_file_open(&fc, TMP, per_frame);
    ps::RawSource file = ps::ps_make_file_source(&fc);
    std::vector<uint16_t> buf_file((size_t)total, 0);
    long long got_file = -99;
    if (ost == 0) {
        got_file = ps::ps_pump_frames(file, per_frame, small.frames,
                                      buf_file.data());
    }

    // --- 相机源（实物未到，必须优雅返回而不是崩溃）---
    ps::CameraRawCtx cc;
    cc.spec = small;
    const int copen = ps::ps_camera_open(&cc, "/dev/video0");
    ps::RawSource cam = ps::ps_make_camera_source(&cc);
    std::vector<uint16_t> buf_cam((size_t)per_frame, 0);
    const long long got_cam =
        ps::ps_pump_frames(cam, per_frame, small.frames, buf_cam.data());

    // 「同一接口」的实质：三个对象形状相同、可被同一段驱动代码消费。
    // 注意不要写成 `a == b || true` 这类恒真式——那样等于没验。
    const bool same_shape =
        (sim.pull != NULL) && (file.pull != NULL) && (cam.pull != NULL) &&
        (sim.rewind != NULL) && (file.rewind != NULL) && (cam.rewind != NULL) &&
        (sim.name != NULL) && (file.name != NULL) && (cam.name != NULL) &&
        (sim.spec_known == 1) && (cam.spec_known == 1) &&
        (sim.kind == ps::kSrcSim) && (file.kind == ps::kSrcFile) &&
        (cam.kind == ps::kSrcCamera);
    check("三条源形状同构、可被同一段驱动消费", same_shape,
          std::string(sim.name) + " / " + file.name + " / " + cam.name);
    check("仿真源灌满", got_sim == total,
          "got=" + std::to_string(got_sim) + "/" + std::to_string(total));
    check("文件源可用且灌满", wst == 0 && ost == 0 && got_file == total,
          "got=" + std::to_string(got_file));
    check("相机源返回「不可用」且不崩",
          copen == PS_PULL_UNAVAILABLE && got_cam == PS_PULL_UNAVAILABLE,
          "open=" + std::to_string(copen) + " pump=" + std::to_string(got_cam));

    ps::ps_file_close(&fc);
    std::remove(TMP);

    // T3：两条路径取出的像素逐值相同
    std::printf("\n=== T3 sim ↔ file 互换（逐值）===\n");
    bool same = (got_sim == got_file);
    long long first_diff = -1;
    if (same) {
        for (long long i = 0; i < total; ++i) {
            if (buf_sim[(size_t)i] != buf_file[(size_t)i]) {
                same = false; first_diff = i; break;
            }
        }
    }
    check("逐值相同", same,
          same ? ("共 " + std::to_string(total) + " 个像素")
               : ("首个不同 @ " + std::to_string(first_diff)));
}

// ---------------------------------------------------------------- T4

static void t4_beat_packing() {
    std::printf("\n=== T4 AXI-Stream 拍打包契约 ===\n");

    const uint16_t pix[4] = {0x1234, 0xABCD, 0x0001, 0xFF00};
    uint8_t beat[PS_BEAT_BYTES];
    uint16_t back[4] = {0, 0, 0, 0};

    ps::ps_pack_beat(pix, 4, beat);
    ps::ps_unpack_beat(beat, back, 4);
    bool rt = true;
    for (int i = 0; i < 4; ++i) if (back[i] != pix[i]) rt = false;
    check("满拍往返精确", rt);

    // 小端：pix[0]=0x1234 → beat[0]=0x34, beat[1]=0x12
    check("小端字节序", beat[0] == 0x34 && beat[1] == 0x12,
          "beat[0]=" + std::to_string((int)beat[0]) +
          " beat[1]=" + std::to_string((int)beat[1]));

    // 不足一拍：高字节补零，不重复不回绕
    const uint16_t two[2] = {0x00AA, 0x00BB};
    ps::ps_pack_beat(two, 2, beat);
    bool zeroed = true;
    for (int b = 4; b < PS_BEAT_BYTES; ++b) if (beat[b] != 0) zeroed = false;
    check("不足一拍时高字节补零", zeroed && beat[0] == 0xAA && beat[2] == 0xBB);

    // 帧拍数与 TLAST
    const ps::RawStreamSpec s = mk_spec(2016, 2016, 16, 1, 0, 2016, 8, 1, 0);
    const long long bpf = ps::ps_beats_per_frame(s);
    const long long expect =
        (2016LL * 2016 + PS_PIX_PER_BEAT - 1) / PS_PIX_PER_BEAT;
    check("每帧拍数正确", bpf == expect,
          std::to_string(bpf) + " 拍 = " + std::to_string(expect));
    check("TLAST 只在末拍置位",
          ps::ps_beat_is_last(s, bpf - 1) == 1 && ps::ps_beat_is_last(s, 0) == 0);
    check("每帧流字节数 = 拍数 * 拍宽",
          ps::ps_frame_stream_bytes(s) == bpf * (long long)PS_BEAT_BYTES);
}

// ---------------------------------------------------------------- T5

// M1 彩色链路：裁剪 → 解交织 G1/G2 → 求和 → 分块 → 累积 → 行差分
static void run_m1_color(const ps::RawStreamSpec& s, const uint16_t* raw,
                         std::vector<ps::acc_t>& acc,
                         std::vector<ps::dif_t>& diff, int* bh, int* bw) {
    const int cs = ps::ps_crop_side(s);
    const size_t csn = (size_t)cs * cs;

    // center_crop 要的是 pix_t*，故先把 uint16 容器升到 pix_t
    std::vector<ps::pix_t> wide(csn, 0);
    for (size_t i = 0; i < csn; ++i) wide[i] = (ps::pix_t)raw[i];

    std::vector<ps::pix_t> cropped(csn, 0);
    ps::center_crop(wide.data(), s.height, s.width, cs, cropped.data());

    const int ph = cs / 2, pw = cs / 2;
    const size_t pn = (size_t)ph * pw;
    std::vector<ps::pix_t> g1(pn, 0), g2(pn, 0);
    ps::bayer_extract(cropped.data(), cs, cs, 1, s.bayer_phase, g1.data());
    ps::bayer_extract(cropped.data(), cs, cs, 2, s.bayer_phase, g2.data());

    std::vector<ps::gsum_t> gs(pn, 0);
    ps::green_sum(g1.data(), g2.data(), pn, gs.data());

    const int n = ps::ps_wall_side(s);
    acc.assign((size_t)n * n, 0);
    diff.assign((size_t)((n > 0) ? (n - 1) * n : 0), 0);
    ps::preprocess_plane_frames(gs.data(), 1, ph, pw, s.block,
                                acc.data(), diff.data(), bh, bw);
}

static void t5_end_to_end_swap() {
    std::printf("\n=== T5 端到端互换（两条源 → 同一条 M1 链）===\n");

    const ps::RawStreamSpec s = mk_spec(2016, 2016, 16, 1, 0, 2016, 8, 1, 0);
    const long long per_frame = (long long)s.width * s.height;

    // 源 A：仿真
    ps::SimRawCtx sc;
    sc.spec = s; sc.seed = 0x5EEDULL; sc.total_pixels = per_frame; sc.pos = 0;
    ps::RawSource sim = ps::ps_make_sim_source(&sc);
    std::vector<uint16_t> raws((size_t)per_frame, 0);
    const long long g1n = ps::ps_pump_frames(sim, per_frame, 1, raws.data());

    // 源 B：文件（内容即源 A 产出）
    const char* TMP = "./x12_tmp_full.bin";
    const int wst = ps::ps_write_raw_frames(TMP, raws.data(), per_frame);
    ps::FileRawCtx fc;
    const int ost = ps::ps_file_open(&fc, TMP, per_frame);
    ps::RawSource file = ps::ps_make_file_source(&fc);
    std::vector<uint16_t> rawf((size_t)per_frame, 0);
    long long g2n = -99;
    if (ost == 0) g2n = ps::ps_pump_frames(file, per_frame, 1, rawf.data());

    check("官方档 RAW 经两条源取满",
          g1n == per_frame && g2n == per_frame,
          std::to_string(g1n) + " / " + std::to_string(g2n));
    check("裸帧文件写读往返成功", wst == 0 && ost == 0);

    std::vector<ps::acc_t> acc_s, acc_f;
    std::vector<ps::dif_t> dif_s, dif_f;
    int bhs = 0, bws = 0, bhf = 0, bwf = 0;
    run_m1_color(s, raws.data(), acc_s, dif_s, &bhs, &bws);
    run_m1_color(s, rawf.data(), acc_f, dif_f, &bhf, &bwf);

    check("M1 输出边长 == 126（X1 那条新约束）",
          bhs == 126 && bws == 126 && bhf == 126 && bwf == 126,
          std::to_string(bhs) + "x" + std::to_string(bws));
    check("M1 输出边长 == ps_wall_side(spec)（契约自洽）",
          bhs == ps::ps_wall_side(s));

    bool acc_eq = (acc_s.size() == acc_f.size() && !acc_s.empty());
    if (acc_eq) {
        for (size_t i = 0; i < acc_s.size(); ++i) {
            if (acc_s[i] != acc_f[i]) { acc_eq = false; break; }
        }
    }
    check("两条源的块累加**逐值相等**", acc_eq,
          "共 " + std::to_string(acc_s.size()) + " 项");

    bool dif_eq = (dif_s.size() == dif_f.size() && !dif_s.empty());
    if (dif_eq) {
        for (size_t i = 0; i < dif_s.size(); ++i) {
            if (dif_s[i] != dif_f[i]) { dif_eq = false; break; }
        }
    }
    check("两条源的行差分**逐值相等**", dif_eq,
          "共 " + std::to_string(dif_s.size()) + " 项");

    // 顺带确认差分形状 = (n-1) x n，与 X1 约定 3 一致
    check("行差分形状 = (125 x 126)",
          dif_s.size() == (size_t)125 * 126,
          std::to_string(dif_s.size()));

    std::printf("        [info] 位宽：in=%d gsum=%d bs=%d acc=%d dif=%d\n",
                PS_IN_BITS, PS_GS_BITS, PS_BS_BITS, PS_ACC_BITS, PS_DIF_BITS);

    ps::ps_file_close(&fc);
    std::remove(TMP);
}

// ---------------------------------------------------------------- T6

static void t6_gram_egress_budget() {
    std::printf("\n=== T6 Gram 搬出预算（X12 判据里的 DDR 核算）===\n");

    ps::GramEgressBudget b = ps::ps_gram_egress_budget(1044, 20, 972);

    std::printf("        上三角项数          %lld\n", b.gram_entries);
    std::printf("        每候选搬出          %.4f MB（Gram %.4f + Aty %.6f）\n",
                b.bytes_per_candidate / 1.0e6, b.gram_bytes / 1.0e6,
                b.aty_bytes / 1.0e6);
    std::printf("        972 候选合计        %.4f GB\n", b.bytes_total / 1.0e9);
    std::printf("        @2.0 GB/s           %.4f s\n", b.seconds_at_2GBps);
    std::printf("        @1.0 GB/s（悲观）   %.4f s\n", b.seconds_at_1GBps);
    std::printf("        对照 整块 32 bit    %.4f MB/候选（X16 已否掉这条路）\n",
                b.bytes_per_cand_at_32bit / 1.0e6);

    check("上三角项数 = 545490", b.gram_entries == 545490,
          std::to_string(b.gram_entries));
    check("每候选 ≈ 1.36 MB",
          std::fabs(b.bytes_per_candidate / 1.0e6 - 1.36) < 0.01,
          std::to_string(b.bytes_per_candidate / 1.0e6));
    check("972 候选合计 ≈ 1.32 GB",
          std::fabs(b.bytes_total / 1.0e9 - 1.322) < 0.01,
          std::to_string(b.bytes_total / 1.0e9));
    check("@2.0 GB/s ≈ 0.66 s",
          std::fabs(b.seconds_at_2GBps - 0.664) < 0.01,
          std::to_string(b.seconds_at_2GBps));

    // 与 X10 实测的算力侧对照：0.661 s/候选
    const double compute_total = 0.661 * 972.0;
    const double ratio = b.seconds_at_2GBps / compute_total;
    std::printf("        [info] 算力侧 X10 实测 0.661 s/候选，972 个共 %.1f s\n",
                compute_total);
    std::printf("        [info] 搬出/计算 = %.2e → 搬出不是瓶颈\n", ratio);
    check("搬出耗时比计算小两个数量级以上", ratio < 1e-2,
          "比值 " + std::to_string(ratio));
}

// ----------------------------------------------------------------

int main() {
    std::printf("X12 验收测试：PS 数据契约与三条数据源\n");
    std::printf("  [info] PS_IN_BITS=%d PS_BLOCK=%d PS_FRAMES=%d PS_COLOR=%d\n",
                PS_IN_BITS, PS_BLOCK, PS_FRAMES, PS_COLOR);
    std::printf("  [info] PS_BEAT_BYTES=%d → 每拍 %d 个像素\n",
                PS_BEAT_BYTES, PS_PIX_PER_BEAT);

    t1_spec_validation();
    t2_three_sources_one_interface();
    t4_beat_packing();
    t5_end_to_end_swap();
    t6_gram_egress_budget();

    std::printf("\n%s  PASS %d  FAIL %d\n",
                (g_fail == 0) ? "全部通过。" : "存在失败项。", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
