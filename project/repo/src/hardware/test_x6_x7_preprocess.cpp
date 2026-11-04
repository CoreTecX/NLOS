// ============================================================================
// X6/X7 主机侧测试：12 bit RGGB 解马赛克 + 分块求和，**双打包口径**
//
// 背景（configs/camera_b.yaml、docs/hardware_freeze.md §1.2）：
//   相机冻结为 彩色 / 有效位深 12 bit / Bayer RGGB / 16 bit 容器，但
//   「12 bit 值摆在 16 bit 容器的哪几位」首帧实拍前无法判定：
//     packed    右对齐   -> 饱和帧最大值 4095
//     fullscale 左移 4   -> 饱和帧最大值 65520
//   本测试在**同一次运行**里把两条链都跑一遍，并核对它们只差 2^4 倍。
//
// 判据（与 S0-3 一致：全程整数、逐值相等）：
//   T1 口径常量与首帧判定：满量程 / 左移量 / 强判据（低 4 位恒零）/ 弱判据（最大值）
//   T2 解马赛克四路逐值（RGGB）        —— 两种口径
//   T3 分块求和逐值（R / G1+G2 / B）   —— 两种口径
//   T4 行差分逐值                     —— 两种口径
//   T5 8 帧累积（1/4/8 帧档）逐值      —— 两种口径
//   T6 位宽核算（preprocess.hpp 的 packing_budget）与实测上界对齐
//   T7 冻结 24 bit 差分通路在两个口径下的裁决（含饱和演示）
//   T8 与 float64 参考的相对误差
//   T9 口径等价：full == packed << 4（平面 / 块和 / 差分 / 累积）
//
// 编译（在仓库根）：
//   g++ -std=c++17 -O2 -Wall -Wextra -Wno-unknown-pragmas -Isrc/hardware src/hardware/test_x6_x7_preprocess.cpp -o build/test_x6_x7.exe
// 运行：
//   build/test_x6_x7.exe src/hardware/hwdata
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "preprocess.hpp"

static int g_pass = 0;
static int g_fail = 0;
static std::string g_dir;

// ---------------------------------------------------------------- 计数与比对

static void check(bool cond, const char* tag, const std::string& detail = "") {
    if (cond) {
        std::printf("  [ok]   %-46s %s\n", tag, detail.c_str());
        ++g_pass;
    } else {
        std::printf("  [FAIL] %-46s %s\n", tag, detail.c_str());
        ++g_fail;
    }
}

template <typename A, typename B>
static void cmp_arr(const char* tag, const A* got, const B* exp, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        if ((int64_t)got[i] != (int64_t)exp[i]) {
            std::printf("  [FAIL] %-40s 第 %zu 项: got %lld, exp %lld\n", tag, i,
                        (long long)got[i], (long long)exp[i]);
            ++g_fail;
            return;
        }
    }
    std::printf("  [ok]   %-40s (%zu 项逐值相等)\n", tag, n);
    ++g_pass;
}

// ---------------------------------------------------------------- 加载

static bool file_exists(const std::string& p) {
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) return false;
    std::fclose(f);
    return true;
}

static bool resolve_dir(const std::string& d) {
    if (file_exists(d + "/x6_x7_bitdepth__meta_block.bin")) {
        g_dir = d;
        return true;
    }
    return false;
}

static std::vector<uint8_t> slurp(const std::string& name) {
    const std::string p = g_dir + "/" + name + ".bin";
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) {
        std::printf("  [ERR] 无法打开 %s\n", p.c_str());
        std::exit(2);
    }
    std::fseek(f, 0, SEEK_END);
    const long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> b((std::size_t)n);
    if (n > 0 && std::fread(b.data(), 1, (std::size_t)n, f) != (std::size_t)n) {
        std::printf("  [ERR] 读取失败 %s\n", p.c_str());
        std::exit(2);
    }
    std::fclose(f);
    return b;
}

template <typename T>
static std::vector<T> load(const std::string& name) {
    const std::vector<uint8_t> b = slurp(name);
    std::vector<T> v(b.size() / sizeof(T));
    if (!v.empty()) std::memcpy(v.data(), b.data(), v.size() * sizeof(T));
    return v;
}

// 从 hwdata 目录名反推 npz 的字段名（export 用 <stem>__<key>）
static std::string A(const std::string& key) { return "x6_x7_bitdepth__" + key; }

// ---------------------------------------------------------------- 位宽小工具

static int bits_unsigned(uint64_t x) {
    int b = 0;
    while (x > 0) { ++b; x >>= 1; }
    return b;
}
static int bits_signed(int64_t x) {
    const uint64_t m = (x < 0) ? (uint64_t)(-x) : (uint64_t)x;
    return bits_unsigned(m) + 1;
}
// 有符号饱和到 bits 位（**不是回绕**，与 fixed_point_b.yaml 的 rounding 口径一致）
static int64_t sat_signed(int64_t v, int bits) {
    const int64_t lo = -((int64_t)1 << (bits - 1));
    const int64_t hi = ((int64_t)1 << (bits - 1)) - 1;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------------------------------------------------------------- 规模常量
// 与 src/model/scripts/make_x6_x7_vectors.py 严格对应
static const int kN = 40;
static const int kBlock = 10;
static const int kPh = kN / 2;
static const int kPw = kN / 2;
static const int kBh = kPh / kBlock;       // 2
static const int kDiffRows = kBh - 1;      // 1
static const int kNFrames = 8;
static const int kMarks[3] = {1, 4, 8};
static const char* kCases[3] = {"ramp", "mix", "sat"};
static const char* kPacks[2] = {"packed", "full"};
static const int kChannels[3] = {0, 1, 2};                 // R / G1+G2 / B
static const char* kSumName[3] = {"r", "g1g2", "b"};       // 块和命名
static const char* kDiffName[3] = {"r", "g", "b"};         // 差分命名（既有约定）

static ps::ValuePacking pack_of(int i) {
    return (i == 0) ? ps::kPacked : ps::kFullscale;
}

// ============================================================================
// T1 口径常量与首帧判定
// ============================================================================
static void t1_packing_constants() {
    std::printf("[T1] 口径常量与首帧判定\n");
    const ps::PackingSpec pk = ps::packing_spec(ps::kPacked);
    const ps::PackingSpec fs = ps::packing_spec(ps::kFullscale);

    check(pk.full_scale == 4095 && fs.full_scale == 65520,
          "满量程 4095 / 65520", "packed=4095 fullscale=65520");
    check(pk.left_shift == 0 && fs.left_shift == 4, "左移量 0 / 4", "0 / 4");
    check(pk.src_bits == 12 && fs.src_bits == 16,
          "参与累加的源位宽 12 / 16", "12 / 16");
    check(pk.effective_bits == 12 && pk.container_bits == 16 && fs.container_bits == 16,
          "有效位深 12 / 容器 16（camera_b.yaml）", "12 / 16");
    check(ps::packing_to_fullscale(4095) == 65520, "packed->fullscale：4095 -> 65520",
          "4095<<4 = 65520");

    uint64_t back = 0;
    const bool ok_div = ps::packing_to_packed(65520, &back) && back == 4095;
    check(ok_div, "fullscale->packed：65520 -> 4095", "65520>>4 = 4095");
    uint64_t dummy = 0;
    check(!ps::packing_to_packed(65521, &dummy),
          "低 4 位非零时拒绝折算（不静默丢低位）", "65521 -> 拒绝");

    // 弱判据（饱和帧看最大值）
    check(ps::detect_packing_by_max(4095) == ps::kDetectPacked,
          "弱判据 max=4095 -> packed", "packed");
    check(ps::detect_packing_by_max(4096) == ps::kDetectFullscale,
          "弱判据 max=4096 -> fullscale", "fullscale");
    check(ps::detect_packing_by_max(65520) == ps::kDetectFullscale,
          "弱判据 max=65520 -> fullscale", "fullscale");
    check(ps::detect_packing_by_max(65521) == ps::kDetectUnknown,
          "弱判据 max=65521 -> 未知（必须查）", "unknown");

    // 强判据（整帧低 4 位恒零）——用真实向量验证
    const std::vector<uint16_t> mix_p = load<uint16_t>(A("raw_mix_packed"));
    const std::vector<uint16_t> mix_f = load<uint16_t>(A("raw_mix_full"));
    check(!ps::packing_low_bits_all_zero(mix_p.data(), mix_p.size()),
          "强判据：packed 帧的低 4 位不全零（否决 fullscale）", "raw_mix_packed");
    check(ps::packing_low_bits_all_zero(mix_f.data(), mix_f.size()),
          "强判据：fullscale 帧的低 4 位恒零（坐实）", "raw_mix_full");
    check(ps::detect_packing_by_max(*std::max_element(mix_p.begin(), mix_p.end()))
              == ps::kDetectPacked,
          "真实向量 mix：max=4095 -> packed", "4095");
    check(ps::detect_packing_by_max(*std::max_element(mix_f.begin(), mix_f.end()))
              == ps::kDetectFullscale,
          "真实向量 mix_full：max=65520 -> fullscale", "65520");
    std::printf("\n");
}

// ============================================================================
// T2 解马赛克四路逐值（用既有 bayer_extract / green_sum，两口径）
// ============================================================================
static void t2_demosaic() {
    std::printf("[T2] RGGB 解马赛克四路逐值（20x20 x 4 路 x 3 用例 x 2 口径）\n");
    for (int ci = 0; ci < 3; ++ci) {
        for (int pi = 0; pi < 2; ++pi) {
            const std::string key = std::string(kCases[ci]) + "_" + kPacks[pi];
            const std::vector<uint16_t> raw = load<uint16_t>(A("raw_" + key));
            if (raw.size() != (std::size_t)kN * kN) {
                std::printf("  [ERR] raw_%s 尺寸异常\n", key.c_str());
                ++g_fail;
                continue;
            }
            ps::pix_t plane[kPh * kPw];
            const char* fields[4] = {"r", "g1", "g2", "b"};
            for (int wh = 0; wh < 4; ++wh) {
                ps::bayer_extract(raw.data(), kN, kN, wh, 0, plane);
                const std::vector<uint16_t> exp = load<uint16_t>(A(key + "_plane_" + fields[wh]));
                const std::string tag = key + " plane " + fields[wh];
                cmp_arr(tag.c_str(), plane, exp.data(), (std::size_t)kPh * kPw);
            }
        }
    }
    std::printf("\n");
}

// ============================================================================
// T3 + T4 + T9 + T8：分块求和 / 行差分 / 口径等价 / 相对误差（单帧）
// ============================================================================
struct SingleResult {
    ps::acc_t acc[3][kBh * kBh];
    ps::dif_t dif[3][kDiffRows * kBh];
    ps::acc_t raw_max[3];
    int bh = 0, bw = 0;
};

static void run_single(const std::string& key, ps::ValuePacking packing,
                       SingleResult* out) {
    const std::vector<uint16_t> raw = load<uint16_t>(A("raw_" + key));
    for (int ci = 0; ci < 3; ++ci) {
        uint64_t mx = 0;
        ps::preprocess_color_frames(raw.data(), 1, kN, kN, kBlock,
                                    kChannels[ci], 0, packing,
                                    out->acc[ci], out->dif[ci],
                                    &out->bh, &out->bw, &mx);
        out->raw_max[ci] = (ps::acc_t)mx;
    }
}

static void t3_t4_t9_single() {
    std::printf("[T3/T4] 分块求和 + 行差分逐值（两口径）\n");
    for (int ci = 0; ci < 3; ++ci) {
        for (int pi = 0; pi < 2; ++pi) {
            const std::string key = std::string(kCases[ci]) + "_" + kPacks[pi];
            SingleResult r;
            run_single(key, pack_of(pi), &r);
            check(r.bh == kBh && r.bw == kBh, "块网格 2x2",
                  "bh=" + std::to_string(r.bh) + " bw=" + std::to_string(r.bw));
            for (int ch = 0; ch < 3; ++ch) {
                const std::vector<uint32_t> es =
                    load<uint32_t>(A(key + "_sum_" + kSumName[ch]));
                cmp_arr((key + " sum " + kSumName[ch]).c_str(), r.acc[ch], es.data(),
                        (std::size_t)(r.bh * r.bw));
                const std::vector<int32_t> ed =
                    load<int32_t>(A(key + "_diff_" + kDiffName[ch]));
                cmp_arr((key + " diff " + kDiffName[ch]).c_str(), r.dif[ch], ed.data(),
                        (std::size_t)(kDiffRows * kBh));
            }
        }
    }

    std::printf("[T9] 口径等价：fullscale == packed << 4（逐值，单帧）\n");
    for (int ci = 0; ci < 3; ++ci) {
        SingleResult p, f;
        const std::string key = kCases[ci];
        run_single(key + "_packed", ps::kPacked, &p);
        run_single(key + "_full", ps::kFullscale, &f);
        bool ok = true;
        for (int ch = 0; ch < 3 && ok; ++ch) {
            for (int i = 0; i < p.bh * p.bw; ++i) {
                if ((uint64_t)f.acc[ch][i] != ((uint64_t)p.acc[ch][i] << 4)) ok = false;
            }
            for (int i = 0; i < kDiffRows * kBh; ++i) {
                if ((int64_t)f.dif[ch][i] != ((int64_t)p.dif[ch][i] << 4)) ok = false;
            }
        }
        check(ok, (std::string("口径等价 ") + key + "：块和与差分均 << 4 成立").c_str(),
              "3 通道");
    }

    std::printf("[T8] 与 float64 参考的相对误差（整数链 vs 参考实现的 block_mean 路径）\n");
    for (int ci = 0; ci < 3; ++ci) {
        const std::string key = std::string(kCases[ci]) + "_packed";
        SingleResult r;
        run_single(key, ps::kPacked, &r);
        double worst = 0.0;
        for (int ch = 0; ch < 3; ++ch) {
            const std::vector<double> ref =
                load<double>(A("ref_float_" + std::string(kCases[ci]) + "_sum_" + kSumName[ch]));
            for (int i = 0; i < r.bh * r.bw; ++i) {
                const double g = (double)r.acc[ch][i];
                const double d = std::fabs(ref[(std::size_t)i] - g) /
                                 std::fmax(std::fabs(g), 1.0);
                if (d > worst) worst = d;
            }
        }
        char buf[128];
        std::snprintf(buf, sizeof(buf), "max rel = %.3e", worst);
        check(worst < 1e-12, (std::string("相对误差 ") + kCases[ci]).c_str(), buf);
        std::printf("         （整数链自身与 golden 是**精确相等**，此处只反映浮点除法舍入）\n");
    }
    std::printf("\n");
}

// ============================================================================
// T5 8 帧累积（1/4/8 帧档）逐值
// ============================================================================
static void t5_frames() {
    std::printf("[T5] 8 帧累积（1/4/8 帧档逐值，两口径）\n");
    for (int pi = 0; pi < 2; ++pi) {
        const std::string key = std::string("frames_mix_") + kPacks[pi];
        const std::vector<uint16_t> frames = load<uint16_t>(A(key));
        if (frames.size() != (std::size_t)kNFrames * kN * kN) {
            std::printf("  [ERR] %s 尺寸异常 %zu\n", key.c_str(), frames.size());
            ++g_fail;
            continue;
        }
        for (std::size_t mi = 0; mi < 3; ++mi) {
            const int mark = kMarks[mi];
            for (int ci = 0; ci < 3; ++ci) {
                ps::acc_t acc[kBh * kBh];
                ps::dif_t dif[kDiffRows * kBh];
                int bh = 0, bw = 0;
                ps::preprocess_color_frames(frames.data(), mark, kN, kN, kBlock,
                                            kChannels[ci], 0, pack_of(pi),
                                            acc, dif, &bh, &bw, nullptr);
                const std::string base = "acc" + std::to_string(mark) +
                                         "_mix_" + kPacks[pi] + "_";
                const std::vector<uint32_t> es =
                    load<uint32_t>(A(base + "sum_" + kSumName[ci]));
                cmp_arr((base + "sum " + kSumName[ci]).c_str(), acc, es.data(),
                        (std::size_t)(bh * bw));
                const std::vector<int32_t> ed =
                    load<int32_t>(A(base + "diff_" + kDiffName[ci]));
                cmp_arr((base + "diff " + kDiffName[ci]).c_str(), dif, ed.data(),
                        (std::size_t)(kDiffRows * kBh));
            }
        }
        // 口径等价的累积侧
        bool ok = true;
        {
            const std::vector<uint16_t> fp = load<uint16_t>(A("frames_mix_packed"));
            const std::vector<uint16_t> ff = load<uint16_t>(A("frames_mix_full"));
            for (int ci = 0; ci < 3 && ok; ++ci) {
                ps::acc_t a_p[kBh * kBh], a_f[kBh * kBh];
                int bh = 0, bw = 0;
                ps::preprocess_color_frames(fp.data(), kNFrames, kN, kN, kBlock,
                                            kChannels[ci], 0, ps::kPacked,
                                            a_p, nullptr, &bh, &bw, nullptr);
                ps::preprocess_color_frames(ff.data(), kNFrames, kN, kN, kBlock,
                                            kChannels[ci], 0, ps::kFullscale,
                                            a_f, nullptr, &bh, &bw, nullptr);
                for (int i = 0; i < bh * bw; ++i) {
                    if ((uint64_t)a_f[i] != ((uint64_t)a_p[i] << 4)) ok = false;
                }
            }
        }
        check(ok, (std::string("口径等价 ") + kPacks[pi] + " 累积侧 << 4 成立").c_str(),
              "8 帧 3 通道");
    }
    std::printf("\n");
}

// ============================================================================
// T6 位宽核算（与实测上界对齐）
// ============================================================================
static void t6_bit_budget() {
    std::printf("[T6] 位宽核算（block=10，block^2=100 -> ceil(log2)=7）\n");
    std::printf("  口径        帧数  源  块和(最小/声明)  累积(最小/声明)  "
                "差分需  双绿累积  双绿差分需  声明24够否\n");
    for (int pi = 0; pi < 2; ++pi) {
        for (int mi = 0; mi < 3; ++mi) {
            const int frames = kMarks[mi];
            const ps::PackingBitBudget b = ps::packing_budget(pack_of(pi), kBlock, frames);
            std::printf("  %-10s  %3d  %3d  %6d/%-6d  %6d/%-6d  %6d  %8d  %10d  %s\n",
                        b.packing, frames, b.src_bits, b.block_sum_min,
                        b.block_sum_declared, b.frame_acc_min, b.frame_acc_declared,
                        b.diff_needed, b.gsum_frame_acc_min, b.gsum_diff_needed,
                        b.diff_fits_gsum ? "够" : "**不够**");
        }
    }
    std::printf("\n");

    // ---- 单通道口径（= configs/fixed_point_b.yaml 的 bit_chain 口径）
    const ps::PackingBitBudget p4 = ps::packing_budget(ps::kPacked, kBlock, 4);
    const ps::PackingBitBudget p8 = ps::packing_budget(ps::kPacked, kBlock, 8);
    check(p4.block_sum_min == 19 && p4.block_sum_declared == 20,
          "packed 块和 19(最小)/20(声明)", "19 / 20");
    check(p4.frame_acc_min == 21 && p4.frame_acc_declared == 22,
          "packed 4 帧累积 21(最小)/22(声明)", "21 / 22");
    check(p8.frame_acc_min == 22 && p8.frame_acc_declared == 23,
          "packed 8 帧累积 22(最小)/23(声明)", "22 / 23");
    check(p8.diff_needed == 23 && p8.diff_fits_single,
          "packed 8 帧差分需 23 <= 声明 24", "余量 1 bit");

    const ps::PackingBitBudget f1 = ps::packing_budget(ps::kFullscale, kBlock, 1);
    const ps::PackingBitBudget f4 = ps::packing_budget(ps::kFullscale, kBlock, 4);
    const ps::PackingBitBudget f8 = ps::packing_budget(ps::kFullscale, kBlock, 8);
    check(f1.block_sum_min == 23 && f1.block_sum_declared == 23,
          "fullscale 块和 23(最小)/23(声明)", "23 / 23");
    check(f4.frame_acc_min == 25 && f4.frame_acc_declared == 25,
          "fullscale 4 帧累积 25", "25");
    check(f8.frame_acc_min == 26 && f8.frame_acc_declared == 26,
          "fullscale 8 帧累积 26（冻结声明值）", "26");
    check(!f4.diff_fits_single && !f8.diff_fits_single,
          "fullscale 单通道差分 24 不够（需 26 / 27）", "差 2 / 3 bit");

    // ---- 双绿口径（真实最坏情形，块和比单通道多 1 bit）
    check(p8.gsum_block_sum_min == 20 && p8.gsum_frame_acc_min == 23 &&
              p8.gsum_diff_needed == 24,
          "packed 双绿：块和 20 / 8 帧累积 23 / 差分需 24",
          "20 / 23 / 24");
    check(p8.diff_fits_gsum && p8.headroom_gsum_bits == 0,
          "packed 双绿差分恰好用满 24 bit（余量 0）", "余量 0");
    check(f8.gsum_block_sum_min == 24 && f8.gsum_frame_acc_min == 27 &&
              f8.gsum_diff_needed == 28,
          "fullscale 双绿：块和 24 / 8 帧累积 27 / 差分需 28",
          "24 / 27 / 28");
    check(f8.headroom_gsum_bits == -4,
          "fullscale 双绿差分比声明 24 bit 差 4 bit", "差 4 bit");

    // ---- 实测上界（用真实向量量出来，与核算对照）
    std::printf("  实测上界（mix 用例：上半幅满量程 / 下半幅零，取 G1+G2 通道）\n");
    std::printf("    口径        帧数  累积最大值       无符号位宽  差分最坏值         有符号位宽\n");
    struct Row { const char* tag; int frames; uint64_t acc; };
    const Row rows[4] = {{"packed", 4, 3276000ULL}, {"packed", 8, 6552000ULL},
                         {"full", 4, 52416000ULL}, {"full", 8, 104832000ULL}};
    bool measured_ok = true;
    for (int i = 0; i < 4; ++i) {
        const std::vector<uint16_t> frames =
            load<uint16_t>(A(std::string("frames_mix_") + rows[i].tag));
        ps::acc_t acc[kBh * kBh];
        ps::dif_t dif[kDiffRows * kBh];
        int bh = 0, bw = 0;
        ps::preprocess_color_frames(frames.data(), rows[i].frames, kN, kN, kBlock,
                                    1 /* G1+G2 */, 0,
                                    (std::strcmp(rows[i].tag, "packed") == 0)
                                        ? ps::kPacked : ps::kFullscale,
                                    acc, dif, &bh, &bw, nullptr);
        uint64_t amax = 0;
        int64_t dmin = 0;
        for (int k = 0; k < bh * bw; ++k) if ((uint64_t)acc[k] > amax) amax = (uint64_t)acc[k];
        for (int k = 0; k < kDiffRows * kBh; ++k) if ((int64_t)dif[k] < dmin) dmin = (int64_t)dif[k];
        std::printf("    %-10s  %3d  %12llu  %10d  %12lld  %10d\n", rows[i].tag,
                    rows[i].frames, (unsigned long long)amax, bits_unsigned(amax),
                    (long long)dmin, bits_signed(dmin));
        if (amax != rows[i].acc) measured_ok = false;
    }
    check(measured_ok, "实测累积上界 == 理论最坏值（4 档全中）",
          "3276000 / 6552000 / 52416000 / 104832000");

    // ---- 冻结声明在双绿口径下的裁决（用实测值算，不靠公式）
    const int need_p8 = bits_signed(-(int64_t)6552000LL);
    const int need_f4 = bits_signed(-(int64_t)52416000LL);
    const int need_f8 = bits_signed(-(int64_t)104832000LL);
    check(need_p8 == 24, "packed 双绿 8 帧差分需 24 bit（= 声明，余量 0）",
          "24");
    check(need_f4 == 27 && need_f8 == 28,
          "fullscale 双绿差分需 27 / 28 bit（> 声明 24）", "27 / 28");
    std::printf("\n");
}

// ============================================================================
// T7 冻结 24 bit 差分通路的饱和演示
// ============================================================================
static void t7_saturation_demo() {
    std::printf("[T7] 冻结 24 bit 有符号差分通路：两口径下的实际后果\n");
    struct Demo { const char* tag; int frames; int64_t diff; };
    const Demo demos[4] = {{"packed", 4, -3276000LL}, {"packed", 8, -6552000LL},
                           {"full", 4, -52416000LL}, {"full", 8, -104832000LL}};
    bool packed_safe = true, full_lossy = true;
    for (int i = 0; i < 4; ++i) {
        const int64_t s = sat_signed(demos[i].diff, 24);
        const double rel = (double)std::llabs(s - demos[i].diff) /
                           (double)std::llabs(demos[i].diff);
        std::printf("    %-8s %d 帧  真实差分 %13lld  24bit 后 %13lld  相对失真 %.3e%s\n",
                    demos[i].tag, demos[i].frames, (long long)demos[i].diff,
                    (long long)s, rel, (s == demos[i].diff) ? "  （未饱和）" : "  **饱和**");
        if (std::strcmp(demos[i].tag, "packed") == 0 && s != demos[i].diff) packed_safe = false;
        if (std::strcmp(demos[i].tag, "full") == 0 && s == demos[i].diff) full_lossy = false;
    }
    check(packed_safe, "packed：24 bit 差分通路不饱和（数值可控）", "4/8 帧均通过");
    check(full_lossy, "fullscale：24 bit 差分通路饱和（静默削顶）", "4/8 帧均失真");
    std::printf("        注意：饱和在图像上只表现为「亮区被削平」，不会报错。\n\n");
}

// ============================================================================
int main(int argc, char** argv) {
    const char* cands[] = {"src/hardware/hwdata", "hwdata", "../hardware/hwdata",
                           "../../src/hardware/hwdata", "."};
    bool found = false;
    if (argc > 1) found = resolve_dir(argv[1]);
    for (int i = 0; !found && i < 5; ++i) found = resolve_dir(cands[i]);
    if (!found) {
        std::printf("[ERR] 找不到 hwdata（含 x6_x7_bitdepth__meta_block.bin）。\n"
                    "      请先运行：python src/model/scripts/make_x6_x7_vectors.py\n");
        return 2;
    }
    std::printf("== X6/X7 主机侧测试：12 bit 双打包口径 ==\n");
    std::printf("  向量目录：%s\n", g_dir.c_str());
    std::printf("  编译期宏：IN=%d BLOCK=%d FRAMES=%d COLOR=%d BAYER=%d "
                "EFF=%d CONTAINER=%d SHIFT=%d\n\n",
                PS_IN_BITS, PS_BLOCK, PS_FRAMES, PS_COLOR, PS_BAYER_PHASE,
                PS_EFFECTIVE_BITS, PS_CONTAINER_BITS, PS_FULLSCALE_SHIFT);

    t1_packing_constants();
    t2_demosaic();
    t3_t4_t9_single();
    t5_frames();
    t6_bit_budget();
    t7_saturation_demo();

    std::printf("==========================================================\n");
    std::printf("X6/X7 结果：PASS %d  FAIL %d\n", g_pass, g_fail);
    std::printf("  ★ 口径结论：packed 与 fullscale 两条链均可跑通，但"
                "**哪一种尚未确定**；\n");
    std::printf("    首帧实拍看饱和帧最大值与低 4 位即可判定：\n");
    std::printf("      最大值 == 4095 且低 4 位不全零 -> packed\n");
    std::printf("      最大值 >  4095 且低 4 位恒零   -> fullscale\n");
    std::printf("    差分通路 24 bit 只在 packed 下安全；fullscale 需加宽到 26~28 bit。\n");
    std::printf("==========================================================\n");
    return g_fail == 0 ? 0 : 1;
}
