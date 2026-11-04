// ============================================================================
// lr_operator 主机侧 testbench · tb.cpp
//
// 编译并运行（在 src/fpga/hls/lr_operator 目录下）：
//   g++ -std=c++17 -Wall -Wextra lr_operator.cpp tb.cpp -o tb.exe
//   ./tb.exe
//
// 自检内容（合成数据，真跑真比）：
//   B1 model.psm 候选记录 -> 因子还原（只走 psm_format.h 访问器）
//   B2 **Bf 与稠密 B = U·diag(S)·Vᵀ 相乘一致**，要求相对误差 <= 1e-9
//   B3 伴随 Bᵀr 与稠密 Bᵀr 一致（同上判据）
//   B4 伴随恒等式 <Bf, r> == <f, Bᵀr>
//   B5 R 可配置（R = 4 / 8 各自成立，且 R=8 与 R=4 的 B 不同）
//   B6 LANES = 8/16/32 逐位一致
// ============================================================================

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>

#include "lr_operator.h"

namespace {

const int kMeasH = 9;
const int kMeasW = 10;
const int kHiddenH = 6;
const int kHiddenW = 8;
const int kRankMax = 8;

const double kUScale = 0.0009765625;      // 2^-10（2 的幂 -> code*scale 精确）
const double kSScale = 0.00006103515625;  // 2^-14
const double kVScale = 0.0009765625;      // 2^-10

struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed ? seed : 1) {}
    uint64_t next() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return s;
    }
    int16_t code() { return (int16_t)((int)((next() >> 17) % 6001u) - 3000); }
    uint16_t ucode() { return (uint16_t)((next() >> 17) % 8001u + 800u); }
    double unit() { return (double)(next() >> 11) / 9007199254740992.0; }
};

void wr_i16(std::vector<uint8_t>& b, uint64_t off, int16_t v) {
    std::memcpy(&b[(std::size_t)off], &v, sizeof(v));
}
void wr_u16(std::vector<uint8_t>& b, uint64_t off, uint16_t v) {
    std::memcpy(&b[(std::size_t)off], &v, sizeof(v));
}

// 显式稠密 B = U·diag(S)·Vᵀ（M 行 N 列，行主序）。仅供 tb/PS 侧核对，
// 板端不物化 B（方案 §5.2 的体积理由）。
void dense_B(const ps::LrFactorsFp& F, double* B) {
    for (int m = 0; m < F.M; ++m) {
        for (int n = 0; n < F.N; ++n) {
            double acc = 0.0;
            for (int k = 0; k < F.K; ++k) {
                acc += F.U[(std::size_t)k * F.M + m] * F.S[k]
                     * F.V[(std::size_t)k * F.N + n];
            }
            B[(std::size_t)m * F.N + n] = acc;
        }
    }
}

// rel = max|a-b| / max|b|
double max_rel(const std::vector<double>& a, const std::vector<double>& b) {
    double num = 0.0, den = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i];
        const double ad = (d < 0.0) ? -d : d;
        if (ad > num) num = ad;
    }
    for (std::size_t i = 0; i < b.size(); ++i) {
        const double ab = (b[i] < 0.0) ? -b[i] : b[i];
        if (ab > den) den = ab;
    }
    return (den > 0.0) ? num / den : num;
}

}  // namespace

int main() {
    std::printf("=== lr_operator tb（方案 §6.5 低秩前向 / 伴随）===\n\n");

    // ------------------------------------------------------------ 造 model.psm
    ps_psm_header_t h;
    std::memset(&h, 0, sizeof(h));
    std::memcpy(h.magic, PS_PSM_MAGIC, 4);
    h.version = PS_PSM_VERSION;
    h.header_bytes = (uint16_t)PS_PSM_HEADER_BYTES;
    h.meas_h = (uint16_t)kMeasH;
    h.meas_w = (uint16_t)kMeasW;
    h.hidden_h = (uint16_t)kHiddenH;
    h.hidden_w = (uint16_t)kHiddenW;
    h.candidates = 1;
    h.rank_max = (uint16_t)kRankMax;
    h.rank_default = (uint16_t)kRankMax;
    h.loc_rank = 4;
    const uint64_t M = ps_psm_meas_len(&h);
    const uint64_t N = ps_psm_hidden_len(&h);
    const uint64_t rec = ps_psm_record_bytes_hdr(&h);
    h.payload_bytes = rec;

    std::vector<uint8_t> file((std::size_t)(PS_PSM_HEADER_BYTES + rec), 0);
    std::memcpy(file.data(), &h, sizeof(h));

    Rng rng(0x243F6A8885A308D3ull);
    {
        const uint64_t base = ps_psm_candidate_offset(&h, 0);
        ps_psm_candidate_hdr_t ch;
        std::memset(&ch, 0, sizeof(ch));
        ch.u_scale = (float)kUScale;
        ch.s_scale = (float)kSScale;
        ch.v_scale = (float)kVScale;
        std::memcpy(&file[(std::size_t)base], &ch, sizeof(ch));

        const uint64_t u_off = base + ps_psm_u_offset(h.rank_max, M);
        const uint64_t s_off = base + ps_psm_s_offset(h.rank_max, M);
        const uint64_t v_off = base + ps_psm_v_offset(h.rank_max, M);
        for (int k = 0; k < kRankMax; ++k) {
            for (uint64_t m = 0; m < M; ++m) {
                wr_i16(file, u_off + ((uint64_t)k * M + m) * 2u, rng.code());
            }
            wr_u16(file, s_off + (uint64_t)k * 2u, rng.ucode());
            for (uint64_t n = 0; n < N; ++n) {
                wr_i16(file, v_off + ((uint64_t)k * N + n) * 2u, rng.code());
            }
        }
    }

    // ------------------------------------------------------------ B1 因子还原
    std::printf("--- B1 因子还原（psm_format.h 访问器）---\n");
    const int chk = ps_psm_check_header(&h, 0, (uint32_t)kMeasH, (uint32_t)kMeasW,
                                       (uint32_t)kHiddenH, (uint32_t)kHiddenW);
    std::printf("  头部自检 -> %d（0 = PS_PSM_OK）  M=%llu  N=%llu  record=%llu\n",
                chk, (unsigned long long)M, (unsigned long long)N,
                (unsigned long long)rec);
    if (chk != PS_PSM_OK) { std::printf("[FAIL] 头部不合法\n"); return 1; }

    const uint8_t* cand = file.data() + ps_psm_candidate_offset(&h, 0);
    std::vector<double> U8((std::size_t)kRankMax * M), S8((std::size_t)kRankMax),
        V8((std::size_t)kRankMax * N);
    std::vector<double> U4((std::size_t)4 * M), S4(4), V4((std::size_t)4 * N);
    ps::LrFactorsFp F8, F4;
    int st = ps::lr_factors_from_psm(cand, &h, kRankMax, U8.data(), S8.data(),
                                     V8.data(), F8);
    int st4 = ps::lr_factors_from_psm(cand, &h, 4, U4.data(), S4.data(),
                                      V4.data(), F4);
    std::printf("  lr_factors_from_psm: R=8 -> %d，R=4 -> %d（0 = kLrOkFp）\n",
                st, st4);
    std::printf("  真实系数抽样：U[0][0]=%.6e  S[0]=%.6e  V[0][0]=%.6e\n",
                F8.U[0], F8.S[0], F8.V[0]);
    if (st != ps::kLrOkFp || st4 != ps::kLrOkFp) {
        std::printf("[FAIL] 因子还原失败\n");
        return 1;
    }

    // ------------------------------------------------------------ B2/B3/B4
    std::printf("\n--- B2/B3/B4 Bf 与 Bᵀr 对稠密 U·diag(S)·Vᵀ ---\n");
    std::vector<double> f((std::size_t)N), r((std::size_t)M);
    for (std::size_t i = 0; i < f.size(); ++i) f[i] = rng.unit() - 0.5;
    for (std::size_t i = 0; i < r.size(); ++i) r[i] = rng.unit() - 0.5;

    int fails = 0;
    for (int pass = 0; pass < 2; ++pass) {
        const ps::LrFactorsFp& F = (pass == 0) ? F8 : F4;
        const int R = F.K;

        std::vector<double> B((std::size_t)F.M * F.N);
        dense_B(F, B.data());

        // y_ref = B f（稠密，M x N 乘 N 向量）
        std::vector<double> y_ref((std::size_t)F.M, 0.0);
        for (int m = 0; m < F.M; ++m) {
            double acc = 0.0;
            for (int n = 0; n < F.N; ++n) acc += B[(std::size_t)m * F.N + n] * f[(std::size_t)n];
            y_ref[(std::size_t)m] = acc;
        }
        // g_ref = Bᵀ r（稠密）
        std::vector<double> g_ref((std::size_t)F.N, 0.0);
        for (int n = 0; n < F.N; ++n) {
            double acc = 0.0;
            for (int m = 0; m < F.M; ++m) acc += B[(std::size_t)m * F.N + n] * r[(std::size_t)m];
            g_ref[(std::size_t)n] = acc;
        }

        std::vector<double> y((std::size_t)F.M, 0.0), g((std::size_t)F.N, 0.0);
        ps::lr_forward<16>(F, f.data(), y.data());
        ps::lr_adjoint<16>(F, r.data(), g.data());

        const double ey = max_rel(y, y_ref);
        const double eg = max_rel(g, g_ref);
        std::printf("  R=%d：dense B %d x %d\n", R, F.M, F.N);
        std::printf("    y[0] = %.15e   稠密 y_ref[0] = %.15e\n", y[0], y_ref[0]);
        std::printf("    y[%d] = %.15e   稠密 y_ref[%d] = %.15e\n", F.M - 1,
                    y[(std::size_t)F.M - 1], F.M - 1, y_ref[(std::size_t)F.M - 1]);
        std::printf("    Bf  相对误差（max|Δ|/max|dense|）= %.3e\n", ey);
        std::printf("    Bᵀr 相对误差（max|Δ|/max|dense|）= %.3e\n", eg);
        if (ey <= 1e-9) {
            std::printf("    [ok  ] Bf 与稠密 U·diag(S)·Vᵀ 一致（<= 1e-9）\n");
        } else {
            std::printf("    [FAIL] Bf 相对误差 %.3e 超过 1e-9\n", ey);
            ++fails;
        }
        if (eg <= 1e-9) {
            std::printf("    [ok  ] Bᵀr 与稠密 Bᵀ 一致（<= 1e-9）\n");
        } else {
            std::printf("    [FAIL] Bᵀr 相对误差 %.3e 超过 1e-9\n", eg);
            ++fails;
        }

        // B4 伴随恒等式 <Bf, r> == <f, Bᵀr>
        // 注意分母不能用 <Bf,r> 本身：R=8 时它只有 1.6e-3，而逐项 |y_m r_m| 之和
        // 大得多（近相消），用结果作分母会把 1e-14 的绝对差夸大成 6.6e-12 的
        // "相对差"。这里按标准做法用**项绝对值之和**作分母。
        double lhs = 0.0, rhs = 0.0, scale = 0.0;
        for (int m = 0; m < F.M; ++m) {
            const double t = y[(std::size_t)m] * r[(std::size_t)m];
            lhs += t;
            scale += (t < 0.0) ? -t : t;
        }
        for (int n = 0; n < F.N; ++n) {
            const double t = f[(std::size_t)n] * g[(std::size_t)n];
            rhs += t;
            scale += (t < 0.0) ? -t : t;
        }
        const double ad = (lhs - rhs < 0.0) ? (rhs - lhs) : (lhs - rhs);
        const double rel = (scale > 0.0) ? (ad / scale) : ad;
        std::printf("    <Bf, r> = %.15e   <f, Bᵀr> = %.15e\n", lhs, rhs);
        std::printf("    |<Bf,r> - <f,Bᵀr>| = %.3e，分母 Σ|项| = %.3e，相对差 = %.3e\n",
                    ad, scale, rel);
        if (rel <= 1e-12) {
            std::printf("    [ok  ] 伴随恒等式成立\n");
        } else {
            std::printf("    [FAIL] 伴随恒等式不成立\n");
            ++fails;
        }
    }

    // B5：R=8 与 R=4 的 B 必须不同（否则"可配置 R"没生效）
    {
        std::vector<double> B8((std::size_t)F8.M * F8.N), B4((std::size_t)F4.M * F4.N);
        dense_B(F8, B8.data());
        dense_B(F4, B4.data());
        double dB = 0.0;
        for (std::size_t i = 0; i < B8.size(); ++i) {
            const double d = B8[i] - B4[i];
            if ((d < 0 ? -d : d) > dB) dB = (d < 0 ? -d : d);
        }
        std::printf("\n--- B5 R 可配置 ---\n  max|B(R=8) - B(R=4)| = %.6e\n", dB);
        if (dB > 0.0) {
            std::printf("  [ok  ] R 确实参与运算（R=4 只用前 4 个 rank）\n");
        } else {
            std::printf("  [FAIL] 改 R 没有改变结果\n");
            ++fails;
        }
    }

    // ------------------------------------------------------------ B6 LANES 不变性
    std::printf("\n--- B6 LANES = 8 / 16 / 32 逐位一致性 ---\n");
    std::vector<double> y8((std::size_t)F8.M), y16((std::size_t)F8.M),
        y32((std::size_t)F8.M);
    std::vector<double> g8((std::size_t)F8.N), g16((std::size_t)F8.N),
        g32((std::size_t)F8.N);
    ps::lr_forward<8>(F8, f.data(), y8.data());
    ps::lr_forward<16>(F8, f.data(), y16.data());
    ps::lr_forward<32>(F8, f.data(), y32.data());
    ps::lr_adjoint<8>(F8, r.data(), g8.data());
    ps::lr_adjoint<16>(F8, r.data(), g16.data());
    ps::lr_adjoint<32>(F8, r.data(), g32.data());
    int dy = 0, dg = 0;
    for (int m = 0; m < F8.M; ++m) {
        if (y8[(std::size_t)m] != y16[(std::size_t)m]) ++dy;
        if (y32[(std::size_t)m] != y16[(std::size_t)m]) ++dy;
    }
    for (int n = 0; n < F8.N; ++n) {
        if (g8[(std::size_t)n] != g16[(std::size_t)n]) ++dg;
        if (g32[(std::size_t)n] != g16[(std::size_t)n]) ++dg;
    }
    std::printf("  与 LANES=16 不等的元素数：Bf -> %d，Bᵀr -> %d\n", dy, dg);
    if (dy == 0 && dg == 0) {
        std::printf("  [ok  ] 8/16/32 逐位相同（分块不改求和顺序）\n");
    } else {
        std::printf("  [FAIL] LANES 改变了结果\n");
        ++fails;
    }

    // 算力记账（方案 §9.2 要求报告 MAC 数）
    const long long mac_fwd = (long long)F8.K * F8.N + (long long)F8.K * F8.M;
    const long long mac_adj = (long long)F8.K * F8.M + (long long)F8.K * F8.N;
    std::printf("\n--- 算力记账 ---\n");
    std::printf("  Bf：R*N + R*M = %lld + %lld = %lld 个标量乘\n",
                (long long)F8.K * F8.N, (long long)F8.K * F8.M, mac_fwd);
    std::printf("  Bᵀr：R*M + R*N = %lld + %lld = %lld 个标量乘\n",
                (long long)F8.K * F8.M, (long long)F8.K * F8.N, mac_adj);
    std::printf("  稠密对照：B 为 %d x %d = %d 项（本核不物化 B）\n",
                F8.M, F8.N, F8.M * F8.N);

    std::printf("\n=== 结论：%s（失败项 %d）===\n",
                fails == 0 ? "全部通过" : "有失败项", fails);
    return fails == 0 ? 0 : 1;
}
