// ============================================================================
// PenumbraSight 低秩算子核（B 与 Bᵀ） · lr_operator.cpp
//
// 移位量的推导与布局约定见 lr_operator.hpp 文首。这里只写实现。
//
// 三个必须保持的结构特征
//   L1 **四阶段复用同一 MAC 阵列**：把「K 行 × L 列」的 rank-major 码字矩阵看作
//      同一个对象，只有两种归约方向：
//        rows()  -> out[k] = Σ_l codes[k][l]·x[l]        （Vᵀf、Uᵀr）
//        cols()  -> out[l] = Σ_k codes[k][l]·x[k]        （U·zs、V·ws）
//      四个矩阵乘就是这两个原语的各两次调用，再各插一次逐元素乘 S。
//   L2 cols() 只保留 LANES 个累加器：输出按 LANES 分块、块内把 K 行跑完，
//      于是 M=9120 的测量域向量**不需要**任何长度 M 的片上暂存，增益与收窄
//      也在同一个分块循环里做完（写口就是 AXI-Stream / BRAM 写）。
//   L3 所有量化点走 mul_shift_round / narrow_format（偶数舍入 + 饱和）。
// ============================================================================

#include "lr_operator.hpp"

#ifndef __SYNTHESIS__
#include <cmath>
#include <vector>
#endif

namespace ps {

namespace {

// ---------------------------------------------------------------- MAC 原语

// out[k] = Σ_l codes[k*L + l] · x[l]      （codes 视作 K 行 L 列 rank-major）
// 归约方向沿行内；输出长度 K <= PS_LR_MAX_RANK，由调用方给片上数组。
template <int LANES>
void rank_major_rows(const int16_t* codes, int K, int L, const int32_t* x,
                     int64_t* out, FixFlags& flags, long long& mac_count) {
    for (int k = 0; k < K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        const int16_t* row = codes + (std::size_t)k * (std::size_t)L;
        int64_t acc = 0;
        for (int l0 = 0; l0 < L; l0 += LANES) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
            int64_t mac[LANES];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=mac complete dim=1
#endif
            for (int t = 0; t < LANES; ++t) mac[t] = 0;
            for (int t = 0; t < LANES; ++t) {
                const int l = l0 + t;
                if (l < L) {                       // 尾块补零，不把分支放到乘法上
                    mac[t] = (int64_t)row[l] * (int64_t)x[l];
                }
            }
            for (int t = 0; t < LANES; ++t) acc += mac[t];
        }
        out[k] = sat_mac48(acc, flags);
        mac_count += L;
    }
}

// out[l] = f( Σ_k codes[k*L + l] · x[k] )，f 为「乘 Q16.16 增益 + 右移 + 收窄到声明格式」。
// 输出按 LANES 分块：片上一共只有 LANES 个累加器，x[k] 在块内被 LANES 列复用。
// 这是 L2 的实现，也是把 cols 方向的「增益 + 收窄」融进同一个分块循环的地方。
template <int LANES>
void rank_major_cols_gain(const int16_t* codes, int K, int L, const int32_t* x,
                          long long gain, int shift, long long code_max,
                          int32_t* out, FixFlags& flags, long long& mac_count) {
    for (int l0 = 0; l0 < L; l0 += LANES) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        int64_t mac[LANES];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=mac complete dim=1
#endif
        for (int t = 0; t < LANES; ++t) mac[t] = 0;
        for (int k = 0; k < K; ++k) {
            const int16_t* row = codes + (std::size_t)k * (std::size_t)L;
            const int64_t xk = (int64_t)x[k];
            for (int t = 0; t < LANES; ++t) {
                const int l = l0 + t;
                if (l < L) {
                    mac[t] += (int64_t)row[l] * xk;
                }
            }
            mac_count += LANES;
        }
        for (int t = 0; t < LANES; ++t) {
            const int l = l0 + t;
            if (l < L) {
                const int64_t a = sat_mac48(mac[t], flags);
                const long long v = mul_shift_round(a, gain, shift);
                out[l] = narrow_format(v, code_max, flags);
            }
        }
    }
}

}  // namespace

// ---------------------------------------------------------------- 前向 Bf

template <int LANES>
int lr_forward(const LrFactors& F, const int32_t* f_code, int32_t* y_code,
               LrDiag& dg, LrWork* W) {
    fix_flags_reset(dg.flags);
    dg.mac_count = 0;
    if (F.U == 0 || F.S == 0 || F.V == 0 || f_code == 0 || y_code == 0) {
        return kLrBadArgs;
    }
    if (F.K <= 0 || F.M <= 0 || F.N <= 0 || F.K > PS_LR_MAX_RANK) {
        return kLrBadArgs;
    }
    // 增益：**优先走指数路径**（精确，见 docs/gain_representation.md），
    // exp 无效时退回 Q16.16（旧路径，既有测试走这条）。
    // 指数不是加宽 gain，而是把 scale 折进移位量：scale = 2^exp ->
    // gain 恒为 2^kGainFrac、shift = 常量 - exp。mul_shift_round 已处理
    // 负 shift，故 exp 为正时也正确。
    int sh_u = kShiftUToY, sh_s = kShiftZToS, sh_v = kShiftVToZ;
    long long ug = 0, sg = 0, vg = 0;
    exp_to_gain_shift(F.u_exp, F.u_gain_q16, kShiftUToY, ug, sh_u);
    exp_to_gain_shift(F.s_exp, F.s_gain_q16, kShiftZToS, sg, sh_s);
    exp_to_gain_shift(F.v_exp, F.v_gain_q16, kShiftVToZ, vg, sh_v);
    ug = gain_safe(ug, dg.flags);
    sg = gain_safe(sg, dg.flags);
    vg = gain_safe(vg, dg.flags);

    int64_t acc[PS_LR_MAX_RANK];
    int32_t z[PS_LR_MAX_RANK];
    int32_t zs[PS_LR_MAX_RANK];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=acc complete dim=1
#pragma HLS ARRAY_PARTITION variable=z complete dim=1
#pragma HLS ARRAY_PARTITION variable=zs complete dim=1
#endif

    // ---- 阶段 1：z = Vᵀ f（V 视作 K 行 N 列的 rank-major）----
    rank_major_rows<LANES>(F.V, F.K, F.N, f_code, acc, dg.flags, dg.mac_count);
    for (int k = 0; k < F.K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        const long long v = mul_shift_round(acc[k], vg, sh_v);
        z[k] = narrow_format(v, kImageCodeMax, dg.flags);
    }
    // LrWork 观测通道**仅主机侧**。2026-09-28 实测：把它留在综合视图里，
    // 本函数直接过不了 Syn check ——
    //   ERROR: [HLS 214-134] in function 'int ps::lr_forward<16>(...)':
    //          Pointer to pointer is not supported for variable ''
    // 原因是 LrWork 的成员本身是指针（z/zs/w/ws），核按 `LrWork*` 收它并写
    // `W->z[k]`，在 HLS 的 IR 里就是「指针的指针」。这是**源码级**检查，与调用方
    // 是否真的传了非空指针无关（pdhg_fix 里传的就是字面量 0，同样会报）。
    // 板上不需要这条通道（它只服务于 test_lr_*.cpp 的中间量对照），故整段排除。
#ifndef __SYNTHESIS__
    if (W != 0 && W->z != 0) {
        for (int k = 0; k < F.K; ++k) W->z[k] = z[k];
    }
#endif

    // ---- 阶段 2：zs = S ∘ z（S 是 Q2.14 归一化奇异值）----
    for (int k = 0; k < F.K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        const long long t = (long long)z[k] * (long long)F.S[k];
        const long long v = mul_shift_round(t, sg, sh_s);
        zs[k] = narrow_format(v, kImageCodeMax, dg.flags);
    }
    // 同上：LrWork 观测通道仅主机侧（HLS 214-134，见阶段 1 处的说明）。
#ifndef __SYNTHESIS__
    if (W != 0 && W->zs != 0) {
        for (int k = 0; k < F.K; ++k) W->zs[k] = zs[k];
    }
#endif

    // ---- 阶段 3：y = U @ zs（U 视作 K 行 M 列 rank-major，cols 归约 + 增益收窄）----
    rank_major_cols_gain<LANES>(F.U, F.K, F.M, zs, ug, sh_u,
                                kMeasCodeMax, y_code, dg.flags, dg.mac_count);
    return kLrOk;
}

// ---------------------------------------------------------------- 伴随 Bᵀr

template <int LANES>
int lr_adjoint(const LrFactors& F, const int32_t* r_code, int32_t* g_code,
               LrDiag& dg, LrWork* W) {
    fix_flags_reset(dg.flags);
    dg.mac_count = 0;
    if (F.U == 0 || F.S == 0 || F.V == 0 || r_code == 0 || g_code == 0) {
        return kLrBadArgs;
    }
    if (F.K <= 0 || F.M <= 0 || F.N <= 0 || F.K > PS_LR_MAX_RANK) {
        return kLrBadArgs;
    }
    // 增益：与 lr_forward **同构** —— 优先走指数路径（精确），exp 无效时
    // 退回 Q16.16。两侧必须同时接，否则前向精确、伴随不精确，PDHG 每轮
    // 迭代的 Bᵀr 仍会带着旧的增益误差。
    int sh_u = kShiftUToW, sh_s = kShiftZToS, sh_v = kShiftVToG;
    long long ug = 0, sg = 0, vg = 0;
    exp_to_gain_shift(F.u_exp, F.u_gain_q16, kShiftUToW, ug, sh_u);
    exp_to_gain_shift(F.s_exp, F.s_gain_q16, kShiftZToS, sg, sh_s);
    exp_to_gain_shift(F.v_exp, F.v_gain_q16, kShiftVToG, vg, sh_v);
    ug = gain_safe(ug, dg.flags);
    sg = gain_safe(sg, dg.flags);
    vg = gain_safe(vg, dg.flags);

    int64_t acc[PS_LR_MAX_RANK];
    int32_t w[PS_LR_MAX_RANK];
    int32_t ws[PS_LR_MAX_RANK];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=acc complete dim=1
#pragma HLS ARRAY_PARTITION variable=w complete dim=1
#pragma HLS ARRAY_PARTITION variable=ws complete dim=1
#endif

    // ---- 阶段 1'：w = Uᵀ r（U 视作 K 行 M 列 rank-major，rows 归约）----
    rank_major_rows<LANES>(F.U, F.K, F.M, r_code, acc, dg.flags, dg.mac_count);
    for (int k = 0; k < F.K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        const long long v = mul_shift_round(acc[k], ug, sh_u);
        w[k] = narrow_format(v, kImageCodeMax, dg.flags);
    }
    // 同上：LrWork 观测通道仅主机侧（HLS 214-134，见 lr_forward 阶段 1 处的说明）。
#ifndef __SYNTHESIS__
    if (W != 0 && W->w != 0) {
        for (int k = 0; k < F.K; ++k) W->w[k] = w[k];
    }
#endif

    // ---- 阶段 2'：ws = S ∘ w ----
    for (int k = 0; k < F.K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        const long long t = (long long)w[k] * (long long)F.S[k];
        const long long v = mul_shift_round(t, sg, sh_s);
        ws[k] = narrow_format(v, kImageCodeMax, dg.flags);
    }
    // 同上：LrWork 观测通道仅主机侧（HLS 214-134，见 lr_forward 阶段 1 处的说明）。
#ifndef __SYNTHESIS__
    if (W != 0 && W->ws != 0) {
        for (int k = 0; k < F.K; ++k) W->ws[k] = ws[k];
    }
#endif

    // ---- 阶段 3'：g = V @ ws（V 视作 K 行 N 列 rank-major，cols 归约 + 增益收窄）----
    rank_major_cols_gain<LANES>(F.V, F.K, F.N, ws, vg, sh_v,
                                kImageCodeMax, g_code, dg.flags, dg.mac_count);
    return kLrOk;
}

// ---------------------------------------------------------------- 主机侧工具
#ifndef __SYNTHESIS__

void lr_codes_to_dense_B(const LrFactors& F, double* B_dense) {
    // B[m, i] = Σ_k U[m,k]·S[k]·V[i,k]，用 Q16.16 增益还原 scale。
    // 只在主机/PS 侧调用：板端不物化 B（方案 §5.2 的体积理由）。
    const double ug = gain_q16_to_double(F.u_gain_q16);
    const double sg = gain_q16_to_double(F.s_gain_q16);
    const double vg = gain_q16_to_double(F.v_gain_q16);
    for (int m = 0; m < F.M; ++m) {
        for (int i = 0; i < F.N; ++i) {
            double s = 0.0;
            for (int k = 0; k < F.K; ++k) {
                const double u = (double)F.U[(std::size_t)k * F.M + m] * ug;
                const double sk = (double)F.S[k] * sg;
                const double v = (double)F.V[(std::size_t)k * F.N + i] * vg;
                s += u * sk * v;
            }
            B_dense[(std::size_t)m * F.N + i] = s;
        }
    }
}

double lr_spectral_norm_power(const LrFactors& F, int iters, double tol,
                              uint64_t seed, int* iters_used, double* delta) {
    // 与 quantize.power_iteration_norm 同口径：只要求算子（Bv 与 Bᵀw），
    // 不物化 B。返回 sqrt(λ)，λ = ||BᵀBv||/||v|| 的收敛值。
    if (iters_used) *iters_used = 0;
    if (delta) *delta = 0.0;
    if (F.K <= 0 || F.M <= 0 || F.N <= 0) return 0.0;

    const double ug = gain_q16_to_double(F.u_gain_q16);
    const double sg = gain_q16_to_double(F.s_gain_q16);
    const double vg = gain_q16_to_double(F.v_gain_q16);

    std::vector<double> v(F.N), t(F.K), y(F.M), s2(F.K), w(F.N);
    uint64_t st = seed ? seed : 1;
    for (int i = 0; i < F.N; ++i) {
        st += 0x9E3779B97F4A7C15ull;
        uint64_t z = st;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= (z >> 31);
        v[i] = (double)(z >> 11) * (1.0 / 9007199254740992.0) - 0.5;
    }
    double nv = 0.0;
    for (int i = 0; i < F.N; ++i) nv += v[i] * v[i];
    nv = std::sqrt(nv);
    if (nv <= 0.0) return 0.0;
    for (int i = 0; i < F.N; ++i) v[i] /= nv;

    double lam_prev = 0.0, dl = 1.0;
    int used = 0;
    for (int it = 1; it <= iters; ++it) {
        // t = S ∘ (Vᵀ v)
        for (int k = 0; k < F.K; ++k) {
            double a = 0.0;
            const int16_t* row = F.V + (std::size_t)k * F.N;
            for (int i = 0; i < F.N; ++i) {
                a += (double)row[i] * vg * v[i];
            }
            t[k] = a * (double)F.S[k] * sg;
        }
        // y = U t
        for (int m = 0; m < F.M; ++m) {
            double a = 0.0;
            for (int k = 0; k < F.K; ++k) {
                a += (double)F.U[(std::size_t)k * F.M + m] * ug * t[k];
            }
            y[m] = a;
        }
        // s2 = S ∘ (Uᵀ y)
        for (int k = 0; k < F.K; ++k) {
            double a = 0.0;
            const int16_t* row = F.U + (std::size_t)k * F.M;
            for (int m = 0; m < F.M; ++m) {
                a += (double)row[m] * ug * y[m];
            }
            s2[k] = a * (double)F.S[k] * sg;
        }
        // w = V s2
        for (int i = 0; i < F.N; ++i) {
            double a = 0.0;
            for (int k = 0; k < F.K; ++k) {
                a += (double)F.V[(std::size_t)k * F.N + i] * vg * s2[k];
            }
            w[i] = a;
        }
        double nw = 0.0;
        for (int i = 0; i < F.N; ++i) nw += w[i] * w[i];
        nw = std::sqrt(nw);
        if (nw <= 0.0) return 0.0;
        for (int i = 0; i < F.N; ++i) v[i] = w[i] / nw;
        const double lam = nw;                       // ||BᵀBv|| / ||v||
        dl = std::fabs(lam - lam_prev) / (std::fabs(lam) > 1e-300 ? std::fabs(lam) : 1e-300);
        lam_prev = lam;
        used = it;
        if (dl < tol) break;
    }
    if (iters_used) *iters_used = used;
    if (delta) *delta = dl;
    return std::sqrt(lam_prev > 0.0 ? lam_prev : 0.0);
}

#endif  // __SYNTHESIS__

// ---------------------------------------------------------------- 实例化
// 方案 §6.2「可配置 R 和 LANES」；LANES 取 8/16/32（A 档 / B 档 / 冲刺档）。
#define PS_INSTANTIATE_LR(L)                                                  \
    template int lr_forward<L>(const LrFactors&, const int32_t*, int32_t*,    \
                               LrDiag&, LrWork*);                             \
    template int lr_adjoint<L>(const LrFactors&, const int32_t*, int32_t*,    \
                               LrDiag&, LrWork*);

PS_INSTANTIATE_LR(8)
PS_INSTANTIATE_LR(16)
PS_INSTANTIATE_LR(32)

#undef PS_INSTANTIATE_LR

}  // namespace ps
