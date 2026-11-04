// ============================================================================
// PenumbraSight M5 重建核（固定迭代原始-对偶 PDHG + TV） · pdhg_core.cpp
//
// 更新式、定点格式与"为什么必须叠上 ||∇||² 定步长"见 pdhg_core.hpp 文首。
// 这里只写实现，顺序严格照抄方案 §6.5：
//
//   q^{k+1} = Π_{||q||<=λ}( q^k + σ∇f̄^k )
//   f^{k+1} = Π_{f>=0}( f^k − τ[Bᵀ(Bf^k − d) + ∇ᵀq^{k+1}] )
//   f̄^{k+1} = f^{k+1} + θ(f^{k+1} − f^k)
//
// 三个必须保持的结构特征
//   D1 迭代次数**固定**，无回溯线搜索、无数据依赖循环（这是选 PDHG 而不是
//      FISTA 的核心理由，见 recon.hpp 文首对 X15g 的结论）。
//   D2 每一步都是"乘 Q16.16 增益 + 移位 + 收窄"，全部走 ps_fixed 的偶数舍入 +
//      饱和；没有一处 (int) 截断。
//   D3 残差每 residual_every 次才算，且只在 max(min_iters, 8) 之后允许早停（§6.5）。
//      残差与**双精度层 pdhg_core.cpp 同一口径**：用更新后的 f^{it} 重算
//      r = B f^{it} - d，而不是复用本迭代第 2 步的 r = B f^{it-1} - d。
// ============================================================================

#include "pdhg_core.hpp"

#ifndef __SYNTHESIS__
#include <cmath>
#endif

namespace ps {

namespace {

// 平方和累加 + 封顶（见 .hpp 文首第 2 条）。封顶计数进 score_overflow。
inline uint64_t pdhg_acc_sq(uint64_t acc, int32_t v, FixFlags& fl) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    const uint64_t t = (uint64_t)((long long)v * (long long)v);
    uint64_t s = acc + t;
    if (s < acc || s > (uint64_t)PS_PDHG_SQ_MAX) {
        s = (uint64_t)PS_PDHG_SQ_MAX;
        fl.score_overflow++;
        fl.sat_events++;
    }
    return s;
}

}  // namespace

// ---------------------------------------------------------------- 单通道

template <int LANES>
int pdhg_solve_channel(const LrFactors& F, const int32_t* d_code,
                       const ReconShape& shape, const PdhgParams& P,
                       int32_t* f_code, PdhgWork& W, PdhgOut& out) {
    out.n_used = 0;
    out.early_stopped = 0;
    out.overflow = 0;
    out.rho_q8 = 0;
    out.rho_q16 = 0;
    out.res_sq = 0;
    out.d_sq = 0;
    out.mac_count = 0;

    const int N = recon_n(shape);
    if (F.U == 0 || F.S == 0 || F.V == 0 || d_code == 0 || f_code == 0) {
        return kPdhgBadArgs;
    }
    if (F.N != N || F.M <= 0 || F.K <= 0) return kPdhgBadArgs;
    if (P.n_iter <= 0 || P.n_iter > 4096) return kPdhgBadArgs;
    if (W.f_prev == 0 || W.fbar == 0 || W.q == 0 || W.grad == 0 || W.divq == 0
        || W.y == 0 || W.r == 0 || W.btr == 0 || W.tmp == 0) {
        return kPdhgBadArgs;
    }

    FixFlags fl;
    fix_flags_reset(fl);

    const long long tau = gain_safe((long long)P.tau_q16, fl);
    const long long sig = gain_safe((long long)P.sigma_q16, fl);
    const long long th  = gain_safe((long long)P.theta_q16, fl);
    // 对偶球半径 λ 换算成 Q6.18 码字：radius = λ_real · 2^18 = λ_q16 · 2^(18-16)
    // 两个幂次相除，**精确**，不引入任何舍入。
    const long long lam = gain_safe((long long)P.lambda_q16, fl);
    const long long radius = lam << (kImageFrac - kGainFrac);

    // 早停迭代下限：§6.5 的最小 8 次是硬下限，调用方只能往上调（与双精度层同口径）。
    int min_stop = P.min_iters;
    if (min_stop < kPdhgEarlyStopMinIters) min_stop = kPdhgEarlyStopMinIters;

    // ---- 初值：f = 0、f̄ = 0、q = 0（f >= 0 的可行域内部）----
    for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=432 max=432
#endif
        f_code[i] = 0;
        W.fbar[i] = 0;
    }
    for (int i = 0; i < 2 * N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=864 max=864
#endif
        W.q[i] = 0;
    }

    // ---- 分母 Σd²（一次）----
    uint64_t d_sq = 0;
    for (int m = 0; m < F.M; ++m) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=9120 max=9120
#endif
        d_sq = pdhg_acc_sq(d_sq, d_code[m], fl);
    }

    for (int it = 1; it <= P.n_iter; ++it) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#pragma HLS LOOP_TRIPCOUNT min=16 max=16
#endif
        // ---- 1) 对偶上升 + 投影到 ||q|| <= λ ----
        pdhg_gradient_i32(W.fbar, shape, W.grad, fl);
        for (int i = 0; i < 2 * N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=864 max=864
#endif
            const long long d = mul_shift_round(W.grad[i], sig, kGainFrac);
            W.q[i] = narrow_format((long long)W.q[i] + d, kImageCodeMax, fl);
        }
        if (P.tv_mode == kTvAnisoClamp) {
            pdhg_clamp_aniso_i32(W.q, shape, radius, fl);
        } else {
            pdhg_project_l2_ball_i32(W.q, shape, radius, fl);
        }

        // ---- 2) Bf 与残差 r = Bf − d（Q2.16）----
        {
            LrDiag dgl;
            lr_forward<LANES>(F, f_code, W.y, dgl, 0);
            fix_flags_merge(fl, dgl.flags);
            out.mac_count += dgl.mac_count;
        }
        for (int m = 0; m < F.M; ++m) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=9120 max=9120
#endif
            W.r[m] = narrow_format((long long)W.y[m] - (long long)d_code[m],
                                   kMeasCodeMax, fl);
        }

        // ---- 3) Bᵀr（图像域 Q6.18）----
        {
            LrDiag dga;
            lr_adjoint<LANES>(F, W.r, W.btr, dga, 0);
            fix_flags_merge(fl, dga.flags);
            out.mac_count += dga.mac_count;
        }

        // ---- 4) ∇ᵀq：recon.tv_divergence 给的是 −∇ᵀq，这里取负得到 +∇ᵀq ----
        pdhg_divergence_i32(W.q, shape, W.divq, fl);
        for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=432 max=432
#endif
            W.tmp[i] = narrow_format((long long)W.btr[i] - (long long)W.divq[i],
                                     kImageCodeMax, fl);
        }

        // ---- 5) 原始下降 + 非负投影（Π_{f>=0}）----
        for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=432 max=432
#endif
            W.f_prev[i] = f_code[i];                 // 保存 f^k 供惯性外推用
            const long long st = mul_shift_round(W.tmp[i], tau, kGainFrac);
            long long nf = (long long)f_code[i] - st;
            if (nf < 0) nf = 0;                      // §6.5 的非负投影
            f_code[i] = narrow_format(nf, kImageCodeMax, fl);
        }

        // ---- 6) 惯性外推 f̄ = f + θ(f − f_prev) ----
        for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=432 max=432
#endif
            const long long diff = (long long)f_code[i] - (long long)W.f_prev[i];
            const long long df = mul_shift_round(diff, th, kGainFrac);
            W.fbar[i] = narrow_format((long long)f_code[i] + df, kImageCodeMax, fl);
        }

        out.n_used = it;

        // ---- 7) 残差监测与早停（每 residual_every 次，且不早于 max(min_iters,8)）----
        if (P.residual_every > 0 && (it % P.residual_every) == 0) {
            // 残差用**更新后的** f^{it} 度量（与双精度层 pdhg_core.cpp 同一口径）：
            // 第 2 步的 W.r 是 B f^{it-1} - d，落后一迭代，故在这里重算 Bf^{it}。
            {
                LrDiag dgl;
                lr_forward<LANES>(F, f_code, W.y, dgl, 0);
                fix_flags_merge(fl, dgl.flags);
                out.mac_count += dgl.mac_count;
            }
            for (int m = 0; m < F.M; ++m) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=9120 max=9120
#endif
                W.r[m] = narrow_format((long long)W.y[m] - (long long)d_code[m],
                                       kMeasCodeMax, fl);
            }
            uint64_t rs = 0;
            for (int m = 0; m < F.M; ++m) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=9120 max=9120
#endif
                rs = pdhg_acc_sq(rs, W.r[m], fl);
            }
            out.res_sq = rs;
            out.d_sq = d_sq;
            out.rho_q8 = pdhg_rho_q8(rs, d_sq);
            out.rho_q16 = out.rho_q8 << kRhoFrac;    // Q8.8 -> Q0.16（§8.2 上报口径）
            if (P.rho_stop_q8 > 0 && it >= min_stop
                && out.rho_q8 <= (uint32_t)P.rho_stop_q8) {
                out.early_stopped = 1;
                break;
            }
        }
    }

    out.overflow = fl.any() ? 1 : 0;
    return kPdhgOk;
}

// ---------------------------------------------------------------- 三通道

template <int LANES>
int pdhg_core(const LrFactors& F, const int32_t* d3, const ReconShape& shape,
              const PdhgParams& P, int32_t* f3, PdhgWork& W, PdhgOut out3[3]) {
    if (d3 == 0 || f3 == 0 || out3 == 0) return kPdhgBadArgs;
    const int N = recon_n(shape);
    const int M = F.M;
    for (int c = 0; c < 3; ++c) {
        // RGB 共用同一几何（同一 U/S/V），只是 d 与 f 各一份（§5.1）。
        // 顺序求解 + 复用同一份 PdhgWork：三通道并行会把暂存翻三倍，
        // 而 §6.5 的延迟预算（16 次迭代 <= 300 ms）允许顺序做。
        const int st = pdhg_solve_channel<LANES>(
            F, d3 + (std::size_t)c * M, shape, P,
            f3 + (std::size_t)c * N, W, out3[c]);
        if (st != kPdhgOk) return st;
    }
    return kPdhgOk;
}

// ---------------------------------------------------------------- 主机侧标定
#ifndef __SYNTHESIS__
void pdhg_step_sizes_q16(double norm_B, double norm_grad, double safety,
                         int32_t* tau_q16, int32_t* sigma_q16) {
    // τ = σ = sqrt(safety) / sqrt(||B||² + ||∇||²)
    // norm_grad = 0 时逐值退化成 quantize.pdhg_step_sizes 的
    //   s = sqrt(safety)/||B||（该函数见 quantize.py，safety 默认 0.99）。
    const double L2 = norm_B * norm_B + norm_grad * norm_grad;
    if (!(L2 > 0.0)) {
        if (tau_q16) *tau_q16 = 0;
        if (sigma_q16) *sigma_q16 = 0;
        return;
    }
    const double s = std::sqrt(safety) / std::sqrt(L2);
    if (tau_q16) *tau_q16 = scale_to_gain_q16(s);
    if (sigma_q16) *sigma_q16 = scale_to_gain_q16(s);
}
#endif  // __SYNTHESIS__

// ---------------------------------------------------------------- 实例化
#define PS_INSTANTIATE_PDHG(L)                                                \
    template int pdhg_solve_channel<L>(const LrFactors&, const int32_t*,      \
                                       const ReconShape&, const PdhgParams&,  \
                                       int32_t*, PdhgWork&, PdhgOut&);        \
    template int pdhg_core<L>(const LrFactors&, const int32_t*,               \
                              const ReconShape&, const PdhgParams&, int32_t*, \
                              PdhgWork&, PdhgOut*);

PS_INSTANTIATE_PDHG(8)
PS_INSTANTIATE_PDHG(16)
PS_INSTANTIATE_PDHG(32)

#undef PS_INSTANTIATE_PDHG

}  // namespace ps
