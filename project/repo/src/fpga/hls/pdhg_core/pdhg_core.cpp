// ============================================================================
// PenumbraSight M5 重建核（固定迭代原始-对偶 PDHG + TV） · pdhg_core.cpp
//
// 更新式、布局、符号约定与「为什么自带 TV 原语」见 pdhg_core.h 文首。
// 这里只写实现，迭代顺序严格照抄方案 §6.5：
//
//   q^{k+1} = Π_{||q||<=λ}( q^k + σ∇f̄^k )
//   f^{k+1} = Π_{f>=0}( f^k − τ[Bᵀ(Bf^k − d) + ∇ᵀq^{k+1}] )
//   f̄^{k+1} = f^{k+1} + θ(f^{k+1} − f^k)
//
// 三个必须保持的结构特征
//   D1 迭代次数**固定**：没有回溯线搜索、没有数据依赖的内层循环（这正是选 PDHG
//      而不是 FISTA 的核心理由 —— 整条流水可定点化）。
//   D2 只用 recon.hpp 同族的双精度 TV 原语 + lr_operator 的 Bf/Bᵀr，不物化 B。
//   D3 残差每 residual_every 次才算，且只在 max(min_iters, 8) 之后允许早停。
// ============================================================================

#include "pdhg_core.h"

namespace ps {

template <int LANES>
int pdhg_core(const LrFactorsFp& F,
              const double* d,
              const PdhgShapeFp& shape,
              const PdhgParamsFp& P,
              double* f,
              PdhgWorkFp& W,
              PdhgOutFp& out,
              double* res_hist,
              int res_hist_cap) {
    // ---- 结论结构先置零：出错时上报「没迭代过」而不是留脏值 ----
    out.n_used = 0;
    out.early_stopped = 0;
    out.n_res = 0;
    out.monotone = 1;
    out.rho = 0.0;
    out.rho_first = 0.0;
    out.res_norm = 0.0;
    out.d_norm = 0.0;

    const int N = pdhg_shape_n(shape);
    if (F.U == 0 || F.S == 0 || F.V == 0 || d == 0 || f == 0) {
        return kPdhgBadArgsFp;
    }
    if (F.K <= 0 || F.M <= 0 || F.N <= 0 || F.N != N) return kPdhgBadArgsFp;
    if (P.n_iter <= 0 || P.residual_every < 1) return kPdhgBadArgsFp;
    if (!(P.tau > 0.0) || !(P.sigma > 0.0) || !(P.lam >= 0.0)) {
        return kPdhgBadArgsFp;
    }
    if (W.f_prev == 0 || W.fbar == 0 || W.q == 0 || W.grad == 0
        || W.divq == 0 || W.bf == 0 || W.r == 0 || W.btr == 0) {
        return kPdhgBadArgsFp;
    }

    // 早停的迭代下限：§6.5 的最小 8 次是硬下限，调用方只能往上调。
    int min_stop = P.min_iters;
    if (min_stop < kPdhgEarlyStopMinItersFp) min_stop = kPdhgEarlyStopMinItersFp;

    // ---- 初值：f = 0、f̄ = 0、q = 0（都落在可行域内部）----
    for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        f[i] = 0.0;
        W.fbar[i] = 0.0;
    }
    for (int i = 0; i < 2 * N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        W.q[i] = 0.0;
    }

    // ---- 分母 ||d||（只算一次）----
    double d2 = 0.0;
    for (int m = 0; m < F.M; ++m) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        d2 += d[m] * d[m];
    }
    const double d_norm = std::sqrt(d2);
    out.d_norm = d_norm;

    double rho_prev = 0.0;
    int have_prev = 0;

    for (int it = 1; it <= P.n_iter; ++it) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        // ---- 1) 对偶上升 + 投影到 ||q|| <= λ（逐像素 L2 球）----
        pdhg_gradient_fp(W.fbar, shape, W.grad);
        for (int i = 0; i < 2 * N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
            W.q[i] += P.sigma * W.grad[i];
        }
        pdhg_project_l2_ball_fp(W.q, shape, P.lam);

        // ---- 2) Bf 与残差 r = Bf − d ----
        if (lr_forward<LANES>(F, f, W.bf) != kLrOkFp) return kPdhgBadArgsFp;
        for (int m = 0; m < F.M; ++m) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
            W.r[m] = W.bf[m] - d[m];
        }

        // ---- 3) Bᵀr（图像域）----
        if (lr_adjoint<LANES>(F, W.r, W.btr) != kLrOkFp) {
            return kPdhgBadArgsFp;
        }

        // ---- 4) ∇ᵀq：本层 divergence 返回的是 −∇ᵀq，故写 btr − divq ----
        pdhg_divergence_fp(W.q, shape, W.divq);

        // ---- 5) 原始下降 + 非负投影 Π_{f>=0} ----
        for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
            W.f_prev[i] = f[i];                       // 存 f^k 供惯性外推
            double nf = f[i] - P.tau * (W.btr[i] - W.divq[i]);
            if (nf < 0.0) nf = 0.0;                   // §6.5 的非负投影
            f[i] = nf;
        }

        // ---- 6) 惯性外推 f̄ = f + θ(f − f_prev) ----
        for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
            W.fbar[i] = f[i] + P.theta * (f[i] - W.f_prev[i]);
        }

        out.n_used = it;

        // ---- 7) 归一化残差与早停（每 residual_every 次，且不早于 min_stop）----
        if ((it % P.residual_every) == 0) {
            // 残差用**更新后的** f^{it} 度量：ρ_it = ||B f^{it} − d|| / ||d||
            if (lr_forward<LANES>(F, f, W.bf) != kLrOkFp) return kPdhgBadArgsFp;
            double rs = 0.0;
            for (int m = 0; m < F.M; ++m) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
                const double t = W.bf[m] - d[m];
                rs += t * t;
            }
            const double res_norm = std::sqrt(rs);
            const double rho = (d_norm > 0.0) ? (res_norm / d_norm) : res_norm;
            out.res_norm = res_norm;
            out.rho = rho;

            if (res_hist != 0 && out.n_res < res_hist_cap) {
                res_hist[out.n_res] = rho;
            }
            if (out.n_res == 0) out.rho_first = rho;
            ++out.n_res;

            if (have_prev && rho > rho_prev) out.monotone = 0;
            rho_prev = rho;
            have_prev = 1;

            if (P.rho_stop > 0.0 && it >= min_stop && rho <= P.rho_stop) {
                out.early_stopped = 1;
                break;
            }
        }
    }

    return kPdhgOkFp;
}

// ---------------------------------------------------------------- 主机侧标定
#ifndef __SYNTHESIS__
void pdhg_step_sizes(double norm_B, double norm_grad, double safety,
                     double* tau, double* sigma) {
    const double L2 = norm_B * norm_B + norm_grad * norm_grad;
    if (!(L2 > 0.0)) {
        if (tau) *tau = 0.0;
        if (sigma) *sigma = 0.0;
        return;
    }
    const double s = std::sqrt(safety) / std::sqrt(L2);
    if (tau) *tau = s;
    if (sigma) *sigma = s;
}
#endif  // __SYNTHESIS__

// ---------------------------------------------------------------- 实例化
// LANES 取 8 / 16 / 32（A 档 / B 档 / 冲刺档）。
#define PS_INSTANTIATE_PDHG_FP(L)                                             \
    template int pdhg_core<L>(const LrFactorsFp&, const double*,             \
                              const PdhgShapeFp&, const PdhgParamsFp&,        \
                              double*, PdhgWorkFp&, PdhgOutFp&, double*, int);

PS_INSTANTIATE_PDHG_FP(8)
PS_INSTANTIATE_PDHG_FP(16)
PS_INSTANTIATE_PDHG_FP(32)

#undef PS_INSTANTIATE_PDHG_FP

// ---------------------------------------------------------------- HLS 顶层包装
// Vitis HLS 的 top 必须是**具体函数**，不能是模板，故给一个非模板入口。
// LANES 固定为 PS_PDHG_LANES_B_FP（B 档 16）。
int pdhg_core_top(const LrFactorsFp& F,
                  const double* d,
                  const PdhgShapeFp& shape,
                  const PdhgParamsFp& P,
                  double* f,
                  PdhgWorkFp& W,
                  PdhgOutFp& out,
                  double* res_hist,
                  int res_hist_cap) {
    return pdhg_core<PS_PDHG_LANES_B_FP>(F, d, shape, P, f, W, out, res_hist,
                                         res_hist_cap);
}

}  // namespace ps
