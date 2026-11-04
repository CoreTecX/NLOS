// ============================================================================
// PenumbraSight 低秩算子核（B 与 Bᵀ） · lr_operator.cpp
//
// 接口、布局与「为什么两个方向共用一套原语」见 lr_operator.h 文首。这里只写实现。
//
// 三个必须保持的结构特征
//   L1 **四阶段复用同一 MAC 阵列**：rank_major_rows / rank_major_cols 两个原语，
//      Bf 与 BTr 各调用两次，中间各插一次逐元素乘 S（Task 7 Step 2）。
//   L2 cols 方向只保留 LANES 个累加器：输出按 LANES 分块、块内把 K 行跑完，
//      于是 M = 9120 的测量域向量不需要任何长度 M 的片上暂存。
//   L3 求和顺序与「稠密矩阵乘」逐条相同（行内下标升序、归约下标升序），
//      因此核的结果与稠密 U·diag(S)·Vᵀ 在双精度下只差舍入，能过 1e-9 判据；
//      LANES 只改分块不改顺序，故 8/16/32 逐位相同。
// ============================================================================

#include "lr_operator.h"

namespace ps {

namespace {

// ---------------------------------------------------------------- MAC 原语

// out[k] = Σ_l codes[k*L + l] * x[l]      （codes 视作 K 行 L 列 rank-major）
// 归约在行内，输出长度 K（<= PS_LR_MAX_RANK_FP，由调用方给片上数组）。
template <int LANES>
void rank_major_rows(const double* codes, int K, int L, const double* x,
                     double* out) {
    for (int k = 0; k < K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        const double* row = codes + (std::size_t)k * (std::size_t)L;
        double acc = 0.0;
        for (int l0 = 0; l0 < L; l0 += LANES) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
            double mac[LANES];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=mac complete dim=1
#endif
            for (int t = 0; t < LANES; ++t) mac[t] = 0.0;
            for (int t = 0; t < LANES; ++t) {
                const int l = l0 + t;
                if (l < L) mac[t] = row[l] * x[l];   // 尾块补零，分支不进乘法
            }
            for (int t = 0; t < LANES; ++t) acc += mac[t];
        }
        out[k] = acc;
    }
}

// out[l] = Σ_k codes[k*L + l] * x[k]      （同一个 rank-major 对象的列归约）
// 输出按 LANES 分块：片上一共只有 LANES 个累加器，x[k] 在块内被 LANES 列复用。
template <int LANES>
void rank_major_cols(const double* codes, int K, int L, const double* x,
                     double* out) {
    for (int l0 = 0; l0 < L; l0 += LANES) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        double mac[LANES];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=mac complete dim=1
#endif
        for (int t = 0; t < LANES; ++t) mac[t] = 0.0;
        for (int k = 0; k < K; ++k) {
            const double* row = codes + (std::size_t)k * (std::size_t)L;
            const double xk = x[k];
            for (int t = 0; t < LANES; ++t) {
                const int l = l0 + t;
                if (l < L) mac[t] += row[l] * xk;
            }
        }
        for (int t = 0; t < LANES; ++t) {
            const int l = l0 + t;
            if (l < L) out[l] = mac[t];
        }
    }
}

// 因子结构的最小校验。
inline int lr_check_factors(const LrFactorsFp& F, int need_M, int need_N) {
    if (F.U == 0 || F.S == 0 || F.V == 0) return kLrBadArgsFp;
    if (F.K <= 0 || F.K > PS_LR_MAX_RANK_FP) return kLrBadArgsFp;
    if (F.M <= 0 || F.N <= 0) return kLrBadArgsFp;
    if (need_M > 0 && F.M != need_M) return kLrBadArgsFp;
    if (need_N > 0 && F.N != need_N) return kLrBadArgsFp;
    return kLrOkFp;
}

}  // namespace

// ---------------------------------------------------------------- 前向 Bf

template <int LANES>
int lr_forward(const LrFactorsFp& F, const double* f, double* y) {
    const int st = lr_check_factors(F, 0, 0);
    if (st != kLrOkFp || f == 0 || y == 0) return kLrBadArgsFp;

    double z[PS_LR_MAX_RANK_FP];
    double zs[PS_LR_MAX_RANK_FP];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=z complete dim=1
#pragma HLS ARRAY_PARTITION variable=zs complete dim=1
#endif

    // 阶段 1：z = Vᵀ f（V 视作 K 行 N 列，行内归约）
    rank_major_rows<LANES>(F.V, F.K, F.N, f, z);
    // 阶段 2：zs = S ∘ z（逐元素）
    for (int k = 0; k < F.K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        zs[k] = F.S[k] * z[k];
    }
    // 阶段 3：y = U zs（U 视作 K 行 M 列，列归约 + 输出分块）
    rank_major_cols<LANES>(F.U, F.K, F.M, zs, y);
    return kLrOkFp;
}

// ---------------------------------------------------------------- 伴随 Bᵀr

template <int LANES>
int lr_adjoint(const LrFactorsFp& F, const double* r, double* g) {
    const int st = lr_check_factors(F, 0, 0);
    if (st != kLrOkFp || r == 0 || g == 0) return kLrBadArgsFp;

    double w[PS_LR_MAX_RANK_FP];
    double ws[PS_LR_MAX_RANK_FP];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=w complete dim=1
#pragma HLS ARRAY_PARTITION variable=ws complete dim=1
#endif

    // 阶段 1'：w = Uᵀ r（U 视作 K 行 M 列，行内归约）
    rank_major_rows<LANES>(F.U, F.K, F.M, r, w);
    // 阶段 2'：ws = S ∘ w
    for (int k = 0; k < F.K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        ws[k] = F.S[k] * w[k];
    }
    // 阶段 3'：g = V ws（V 视作 K 行 N 列，列归约 + 输出分块）
    rank_major_cols<LANES>(F.V, F.K, F.N, ws, g);
    return kLrOkFp;
}

// ---------------------------------------------------------------- 实例化
// §6.2「可配置 R 和 LANES」；LANES 取 8/16/32（A 档 / B 档 / 冲刺档）。
#define PS_INSTANTIATE_LR_FP(L)                                               \
    template int lr_forward<L>(const LrFactorsFp&, const double*, double*);   \
    template int lr_adjoint<L>(const LrFactorsFp&, const double*, double*);

PS_INSTANTIATE_LR_FP(8)
PS_INSTANTIATE_LR_FP(16)
PS_INSTANTIATE_LR_FP(32)

#undef PS_INSTANTIATE_LR_FP

// ---------------------------------------------------------------- HLS 顶层包装
// Vitis HLS 的 top 必须是**具体函数**，不能是模板，故各给一个非模板入口。
// 两个方向共用同一套 MAC 原语，所以它们的资源占用对 HLS 是同一份。
int lr_forward_top(const LrFactorsFp& F, const double* f, double* y) {
    return lr_forward<PS_LR_LANES_B_FP>(F, f, y);
}

int lr_adjoint_top(const LrFactorsFp& F, const double* r, double* g) {
    return lr_adjoint<PS_LR_LANES_B_FP>(F, r, g);
}

}  // namespace ps
