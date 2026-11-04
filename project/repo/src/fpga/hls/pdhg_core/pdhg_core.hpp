// ============================================================================
// PenumbraSight M5 重建核（固定迭代原始-对偶 PDHG + TV） · pdhg_core
//
//   pdhg_core.hpp  —— 接口、更新式、定点 TV 原语（本文件）
//   pdhg_core_fixed.cpp —— 可综合的核实现 + LANES 实例化
//                          （_fixed 后缀：同目录另有双精度参考层，
//                           见 src/fpga/hls/readme.md）
//   test_pdhg.cpp  —— 主机侧 testbench（定点 vs 浮点同算法对照）
//
// 依据
// ---------------------------------------------------------------------------
//   方案 §6.2 模块接口（pdhg_core：输入 测量、模型、λ、τ、σ、迭代数；
//                        输出 f、残差历史）
//   方案 §6.5 低秩重建与 TV：
//       q^{k+1} = Π_{||q||<=λ}( q^k + σ∇f̄^k )
//       f^{k+1} = Π_{f>=0}( f^k − τ[Bᵀ(Bf^k − d) + ∇ᵀq^{k+1}] )
//       f̄^{k+1} = f^{k+1} + θ(f^{k+1} − f^k)
//     以及：τ、σ 由 PC 端幂迭代估出的 ||B||₂ 导出；B 档默认 16 次、A 档 8 次；
//     对偶投影用范数（本文件用整数平方根，等价于 §6.5 的"LUT/牛顿一步倒数平方根"
//     但不需要 LUT）；若时序不收敛可降级为各向异性分量钳位；每 4 次迭代算一次
//     归一化残差，只允许在最小 8 次后早停；须分别对照"论文 FISTA 浮点"与
//     "同一 PDHG 浮点"，避免把求解器差异错记成定点损失。
//   configs/fixed_point_b.yaml 的 solver 段（tau/sigma/theta/residual_every/
//     early_stop_min_iters/tv_fallback）
//
// 与既有原语的关系（这是本文件最重要的一段）
// ---------------------------------------------------------------------------
//   ../hardware/recon.hpp 是**双精度参考版**（其文首明写"本实现是双精度参考版，
//   用于主机侧与 Python 逐值对齐"）。pdhg_core 必须是定点，不能直接调它的
//   double 算子，因此本文件给出**同构的整数版**：
//       pdhg_gradient_i32        <-> recon.tv_gradient
//       pdhg_divergence_i32      <-> recon.tv_divergence（同一符号：返回 −∇ᵀq）
//       pdhg_project_l2_ball_i32 <-> recon.tv_project_l2_ball
//       pdhg_clamp_aniso_i32     <-> §6.5 的"各向异性分量钳位"降级方案
//   三条不变式原样继承：
//     * 布局 (H,W) 行 = z、列 = x，扁平索引 = z*W + x；
//       梯度场按 (2,H,W) 展平：g[0..N) 是 z 分量、g[N..2N) 是 x 分量；
//     * Neumann 边界（边缘差分为 0）；
//     * <∇f, q> = −<f, div q>，即 recon.tv_divergence 返回的是 −∇ᵀq，
//       所以原始更新里的 +∇ᵀq 要写成 −div。
//   testbench 用 recon.hpp 的 double 版本作为参照，逐值核对整数版：
//   前向差分在整数码字上**逐位相等**（整数之差不引入舍入），L2 球投影到 1 LSB。
//
// 定点格式与两个容易搞错的量
// ---------------------------------------------------------------------------
//   f、q、∇f̄、∇ᵀq 全部用 §6.3 的 Q6.18（24 bit signed）；d、Bf、r 用 Q2.16。
//   1) **∇f 不越界**：f 经非负投影后 ∈ [0, 2^23)，两码字之差 ∈ (−2^23, 2^23)，
//      正好落在同一个 24 bit signed 格式里 —— 所以 TV 算子不会引入饱和。
//      这一条依赖"f 非负"，若哪天去掉非负投影就必须加宽 ∇ 通路。
//   2) **残差平方和封顶**：Σr² 在真实规模下可达 2^49（|r| ≤ 2^17、M = 9120），
//      而 ρ 的计算要 (Σr² << 16)/Σd² 不能越 64 bit，故把两个平方和**同时**封顶
//      在 2^47（PS_PDHG_SQ_MAX）。封顶只可能发生在 ρ 已远超任何门限时，此时
//      早停本来就不会触发，所以不影响判决；但计数进 score_overflow，如实上报。
//   ρ 用 Q8.8 内部精度（isqrt((Σr²<<16)/Σd²) = ρ·2^8），上报时左移 8 位成 Q0.16
//   （§8.2 的 CONFIDENCE 是 Q0.16）。
//
// 三个通道怎么算
// ---------------------------------------------------------------------------
//   方案 §5.1「RGB 共用同一几何矩阵」：B 只与候选位置有关，与颜色无关。因此
//   本核的三个通道是**同一因子、独立求解**（d_R/d_G/d_B 各解一次），共用一份
//   PdhgWork 暂存。测量侧的三通道共享指数（§6.3）保证通道间相对幅度不被改写，
//   于是三通道残差与 λ 的相对关系是自洽的。
// ============================================================================

#ifndef PENUMBRASIGHT_HLS_PDHG_CORE_HPP
#define PENUMBRASIGHT_HLS_PDHG_CORE_HPP

#include <cstddef>
#include <cstdint>

#include "../../../hardware/recon.hpp"         // ReconShape / recon_n + 双精度 TV 参考
#include "../lr_operator/lr_operator.hpp"    // B 与 Bᵀ
#include "../ps_fixed.hpp"                   // Q 格式、舍入、饱和、isqrt

namespace ps {

// ---------------------------------------------------------------- 编译期参数

// 残差平方和与测量平方和的公共封顶（见文首第 2 条）。2^47 使
// (Σr² << 16) / Σd² <= 2^63，int64 安全。
#ifndef PS_PDHG_SQ_MAX
#define PS_PDHG_SQ_MAX ((1LL << 47) - 1)
#endif

// ρ 的内部 Q 格式
constexpr int kRhoFrac = 8;                  // Q8.8

// 早停的迭代硬下限（§6.5「只允许在最小 8 次后早停」）。与双精度层
// pdhg_core.h 的 kPdhgEarlyStopMinItersFp 取同一宏 PS_PDHG_EARLY_STOP_MIN_ITERS，
// 保证两层同值；调用方把 P.min_iters 调小也不能绕过它。
#ifndef PS_PDHG_EARLY_STOP_MIN_ITERS
#define PS_PDHG_EARLY_STOP_MIN_ITERS 8
#endif
constexpr int kPdhgEarlyStopMinIters = PS_PDHG_EARLY_STOP_MIN_ITERS;

// ---------------------------------------------------------------- 参数/出入

enum PdhgStatus {
    kPdhgOk      = 0,
    kPdhgBadArgs = 2
};

enum PdhgTvMode {
    kTvIsotropicL2 = 0,     // 各向同性：逐像素投到半径 λ 的 L2 球（§6.5 默认）
    kTvAnisoClamp  = 1      // 各向异性降级：分量钳位到 ±λ（§6.5 的 tv_fallback）
};

struct PdhgParams {
    int     n_iter;          // 固定迭代次数（B 档 16，A 档 8，最大 32）
    int     min_iters;       // 允许早停的最小迭代数（§6.5：8）
    int     residual_every;  // 每几次迭代算一次残差（§6.5：4）
    int     tv_mode;         // PdhgTvMode
    int32_t tau_q16;         // 原始步长 Q16.16
    int32_t sigma_q16;       // 对偶步长 Q16.16
    int32_t lambda_q16;      // TV 权重 λ 的 Q16.16
    int32_t theta_q16;       // 惯性外推 θ 的 Q16.16（1.0 -> 65536）
    int32_t rho_stop_q8;     // 归一化残差早停门限（Q8.8）；<= 0 表示不早停
};

// 迭代暂存。G7：综合代码里不许有 new/delete，全部由调用方传入。
//   f_prev / fbar / divq / btr / tmp : N
//   q / grad                         : 2N
//   y / r                            : M
struct PdhgWork {
    int32_t* f_prev;
    int32_t* fbar;
    int32_t* q;
    int32_t* grad;
    int32_t* divq;
    int32_t* y;
    int32_t* r;
    int32_t* btr;
    int32_t* tmp;
};

struct PdhgOut {
    int      n_used;          // 实际迭代次数
    int      early_stopped;   // 1 = 因残差门限提前结束
    int      overflow;        // 1 = 任一级饱和（§6.6：必须体现为不可信）
    uint32_t rho_q8;          // 最后一次测得的 ρ（Q8.8，内部口径）
    uint32_t rho_q16;         // 同上，左移 8 位成 Q0.16（§8.2 上报口径）
    uint64_t res_sq;          // Σ r²（封顶后）
    uint64_t d_sq;            // Σ d²（封顶后）
    long long mac_count;      // 累计 MAC 数
};

// ---------------------------------------------------------------- 定点 TV 原语
// 与 recon.hpp 的同名 double 版本一一对应（见文首）。全部走 narrow_format /
// mul_shift_round，不用 (int) 截断。

// g = ∇f，Neumann 边界。g 长 2N，布局与 recon.tv_gradient 相同。
inline void pdhg_gradient_i32(const int32_t* f, const ReconShape& s, int32_t* g,
                              FixFlags& fl) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE off
#endif
    const int H = s.n_rows, W = s.n_cols, N = H * W;
    for (int i = 0; i < 2 * N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=864 max=864
#endif
        g[i] = 0;
    }
    for (int i = 0; i + 1 < H; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=17 max=17
#endif
        const int32_t* cur = f + (std::size_t)i * W;
        const int32_t* nxt = f + (std::size_t)(i + 1) * W;
        int32_t* dst = g + (std::size_t)i * W;
        for (int j = 0; j < W; ++j) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=24 max=24
#endif
            dst[j] = narrow_format((long long)nxt[j] - (long long)cur[j],
                                   kImageCodeMax, fl);
        }
    }
    for (int i = 0; i < H; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=18 max=18
#endif
        const int32_t* cur = f + (std::size_t)i * W;
        int32_t* dst = g + N + (std::size_t)i * W;
        for (int j = 0; j + 1 < W; ++j) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=23 max=23
#endif
            dst[j] = narrow_format((long long)cur[j + 1] - (long long)cur[j],
                                   kImageCodeMax, fl);
        }
    }
}

// out = divergence(q) = −∇ᵀq（与 recon.tv_divergence 同一符号约定）。
// 原始更新里的 +∇ᵀq 因此写成 −out。中间用 int64 累加后饱和，防 4 项相加越界。
inline void pdhg_divergence_i32(const int32_t* q, const ReconShape& s,
                                int32_t* out, FixFlags& fl) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE off
#endif
    const int H = s.n_rows, W = s.n_cols, N = H * W;
    const int32_t* qz = q;
    const int32_t* qx = q + N;

    for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=432 max=432
#endif
        out[i] = 0;
    }

    for (int i = 0; i + 1 < H; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=17 max=17
#endif
        const int32_t* src = qz + (std::size_t)i * W;
        int32_t* up = out + (std::size_t)i * W;
        int32_t* dn = out + (std::size_t)(i + 1) * W;
        for (int j = 0; j < W; ++j) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=24 max=24
#endif
            up[j] = narrow_format((long long)up[j] - (long long)src[j],
                                  kImageCodeMax, fl);
            dn[j] = narrow_format((long long)dn[j] + (long long)src[j],
                                  kImageCodeMax, fl);
        }
    }
    for (int i = 0; i < H; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=18 max=18
#endif
        const int32_t* src = qx + (std::size_t)i * W;
        int32_t* row = out + (std::size_t)i * W;
        for (int j = 0; j + 1 < W; ++j) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=23 max=23
#endif
            row[j] = narrow_format((long long)row[j] - (long long)src[j],
                                   kImageCodeMax, fl);
            row[j + 1] = narrow_format((long long)row[j + 1] + (long long)src[j],
                                       kImageCodeMax, fl);
        }
    }
    for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=432 max=432
#endif
        out[i] = narrow_format(-(long long)out[i], kImageCodeMax, fl);
    }
}

// 逐像素把 (qz, qx) 投影到半径 radius 的 L2 球（各向同性 TV 的对偶投影）。
// 范数换算成 Q16.16 的比例后一次 mul_shift_round，避免除法进数据通路；
// radius 与 q 都是同一个 Q6.18 格式的**码字**，所以比例是无量纲的。
inline void pdhg_project_l2_ball_i32(int32_t* q, const ReconShape& s,
                                     int64_t radius, FixFlags& fl) {
    const int N = recon_n(s);
    int32_t* qz = q;
    int32_t* qx = q + N;
    if (radius <= 0) {                       // λ = 0 时对偶变量恒为 0
        for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS LOOP_TRIPCOUNT min=432 max=432
#endif
            qz[i] = 0; qx[i] = 0;
        }
        return;
    }
    for (int i = 0; i < N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=432 max=432
#endif
        const int64_t a = qz[i], b = qx[i];
        const uint64_t nrm = isqrt_u64((uint64_t)(a * a + b * b));   // <= 2^24，安全
        if ((int64_t)nrm > radius) {
            // sc = radius / nrm，Q16.16
            const long long sc = ((long long)radius << kGainFrac) / (long long)nrm;
            qz[i] = narrow_format(mul_shift_round(qz[i], sc, kGainFrac),
                                  kImageCodeMax, fl);
            qx[i] = narrow_format(mul_shift_round(qx[i], sc, kGainFrac),
                                  kImageCodeMax, fl);
        }
    }
}

// 各向异性降级：把两个分量各自钳位到 ±radius（不需要开方，时序友好）。
// 被钳位即记账 —— 这条路径只应在 §6.5 的"时序不收敛"降级时启用，
// 报告里必须单独标注（它不再是各向同性 TV）。
inline void pdhg_clamp_aniso_i32(int32_t* q, const ReconShape& s, int64_t radius,
                                 FixFlags& fl) {
    const int N = recon_n(s);
    int64_t r = radius;
    if (r < 0) r = 0;
    if (r > kImageCodeMax) {
        fl.out_overflow++;
        fl.sat_events++;
        r = kImageCodeMax;
    }
    for (int i = 0; i < 2 * N; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#pragma HLS LOOP_TRIPCOUNT min=864 max=864
#endif
        const long long v = saturate_i64((long long)q[i], -r, r);
        if (v != (long long)q[i]) {
            fl.sat_events++;
        }
        q[i] = (int32_t)v;
    }
}

// ρ 的 Q8.8：isqrt((Σr² << 16) / Σd²) = sqrt(Σr²/Σd²)·2^8。
inline uint32_t pdhg_rho_q8(uint64_t res_sq, uint64_t d_sq) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (d_sq == 0) return 0;
    const uint64_t num = res_sq << 16;          // res_sq <= 2^47 -> <= 2^63，安全
    return (uint32_t)isqrt_u64(num / d_sq);
}

// ---------------------------------------------------------------- 核

// 单通道：解 min_{f>=0} ½||Bf − d||² + λ·TV(f)
//   d_code : [M] Q2.16（竖直差分后的测量，已按共享指数归一）
//   f_code : [N] Q6.18，输入初值（通常全 0），输出解
template <int LANES>
int pdhg_solve_channel(const LrFactors& F,
                       const int32_t* d_code,
                       const ReconShape& shape,
                       const PdhgParams& P,
                       int32_t* f_code,
                       PdhgWork& W,
                       PdhgOut& out);

// 三通道：RGB 共用同一几何（同一 U/S/V），独立求解，结果写 f3（通道优先）。
template <int LANES>
int pdhg_core(const LrFactors& F,
              const int32_t* d3,        // [3*M] Q2.16，通道优先
              const ReconShape& shape,
              const PdhgParams& P,
              int32_t* f3,              // [3*N] Q6.18，通道优先
              PdhgWork& W,              // 三通道顺序复用同一份暂存
              PdhgOut out3[3]);

// ---------------------------------------------------------------- 主机侧标定
#ifndef __SYNTHESIS__
// 步长规则：与 quantize.pdhg_step_sizes 同一式子，再叠上 TV 梯度算子的范数上界。
//   τ = σ = sqrt(safety) / sqrt(||B||² + ||∇||²)
// norm_grad = 0 时它**逐值退化**成 quantize.pdhg_step_sizes（只算 ||B||）。
// 为什么必须叠上 ||∇||：原始-对偶的稳定条件是 τσ(||B||² + ||∇||²) < 1；
// 本核的 ∇ 是「2 分量前向差分 + Neumann 边界」，每个分量是 1D 差分矩阵的
// 直和，其 ∇ᵀ∇ 的最大特征值为 4，故 ||∇||² <= 8。只按 ||B|| 定 τ 会在
// λ > 0 时步长偏大 2.9 倍（B 档 ||B||² ~ 0.16 时 sqrt(8.16/0.16) = 7.1 倍）。
// 返回的 τ、σ 已经转成 Q16.16 增益，可直接放进 PdhgParams。
void pdhg_step_sizes_q16(double norm_B, double norm_grad, double safety,
                         int32_t* tau_q16, int32_t* sigma_q16);
#endif  // __SYNTHESIS__

}  // namespace ps

#endif  // PENUMBRASIGHT_HLS_PDHG_CORE_HPP
