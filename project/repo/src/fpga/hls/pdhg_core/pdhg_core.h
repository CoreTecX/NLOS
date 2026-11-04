// ============================================================================
// PenumbraSight M5 重建核（固定迭代原始-对偶 PDHG + TV） · pdhg_core.h
//
//   pdhg_core.h  —— 接口、更新式、双精度 TV 原语（本文件）
//   pdhg_core.cpp—— 可综合的核实现 + LANES 显式实例化
//   tb.cpp       —— 主机侧 testbench（残差历史 / 单调性 / 早停 / 非负性）
//
// 设计目标：同一份源码在两个环境下都能工作（照抄 src/hardware/recon.hpp 的约定）
//   g++       : 主机侧功能验证；本目录综合专用 pragma 全包在 #ifdef __SYNTHESIS__
//   Vitis HLS : 综合（工具自己定义 __SYNTHESIS__）
//
// 依据
// ---------------------------------------------------------------------------
//   教师方案 §6.5 低秩重建与 TV —— 固定迭代原始-对偶更新：
//       q^{k+1} = Π_{||q||<=λ}( q^k + σ·∇f̄^k )
//       f^{k+1} = Π_{f>=0}( f^k − τ·[ Bᵀ(B f^k − d) + ∇ᵀ q^{k+1} ] )
//       f̄^{k+1} = f^{k+1} + θ·( f^{k+1} − f^k )        （惯性外推）
//     以及三条调度约束：
//       * τ、σ 由 PC 端幂迭代估出的 ||B||₂ 导出（§6.5 第 1 条）；
//       * **每 4 次迭代计算一次归一化残差；只允许在最小 8 次后早停**；
//       * 「对偶投影的范数用 LUT/牛顿一步倒数平方根实现」（本层是双精度参考，
//         直接开方；定点层见同目录 pdhg_core_fixed.cpp）。
//   教师方案 §6.2 模块接口：pdhg_core 吃 测量、模型、λ、τ、σ、迭代数，吐 f 与残差历史。
//
// 与 src/hardware/recon.hpp 的关系（本文件最重要的一段）
// ---------------------------------------------------------------------------
//   recon.hpp 是**双精度参考版**（其文首明写"用于主机侧与 Python 逐值对齐"），
//   里面已经有 TV 的三个原语：tv_gradient / tv_divergence / tv_project_l2_ball。
//   本层与它同口径，因此**刻意重抄**这三个原语（前缀 pdhg_），逐行保持同一算式、
//   同一布局、同一符号约定，只做一件事：把综合 pragma 包进 #ifdef __SYNTHESIS__。
//
//   为什么是"重抄"而不是 `#include "../../../hardware/recon.hpp"`：
//   recon.hpp / geometry.hpp / transport.hpp 用的是**裸** `#pragma HLS`，g++ 在
//   -Wall -Wextra 下会对每一处报 `-Wunknown-pragmas`（实测 43 条），而任务要求
//   三个 tb 能用 `g++ -std=c++17 -Wall -Wextra` 干净编译。故本层自带这三个原语。
//   等价性不是声明出来的，是可验证的：tb.cpp 里有一个 `-DPS_PDHG_XCHECK_RECON`
//   开关，打开后直接 `#include recon.hpp` 并把本层结果与 ps::tv_* 逐值相减
//   （见 tb 的 C5 段）。
//
//   三条不变式原样继承：
//     * 布局：隐藏图 (H, W)，**行索引 = z、列索引 = x**，扁平索引 = z*W + x；
//       梯度场按 (2, H, W) 展平 —— g[0, N) 是 z 分量、g[N, 2N) 是 x 分量；
//     * Neumann 边界：边缘差分为 0；
//     * <∇f, q> = −<f, div q>，即 pdhg_divergence_fp 返回的是 **−∇ᵀq**；
//       所以原始更新里的 `+∇ᵀq^{k+1}` 要写成 `−divergence(q)`（代码里是
//       `btr[i] - divq[i]`，因为 divq 已经是 −∇ᵀq）。
//
// 归一化残差与早停（§6.5 第 4 条）
// ---------------------------------------------------------------------------
//   ρ_k = || B f^k − d ||₂ / || d ||₂                （逐像素无权重、全局一个标量）
//   只在 k % residual_every == 0 时计算；residual_every 建议 4。
//   早停条件：ρ_k <= rho_stop **且** k >= min_iters **且** k >= 8。
//   最后一条（8）是 §6.5 的硬约束，写成编译期常量 kPdhgEarlyStopMinItersFp，
//   调用方即使把 min_iters 调小也不会绕过它。
//
// 与「既有定点层」共存时的命名约定
// ---------------------------------------------------------------------------
//   同目录下已存在 §6.3 的定点实现（pdhg_core.hpp + pdhg_core_fixed.cpp
//   + test_pdhg.cpp），其类型名 PdhgParams / PdhgWork / PdhgOut / PdhgStatus
//   与 TV 原语名 pdhg_*_i32 已被占用。本层所有可能跨层同名的类型统一加 Fp 后缀，
//   原语加 _fp 后缀。函数名保持 pdhg_core（参数类型不同即为重载）。
// ============================================================================

#ifndef PENUMBRASIGHT_HLS_PDHG_CORE_H
#define PENUMBRASIGHT_HLS_PDHG_CORE_H

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "../lr_operator/lr_operator.h"

namespace ps {

// ---------------------------------------------------------------- 编译期参数

// §6.5「只允许在最小 8 次后早停」。这是硬下限，调用方不可绕过。
#ifndef PS_PDHG_EARLY_STOP_MIN_ITERS
#define PS_PDHG_EARLY_STOP_MIN_ITERS 8
#endif
constexpr int kPdhgEarlyStopMinItersFp = PS_PDHG_EARLY_STOP_MIN_ITERS;

// HLS 顶层包装（pdhg_core_top）用的并行度。B 档 16。
#ifndef PS_PDHG_LANES_B_FP
#define PS_PDHG_LANES_B_FP 16
#endif

// ---------------------------------------------------------------- 图像布局
//
// 与 recon.hpp 的 ReconShape 同约定（行 = z、列 = x、扁平 = z*W + x），
// 此处独立定义只是为了不拖入 recon.hpp 的裸 pragma（见文首）。
struct PdhgShapeFp {
    int n_rows;   // = 隐藏图像行数（z 方向，B 档 18）
    int n_cols;   // = 隐藏图像列数（x 方向，B 档 24）
};

inline int pdhg_shape_n(const PdhgShapeFp& s) { return s.n_rows * s.n_cols; }

// ---------------------------------------------------------------- TV 原语
// 与 recon.hpp 的 tv_gradient / tv_divergence / tv_project_l2_ball 逐行同式。

// g = ∇f，Neumann 边界。g 长度 2N，布局见文首。
inline void pdhg_gradient_fp(const double* f, const PdhgShapeFp& s, double* g) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE off
#endif
    const int H = s.n_rows, W = s.n_cols, N = H * W;
    for (int i = 0; i < 2 * N; ++i) g[i] = 0.0;

    double* gz = g;          // 行方向（z）分量
    double* gx = g + N;      // 列方向（x）分量
    for (int i = 0; i + 1 < H; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        const double* cur = f + (std::size_t)i * W;
        const double* nxt = f + (std::size_t)(i + 1) * W;
        double* dst = gz + (std::size_t)i * W;
        for (int j = 0; j < W; ++j) dst[j] = nxt[j] - cur[j];
    }
    for (int i = 0; i < H; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        const double* cur = f + (std::size_t)i * W;
        double* dst = gx + (std::size_t)i * W;
        for (int j = 0; j + 1 < W; ++j) dst[j] = cur[j + 1] - cur[j];
    }
}

// out = divergence(q) = −∇ᵀq（负伴随），Neumann 边界。满足 <∇f, q> = −<f, div q>。
inline void pdhg_divergence_fp(const double* q, const PdhgShapeFp& s,
                               double* out) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE off
#endif
    const int H = s.n_rows, W = s.n_cols, N = H * W;
    const double* qz = q;
    const double* qx = q + N;

    for (int i = 0; i < N; ++i) out[i] = 0.0;

    for (int i = 0; i + 1 < H; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        const double* src = qz + (std::size_t)i * W;
        double* up = out + (std::size_t)i * W;
        double* dn = out + (std::size_t)(i + 1) * W;
        for (int j = 0; j < W; ++j) {
            up[j] -= src[j];
            dn[j] += src[j];
        }
    }
    for (int i = 0; i < H; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        const double* src = qx + (std::size_t)i * W;
        double* row = out + (std::size_t)i * W;
        for (int j = 0; j + 1 < W; ++j) {
            row[j] -= src[j];
            row[j + 1] += src[j];
        }
    }
    for (int i = 0; i < N; ++i) out[i] = -out[i];
}

// 逐像素把 (qz, qx) 投影到半径 radius 的 L2 球（各向同性 TV 的对偶投影）。
// radius = λ；λ = 0 时对偶变量恒为 0（在 (0,0) 处投影仍是 0，无需特判）。
inline void pdhg_project_l2_ball_fp(double* q, const PdhgShapeFp& s,
                                    double radius) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
    const int N = pdhg_shape_n(s);
    double* qz = q;
    double* qx = q + N;
    for (int i = 0; i < N; ++i) {
        const double nrm = std::sqrt(qz[i] * qz[i] + qx[i] * qx[i]);
        if (nrm > radius) {
            const double sc = radius / nrm;
            qz[i] *= sc;
            qx[i] *= sc;
        }
    }
}

// ---------------------------------------------------------------- 参数/出入

enum PdhgStatusFp {
    kPdhgOkFp      = 0,
    kPdhgBadArgsFp = 2
};

struct PdhgParamsFp {
    int    n_iter;          // 固定迭代次数上限（B 档 16，A 档 8，最大 32）
    int    min_iters;       // 允许早停的最小迭代数（§6.5：8）
    int    residual_every;  // 每几次迭代算一次归一化残差（§6.5：4）
    double tau;             // 原始步长 τ（由 ||B||₂ 导出）
    double sigma;           // 对偶步长 σ（由 ||B||₂ 导出）
    double lam;             // TV 权重 λ（对偶球半径）
    double theta;           // 惯性外推系数 θ（1.0 = §6.5 默认）
    double rho_stop;        // 归一化残差早停门限；<= 0 表示不早停
};

// 迭代暂存。综合代码里不许有 new/delete，全部由调用方传入。
//   f_prev / fbar / divq / btr : N
//   q / grad                   : 2N
//   bf / r                     : M
struct PdhgWorkFp {
    double* f_prev;
    double* fbar;
    double* q;
    double* grad;
    double* divq;
    double* bf;
    double* r;
    double* btr;
};

struct PdhgOutFp {
    int    n_used;          // 实际迭代次数
    int    early_stopped;   // 1 = 因残差门限提前结束
    int    n_res;           // 残差历史条数（= 实际评估次数）
    int    monotone;        // 1 = 归一化残差历史单调不增
    double rho;             // 最后一次归一化残差 ρ
    double rho_first;       // 第一次归一化残差（看下降倍数）
    double res_norm;        // 最后一次 ||B f − d||
    double d_norm;          // ||d||
};

// ---------------------------------------------------------------- 核

// 单通道：解 min_{f>=0} ½||B f − d||² + λ·TV(f)，固定迭代原始-对偶更新（§6.5）。
//
//   F     : 选中的候选因子（U/S/V，R = F.K）
//   d     : [M] 归一化测量（竖直差分后）
//   shape : 隐藏图像布局；要求 pdhg_shape_n(shape) == F.N
//   f     : [N] 输入初值（通常全 0），输出解
//   W     : 迭代暂存（见 PdhgWorkFp）
//   res_hist : 归一化残差历史，可为 0；建议长度 >= ceil(n_iter/residual_every)
//   res_hist_cap : res_hist 的容量（元素数）
//
// 返回 kPdhgOkFp / kPdhgBadArgsFp。
template <int LANES>
int pdhg_core(const LrFactorsFp& F,
              const double* d,
              const PdhgShapeFp& shape,
              const PdhgParamsFp& P,
              double* f,
              PdhgWorkFp& W,
              PdhgOutFp& out,
              double* res_hist,
              int res_hist_cap);

// ---------------------------------------------------------------- 主机侧标定
#ifndef __SYNTHESIS__
// 步长规则：τ = σ = sqrt(safety) / sqrt(||B||² + ||∇||²)。
//   norm_grad = 0 时退化成「只按 B 定步长」；norm_grad 传前向差分算子的范数上界
//   （2 分量、Neumann 边界，||∇||² <= 8）。原始-对偶的稳定条件是
//   τσ(||B||² + ||∇||²) < 1，只按 ||B|| 定 τ 会在 λ > 0 时把步长放大好几倍。
void pdhg_step_sizes(double norm_B, double norm_grad, double safety,
                     double* tau, double* sigma);
#endif  // __SYNTHESIS__

}  // namespace ps

#endif  // PENUMBRASIGHT_HLS_PDHG_CORE_H
