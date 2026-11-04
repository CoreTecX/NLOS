// ============================================================================
// PenumbraSight 低秩算子核（B 与 Bᵀ） · lr_operator.h
//
//   lr_operator.h  —— 接口 + psm_format 因子还原（本文件）
//   lr_operator.cpp—— 可综合的核实现 + LANES 显式实例化
//   tb.cpp         —— 主机侧 testbench（含稠密 U·diag(S)·Vᵀ 对照）
//
// 设计目标：同一份源码在两个环境下都能工作（照抄 src/hardware/recon.hpp 的约定）
//   g++       : 主机侧功能验证；本目录综合专用 pragma 全包在 #ifdef __SYNTHESIS__
//   Vitis HLS : 综合（工具自己定义 __SYNTHESIS__）
//
// 依据
// ---------------------------------------------------------------------------
//   教师方案 §6.5 低秩重建与 TV：
//       Bf  = U * ( S .* ( Vᵀ * f ) )          （前向）
//       BTr = V * ( S .* ( Uᵀ * r ) )          （伴随）
//   教师方案 §5.2 低秩模型存储：rank-major、**每候选独立 scale**。
//   src/software/include/psm_format.h：U/V 为 rank-major、S 为 Q2.14 归一化奇异值；
//       定位只读前 K 个 rank 时 K*M 个 int16 在文件里连续。
//
// 形状与布局（与 model.psm 一致，错了不会报错只会整体错位）
// ---------------------------------------------------------------------------
//   U : double[K*M]  rank-major，元素 (k, m) 在  U[k*M + m]
//   S : double[K]
//   V : double[K*N]  rank-major，元素 (k, n) 在  V[k*N + n]
//   f : double[N]    隐藏图像（隐藏网格，行 = z、列 = x，扁平 = z*W + x）
//   y : double[M]    差分测量域
//   r : double[M]    （伴随输入）
//   g : double[N]    （伴随输出）
//
//   这里的 U/S/V 已经是**真实系数**（码字 × 该候选的 scale）。还原动作统一由
//   lr_factors_from_psm() 完成，且**只走 psm_format.h 的访问器**：
//       ps_psm_candidate_offset / ps_psm_meas_len / ps_psm_hidden_len /
//       ps_psm_read_U_km / ps_psm_read_S_k / ps_psm_read_V_kn
//   核本身不认识 model.psm，也就不会有人手算偏移。
//
// 为什么两个方向的矩阵乘可以复用同一套原语（Task 7 Step 2 的想法）
// ---------------------------------------------------------------------------
//   U 与 V 都是「K 行、每行长度 L」的 rank-major 码字矩阵，于是四个矩阵乘只有
//   两种归约方向：
//       rows(codes,L,x) -> out[k] = Σ_l codes[k*L + l] * x[l]      （Vᵀf、Uᵀr）
//       cols(codes,L,x) -> out[l] = Σ_k codes[k*L + l] * x[k]      （U·zs、V·ws）
//   Bf 与 BTr 各自就是这两者的组合（中间各插一次逐元素乘 S）。R（= K）只影响
//   归约长度，LANES 只影响并行度，硬件上是同一个 MAC 阵列。
//
// 与「既有定点层」共存时的命名约定
// ---------------------------------------------------------------------------
//   同目录下已存在 §6.3 的定点实现（lr_operator.hpp + lr_operator_fixed.cpp
//   + test_lr_operator.cpp），其类型名 LrFactors / LrWork / LrDiag / LrStatus
//   已被占用。本层所有可能跨层同名的类型统一加 Fp 后缀。函数名保持 lr_forward /
//   lr_adjoint（参数类型不同即为重载）。
// ============================================================================

#ifndef PENUMBRASIGHT_HLS_LR_OPERATOR_H
#define PENUMBRASIGHT_HLS_LR_OPERATOR_H

#include <cstddef>
#include <cstdint>

#include "../../../software/include/psm_format.h"

namespace ps {

// ---------------------------------------------------------------- 编译期参数

// 秩的上界。§8.1 的 rank_max 默认 64，R=96 用单独模型文件；本层把上界开到
// 128（片上 z/zs/w/ws 暂存按此定长），超出即报参数错误而不静默截断。
#ifndef PS_LR_MAX_RANK_FP
#define PS_LR_MAX_RANK_FP 128
#endif

// HLS 顶层包装（lr_forward_top / lr_adjoint_top）用的并行度。B 档 16。
#ifndef PS_LR_LANES_B_FP
#define PS_LR_LANES_B_FP 16
#endif

// ---------------------------------------------------------------- 接口结构

enum LrStatusFp {
    kLrOkFp      = 0,
    kLrBadArgsFp = 2
};

// 一个候选的低秩因子（方案 §5.2：B_p ≈ U_p Σ_p V_pᵀ）。
// 指针全部由调用方持有（综合代码里不许有 new/delete）。
struct LrFactorsFp {
    const double* U;   // [K*M] rank-major
    const double* S;   // [K]
    const double* V;   // [K*N] rank-major
    int K;             // 实际用的秩 R（定位 8；重建 32/48/64）
    int M;             // 测量数（B 档 9120）
    int N;             // 隐藏图像像素数（B 档 432）
};

// ---------------------------------------------------------------- psm 还原

// 把某候选记录还原成真实系数因子，写入调用方缓冲并填好 F。
//   U_out [K*M]、S_out [K]、V_out [K*N]
// 返回 kLrOkFp / kLrBadArgsFp。K 超出 rank_max 时报错（不静默截断）。
inline int lr_factors_from_psm(const uint8_t* cand_base,
                               const ps_psm_header_t* h, int K,
                               double* U_out, double* S_out, double* V_out,
                               LrFactorsFp& F) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE off
#endif
    F.U = 0;
    F.S = 0;
    F.V = 0;
    F.K = 0;
    F.M = 0;
    F.N = 0;
    if (cand_base == 0 || h == 0 || U_out == 0 || S_out == 0 || V_out == 0) {
        return kLrBadArgsFp;
    }
    if (K <= 0 || (int)h->rank_max < K) return kLrBadArgsFp;

    const int M = (int)ps_psm_meas_len(h);
    const int N = (int)ps_psm_hidden_len(h);
    if (M <= 0 || N <= 0) return kLrBadArgsFp;

    ps_psm_candidate_hdr_t ch;
    __builtin_memcpy(&ch, cand_base, sizeof(ch));
    const double usc = (double)ch.u_scale;   // 每候选独立 scale（§6.3）
    const double ssc = (double)ch.s_scale;
    const double vsc = (double)ch.v_scale;

    for (int k = 0; k < K; ++k) {
        for (int m = 0; m < M; ++m) {
            U_out[(std::size_t)k * M + m] =
                (double)ps_psm_read_U_km(cand_base, h, (uint16_t)k,
                                         (uint64_t)m) * usc;
        }
        S_out[k] = (double)ps_psm_read_S_k(cand_base, h, (uint16_t)k) * ssc;
        for (int n = 0; n < N; ++n) {
            V_out[(std::size_t)k * N + n] =
                (double)ps_psm_read_V_kn(cand_base, h, (uint16_t)k,
                                         (uint64_t)n) * vsc;
        }
    }

    F.U = U_out;
    F.S = S_out;
    F.V = V_out;
    F.K = K;
    F.M = M;
    F.N = N;
    return kLrOkFp;
}

// ---------------------------------------------------------------- 核

// y = U * (S .* (Vᵀ f))       y 长度 M
//   z[k]  = Σ_n V[k][n] f[n]
//   zs[k] = S[k] z[k]
//   y[m]  = Σ_k U[k][m] zs[k]
template <int LANES>
int lr_forward(const LrFactorsFp& F, const double* f, double* y);

// g = V * (S .* (Uᵀ r))       g 长度 N
//   w[k]  = Σ_m U[k][m] r[m]
//   ws[k] = S[k] w[k]
//   g[n]  = Σ_k V[k][n] ws[k]
template <int LANES>
int lr_adjoint(const LrFactorsFp& F, const double* r, double* g);

}  // namespace ps

#endif  // PENUMBRASIGHT_HLS_LR_OPERATOR_H
