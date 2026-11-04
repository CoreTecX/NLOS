// ============================================================================
// PenumbraSight 低秩算子核（B 与 Bᵀ） · lr_operator
//
//   lr_operator.hpp  —— 接口、Q 格式与**全部移位量的推导**（本文件）
//   lr_operator_fixed.cpp —— 可综合的核实现（四阶段复用同一 MAC 阵列）+ LANES 实例化
//                            （_fixed 后缀：同目录另有双精度参考层，
//                             见 src/fpga/hls/README.md）
//   test_lr_operator.cpp —— 主机侧 testbench（含伴随性与逐级 1 LSB 判据）
//
// 依据
// ---------------------------------------------------------------------------
//   方案 §6.2 模块接口（lr_operator：输入 U、S、V、f 或残差，输出 Bf 或 Bᵀr，
//                         "HLS，可配置 R 和 LANES"）
//   方案 §6.5 低秩重建与 TV：
//       Bf  = U * (S .* (Vᵀ * f))
//       BTr = V * (S .* (Uᵀ * r))
//   方案 §6.3 定点格式；configs/fixed_point_b.yaml 的 bit_chain
//   方案 §5.2 低秩模型存储（U/V rank-major、per-candidate scale）
//
// 形状与布局（与 model.psm 一致，错了不会报错只会整体错位）
// ---------------------------------------------------------------------------
//   U : int16[K][M]  rank-major，U[k*M + m]     M = 9120（95x96 差分测量）
//   S : uint16[K]    归一化奇异值 Q2.14
//   V : int16[K][N]  rank-major，V[k*N + i]     N = 432（24x18 隐藏图像）
//   f : int32[N]     图像状态，Q6.18（§6.3 image_state 24 bit）
//   d, r : int32[M]  测量域，Q2.16（§6.3 meas_norm 18 bit）
//   Bf : int32[M]    测量域 Q2.16
//   Bᵀr: int32[N]    图像域 Q6.18
//
//   V 与 U 都是「K 行、每行长度 L」的 rank-major 布局，所以四个矩阵乘
//   （Vᵀf、Uz、Uᵀr、Vq）可以共用**同一个** MAC 阵列 —— 这正是 Task 7 Step 2
//   「避免为每个矩阵乘复制硬件」的做法。本文件把它们实现成同一个
//   rank_major_matvec()，只换 (codes, L, K) 三个参数。
//
// 移位量的推导（每一步都不能拍脑袋，这里逐个写清）
// ---------------------------------------------------------------------------
// 记 value = code * 2^(-frac)。三个 scale 用 Q16.16 增益表示：
//   u_gain = round(u_scale * 2^16)，v_gain、s_gain 同理（由 model_loader 算好）。
//
//   记 ΔF = kImageFrac - kMeasFrac = 18 - 16 = 2（图像域与测量域的小数位之差）
//
//   前向 Bf = U (S ∘ (Vᵀ f))
//     1) acc_v = Σ_i V_code[i]·f_code[i]                   int64，48 bit 政策
//        z_real = acc_v · v_scale · 2^-ΔF ... 见 z 码字的定义：
//        z_code ≜ z_real · 2^18 = acc_v · v_scale
//              = acc_v · v_gain · 2^-16
//        => z_code = mul_shift_round(acc_v, v_gain, 16)          [shift = 16]
//        （z 与图像域同一格式 Q6.18，因为它是 Vᵀf，量纲与 f 同）
//     2) zs_code = (z_real·s_real)·2^18 = z_code · S_real
//              = z_code · S_code · s_scale = z_code·S_code·s_gain·2^-16
//        => zs_code = mul_shift_round(z_code·S_code, s_gain, 16)  [shift = 16]
//     3) acc_u = Σ_k U_code[k][m]·zs_code[k]
//        y_real = acc_u · u_scale · 2^-18
//        y_code ≜ y_real·2^16 = acc_u · u_gain · 2^-16 · 2^-2
//        => y_code = mul_shift_round(acc_u, u_gain, 18)           [shift = 18]
//
//   伴随 Bᵀr = V (S ∘ (Uᵀ r))
//     1') acc_u = Σ_m U_code[k][m]·r_code[m]        （r 是 Q2.16）
//         w_real = acc_u · u_scale · 2^-16
//         w_code ≜ w_real·2^18 = acc_u · u_gain · 2^-16 · 2^+2
//         => w_code = mul_shift_round(acc_u, u_gain, 14)          [shift = 14]
//     2') ws_code = mul_shift_round(w_code·S_code, s_gain, 16)    [shift = 16]
//     3') acc_v = Σ_k V_code[k][i]·ws_code[k]
//         g_real = acc_v · v_scale · 2^-18；g_code ≜ g_real·2^18
//         => g_code = mul_shift_round(acc_v, v_gain, 16)          [shift = 16]
//
//   四个移位量因此固定为 16 / 16 / 18 / 14 / 16（含伴随的 16），全部写成
//   具名常量（kShiftVToZ 等），不出现魔法数字。
//
// int64 溢出的前提核算（每条都要成立，testbench 里逐条打印）
// ---------------------------------------------------------------------------
//   |acc| <= K · 32767 · |f/r code| <= 96 · 32767 · 2^23 = 2^43.3   （K<=96 时）
//   乘增益：|acc|·|gain| <= 2^43.3 · 2^16 = 2^59.3 < 2^63          ✓ int64 安全
//   中间的 z_code·S_code <= 2^23 · 2^16 = 2^39，再乘 s_gain <= 2^21 ✓ 安全
//   （前提：增益被 gain_safe() 限制在 ±65535，见 ps_fixed.hpp）
//
// 与既有原语的复用
// ---------------------------------------------------------------------------
//   * ps_fixed.hpp：Q 格式、偶数舍入+饱和、增益、位宽链
//   * ../hardware/recon.hpp：ReconShape / recon_n 的图像布局约定
//     （行 = z、列 = x，Neumann 边界），本核的 N 与向量展平顺序与它一致，
//     因此 pdhg_core 可以直接把本核的输出交给 recon.hpp 的同族 TV 算子。
//   * src/model/penumbrasight_model/quantize.py：power_iteration_norm /
//     pdhg_step_sizes 的规则；本文件的 lr_spectral_norm_power() 是它在 C++
//     一侧的实现，用于给 lr_operator 的 testbench 与 pdhg_core 的步长标定
//     提供**同一个** ||B||₂ 估计（方案 §6.5 要求由 PC 端幂迭代导出 τ、σ）。
// ============================================================================

#ifndef PENUMBRASIGHT_HLS_LR_OPERATOR_HPP
#define PENUMBRASIGHT_HLS_LR_OPERATOR_HPP

#include <cstddef>
#include <cstdint>

#include "../ps_fixed.hpp"

namespace ps {

// ---------------------------------------------------------------- 编译期参数

// 最大秩。model.psm 的 rank_max 默认 64，R=96 用单独模型文件（方案 §8.1）；
// 本核把上界开到 96，超出即报参数错误而不静默截断。
#ifndef PS_LR_MAX_RANK
#define PS_LR_MAX_RANK 96
#endif

// 移位量（推导见文首）。改名会同时改掉四个阶段的定标，务必一起看。
constexpr int kShiftVToZ = kGainFrac;                              // 16
constexpr int kShiftZToS = kGainFrac;                              // 16
constexpr int kShiftUToY = kGainFrac + (kImageFrac - kMeasFrac);   // 18
constexpr int kShiftUToW = kGainFrac - (kImageFrac - kMeasFrac);   // 14
constexpr int kShiftVToG = kGainFrac;                              // 16

// ---------------------------------------------------------------- 接口结构

enum LrStatus {
    kLrOk      = 0,
    kLrBadArgs = 2
};

// 一个候选的低秩因子（方案 §5.2：B_p ≈ U_p Σ_p V_pᵀ）。
// 指针全部由调用方持有（G7：综合代码里不许有 new/delete）。
struct LrFactors {
    const int16_t*  U;            // [K*M] rank-major
    const uint16_t* S;            // [K]   Q2.14 归一化奇异值
    const int16_t*  V;            // [K*N] rank-major
    int K;                        // 实际用的秩（定位用 8，重建 32/48/64）
    int M;                        // 测量数（B 档 9120）
    int N;                        // 隐藏图像像素数（B 档 432）
    int32_t u_gain_q16;           // round(u_scale * 2^16)
    int32_t s_gain_q16;           // round(s_scale * 2^16)
    int32_t v_gain_q16;           // round(v_scale * 2^16)
    // ---- 增益的指数表示（新路径，见 docs/gain_representation.md）----
    // 生产端（quantize.py 的 quantize_symmetric_pow2）保证 scale 取 2 的幂，
    // 于是 u_exp = log2(u_scale) 等，增益**精确可表示**；数据通路按
    // `ps::exp_to_gain_shift` 把指数折进移位量。
    //
    // **默认 kGainExpInvalid 表示「用上面的 Q16.16 字段」**，也就是旧路径。
    // 默认值让既有调用方（含全部 test_*.cpp）不用改，行为逐位不变 ——
    // 这是本改动能安全落地的前提。
    int u_exp = kGainExpInvalid;
    int s_exp = kGainExpInvalid;
    int v_exp = kGainExpInvalid;
};

// 可选的阶段观测缓冲：非空时把四个中间量（z / zs / w / ws）抄出来，
// 供 testbench 做「每级误差 <= 1 LSB」的逐级判据（Task 7 Step 1）。
// 长度均要求 >= K。传 nullptr 表示不观测（板上去掉这几个分支）。
// 观测旁路缓冲（**仅主机侧**）。四个成员是指针，核按 `LrWork*` 收它 —— 这在 HLS 里
// 构成「指针的指针」，Syn check 直接拒绝（HLS 214-134）。故 `lr_operator_fixed.cpp`
// 里所有 `W->*` 写入都包在 `#ifndef __SYNTHESIS__` 内，综合视图中该结构体不出现。
// 板上要看中间量请走各通道的 out_* 导出；这里只服务 test_lr_*.cpp 的逐值对照。
struct LrWork {
    int32_t* z;                   // [K] Vᵀf，Q6.18
    int32_t* zs;                  // [K] S∘(Vᵀf)，Q6.18
    int32_t* w;                   // [K] Uᵀr，Q6.18
    int32_t* ws;                  // [K] S∘(Uᵀr)，Q6.18
};

// 诊断：定点饱和记账 + 算力记账（方案 §9.2 要求报告 MAC 数）
struct LrDiag {
    FixFlags  flags;
    long long mac_count;          // 本次调用累计的标量 MAC 数
};

// ---------------------------------------------------------------- 核

// LANES = 每个周期的列并行度。U 与 V 共享同一个 MAC 阵列，只是长度不同。
// 整数累加可结合，所以 8/16/32 必须给出逐位相同的结果（testbench 会验证）。
template <int LANES>
int lr_forward(const LrFactors& F,
               const int32_t* f_code,     // [N] Q6.18
               int32_t* y_code,           // [M] Q2.16
               LrDiag& dg,
               LrWork* W);

template <int LANES>
int lr_adjoint(const LrFactors& F,
               const int32_t* r_code,     // [M] Q2.16
               int32_t* g_code,           // [N] Q6.18
               LrDiag& dg,
               LrWork* W);

// ---------------------------------------------------------------- 主机侧工具
// 下面两个函数只在主机/PS 侧存在（综合时不编入）。它们给出与 Python 参考
// （quantize.power_iteration_norm / pdhg_step_sizes）同口径的标定工具。

#ifndef __SYNTHESIS__
// 把码字 + Q16.16 增益还原成稠密 float64 的 B = U·diag(S)·Vᵀ，写入 B_dense（M 行 N 列，
// 行主序）。仅供 testbench 与 PS 侧核对，板端**不**物化 B（方案 §5.2 的体积理由）。
void lr_codes_to_dense_B(const LrFactors& F, double* B_dense);

// 幂迭代估计 ||B||₂，与 quantize.power_iteration_norm 同一判据（只要求算子，
// 不物化 B）。返回估计值；iters_used / delta 通过指针带回，没收敛也如实返回 ——
// 悄悄返回一个没收敛的范数会让 τ 偏大、PDHG 发散，而发散在图像上只表现为"一片灰"。
double lr_spectral_norm_power(const LrFactors& F, int iters, double tol,
                              uint64_t seed, int* iters_used, double* delta);
#endif  // __SYNTHESIS__

}  // namespace ps

#endif  // PENUMBRASIGHT_HLS_LR_OPERATOR_HPP
