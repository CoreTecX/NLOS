// ============================================================================
// ps_fixed_top.hpp —— 定点层 HLS 顶层接口（A6 的定点对偶）
//
// 为什么要有这一层
// ----------------------------------------------------------------------------
// 仓库里有两层数学实现（见 ps_axi_top.h 文首）：
//
//   双精度层  lr_forward_top / pdhg_core_top / projection_engine_top（*.cpp + *.h）
//             —— 当前 build_hls.tcl 综合的就是它，但 XCZU9EG 的 DSP48E2 是
//                27×18 乘法器（全器件 2520 个，见 A2_资源预算.md），**做不了
//                双精度浮点**，所以这一层上不了板。
//   定点层    projection_engine / lr_forward / lr_adjoint / pdhg_core（*_fixed.cpp
//             + *.hpp）—— 用 int16 系数码 + int32 累加，是**真正要上板**的一层，
//             有完整的主机测试（test_*.cpp），却**没有任何 csynth 顶层**：
//             它们全是模板核，Vitis HLS 的 top 必须是具体函数。
//
// 本层的 4 个 `ps_*_fix` 顶层补这个缺口。做法与 A6 的 ps_*_axi 层完全一致：
//
//   · 把入参保真成「平坦数组 + 标量」，并显式绑定 m_axi / s_axilite；
//   · 内部只做三件事：**拼装结构体 -> 转调定点核 -> 逐字段回写标量**；
//     数学一行都不重写 —— 所以结果与「直接调用核」**逐位相同**。
//     接口层的 bug 只应该表现为「连错线」，不应该表现为「结果不对」；
//   · 三个定点核（*_fixed.cpp / *.hpp）**一字不改**，主机侧 test_*.cpp 继续跑。
//
// ⚠ 本层交付的是「接口契约 + 可编译的包装」，**不是已验证的 IP**。
//    真正的综合、时序收敛、资源占用仍要等 Vivado —— 与 ps_axi_top.h 同一口径。
//
// 依据
// ----------------------------------------------------------------------------
//   docs/hls_interface_spec.md（接口冻结；bundle 与寄存器映射）
//   src/fpga/hls/ps_axi_top.h（A6 接口层的 bundle 约定与返回码写法，本层沿用）
//   src/fpga/hls/projection_engine/projection_engine.hpp（定位核签名与定标）
//   src/fpga/hls/lr_operator/lr_operator.hpp（低秩算子签名与四个移位量）
//   src/fpga/hls/pdhg_core/pdhg_core.hpp（重建核签名与暂存长度）
//   src/hardware/recon.hpp（ReconShape / recon_n）
//   方案 §6.2 模块接口 / §6.4 定位引擎 / §6.5 低秩重建与 TV
//
// 并行度（唯一一处）
// ----------------------------------------------------------------------------
//   三个 `_fixed.cpp` 都显式实例化了 LANES = 8 / 16 / 32。本层用唯一的宏
//   PS_FIX_LANES（默认 16，B 档，见 §6.4）指定用哪一个，4 个顶层全部用它。
//   改并行度只改这一处，或用编译命令行 `-DPS_FIX_LANES=8` 覆盖。
//
// 硬规则（与 ps_fixed.hpp 文首同一约定）
// ----------------------------------------------------------------------------
//   1) 综合代码里**不许 new / delete**，不许隐藏分配；全部暂存由调用方传入；
//   2) 所有 `#pragma HLS ...` 都写在 `#ifdef __SYNTHESIS__` 里面
//      （主机 g++ 编译零告警，Vitis HLS 综合时照常生效）；
//   3) `__SYNTHESIS__` 下**每一个指针参数**都有显式 INTERFACE pragma；
//   4) 每个顶层开头做入参合法性检查（空指针 / 维度 <= 0 / K > PS_LR_MAX_RANK）。
//
// bundle 约定（与 ps_axi_top.h 完全一致）
// ----------------------------------------------------------------------------
//   gmem0_model   : U / S / V —— 模型体，IP 侧只读
//   gmem1_data    : 测量、图像、分数、结果数组 —— IP 侧读写
//   gmem2_scratch : lr 观测缓冲 + pdhg 的 9 个迭代暂存 —— IP 侧读写
//   control       : 全部标量与返回码（s_axilite），含 `port=return`
//
// 返回码（直接沿用被包装核的枚举，不另立一套数值）
// ----------------------------------------------------------------------------
//   0 = 成功        ps::kProjOk / ps::kLrOk / ps::kPdhgOk
//   2 = 入参非法    ps::kProjBadArgs / ps::kLrBadArgs / ps::kPdhgBadArgs
//     （空指针 / 维度 <= 0 / K > PS_LR_MAX_RANK；对应 ps_fixed.hpp 的统一口径）
//
// K 的合法上限
// ----------------------------------------------------------------------------
//   xc 上界由被包装核自己的 `PS_LR_MAX_RANK`（lr_operator.hpp，默认 96）钉死；
//   本层在入口按同一常量早退，避免把非法 K 带进核。
// ============================================================================

#ifndef PENUMBRASIGHT_HLS_PS_FIXED_TOP_HPP
#define PENUMBRASIGHT_HLS_PS_FIXED_TOP_HPP

#include <stdint.h>

// ---------------------------------------------------------------- 接口版本
// 与 ps_axi_top.h 的 PS_AXI_IF_VERSION 同一用途：块设计脚本
// （src/fpga/bd/penumbrasight_bd.tcl）会读它并与脚本里的常量比对，
// 不一致就报错退出 —— 这样「改了 bundle 或增删了 s_axilite 标量却忘了同步
// 块设计/板端软件」会当场暴露，而不是变成板上一处静默的地址错位。
// **改 bundle 或增删标量时，本值 +1，并同步 penumbrasight_bd.tcl 的 EXPECT_IF_VER。**
#define PS_FIX_IF_VERSION 1

// ---------------------------------------------------------------- 并行度（唯一一处）
// 与三个 _fixed.cpp 的显式实例化（8 / 16 / 32）保持一致；默认 B 档 16。
// 4 个顶层全部用这个宏，改并行度只改这一处。
#ifndef PS_FIX_LANES
#define PS_FIX_LANES 16
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 定位：候选打分与排序
// 包装 ps::projection_engine<PS_FIX_LANES>。
//   meas      [3*M]     Q2.16，通道优先 d_R|d_G|d_B
//   u_codes   [P*K*M]   U 系数，rank-major
//   gain_q16  [P]       round(u_scale_p · d_scale · 2^16)
//   score_out [P]       总分数（三通道求和）
//   score_ch  [P*3]     每候选每通道分数，可为 nullptr
//   best_id / second_id / margin_q16 / boundary / low_conf / overflow
//                       <- ProjResult 逐字段
//   best_score / second_score  <- ProjResult（uint64）
//   sat_events / mac_overflow / out_overflow / score_overflow / gain_clamped /
//   max_abs_acc         <- 核的 FixFlags 逐字段（6 个字段全部回写）
int ps_projection_fix(const int32_t* meas,
                      const int16_t* u_codes,
                      const int32_t* gain_q16,
                      int P, int K, int M,
                      int nx, int ny, int nz,
                      int margin_thresh_q16,
                      uint64_t* score_out,
                      uint64_t* score_ch,
                      int* best_id, int* second_id, int* margin_q16,
                      int* boundary, int* low_conf, int* overflow,
                      unsigned long long* best_score,
                      unsigned long long* second_score,
                      int* sat_events, int* mac_overflow, int* out_overflow,
                      int* score_overflow, int* gain_clamped,
                      long long* max_abs_acc);

// ---------------------------------------------------------------- 低秩前向 y = U·diag(S)·(V f)
// 包装 ps::lr_forward<PS_FIX_LANES>。
//   U [K*M] / S [K] / V [K*N]（rank-major），增益与指数见 LrFactors
//   f [N] Q6.18 -> y [M] Q2.16
//   wk_z / wk_zs [K]：阶段观测缓冲，**传 nullptr 表示不观测**
//     （两个都传 nullptr 时按约定把 LrWork* 传 nullptr）
//   mac_count / 6 个 FixFlags 字段 <- LrDiag 逐字段
int ps_lr_forward_fix(const int16_t* U,
                      const uint16_t* S,
                      const int16_t* V,
                      int K, int M, int N,
                      int32_t u_gain_q16, int32_t s_gain_q16, int32_t v_gain_q16,
                      int u_exp, int s_exp, int v_exp,
                      const int32_t* f,
                      int32_t* y,
                      int32_t* wk_z,
                      int32_t* wk_zs,
                      long long* mac_count,
                      int* sat_events, int* mac_overflow, int* out_overflow,
                      int* score_overflow, int* gain_clamped,
                      long long* max_abs_acc);

// ---------------------------------------------------------------- 低秩伴随 g = Vᵀ·diag(S)·(Uᵀ r)
// 包装 ps::lr_adjoint<PS_FIX_LANES>；参数与 forward 一一对应，只是
//   f -> r [M] Q2.16，y -> g [N] Q6.18，观测缓冲换成 wk_w / wk_ws [K]。
// wk_* 同样支持 nullptr（不观测）。
int ps_lr_adjoint_fix(const int16_t* U,
                      const uint16_t* S,
                      const int16_t* V,
                      int K, int M, int N,
                      int32_t u_gain_q16, int32_t s_gain_q16, int32_t v_gain_q16,
                      int u_exp, int s_exp, int v_exp,
                      const int32_t* r,
                      int32_t* g,
                      int32_t* wk_w,
                      int32_t* wk_ws,
                      long long* mac_count,
                      int* sat_events, int* mac_overflow, int* out_overflow,
                      int* score_overflow, int* gain_clamped,
                      long long* max_abs_acc);

// ---------------------------------------------------------------- 重建：三通道 PDHG
// 包装 ps::pdhg_core<PS_FIX_LANES>（RGB 共用同一 U/S/V，独立求解，顺序复用暂存）。
//   暂存长度（由 PdhgWork 语义钉死，全部必须非 nullptr）：
//     wk_f_prev / wk_fbar / wk_divq / wk_btr / wk_tmp : N
//     wk_q / wk_grad                                  : 2N
//     wk_y / wk_r                                      : M
//   d3 [3*M] Q2.16 通道优先 -> f3 [3*N] Q6.18 通道优先
//   shape = {n_rows, n_cols}（来自 src/hardware/recon.hpp，须 n_rows*n_cols == N）
//   每通道输出数组 <- PdhgOut 逐字段；末尾 6 个标量 <- 顶层按值构造的 FixFlags。
//
//   ⚠ 参数名取舍：规格原文里「每通道结果数组」与「FixFlags 标量组」都写作
//     out_overflow，二者同名无法编译（C++ 不允许同名形参），故把**每通道数组**
//     改名为 out_overflow_ch[3]；FixFlags 标量组的 6 个名字在 4 个顶层里保持
//     完全一致（便于 bd 侧共用一张寄存器表）。
int ps_pdhg_fix(const int16_t* U,
                const uint16_t* S,
                const int16_t* V,
                int K, int M, int N,
                int32_t u_gain_q16, int32_t s_gain_q16, int32_t v_gain_q16,
                int u_exp, int s_exp, int v_exp,
                const int32_t* d3,
                int n_rows, int n_cols,
                int n_iter, int min_iters, int residual_every, int tv_mode,
                int32_t tau_q16, int32_t sigma_q16, int32_t lambda_q16,
                int32_t theta_q16, int32_t rho_stop_q8,
                int32_t* f3,
                int32_t* wk_f_prev, int32_t* wk_fbar, int32_t* wk_q,
                int32_t* wk_grad, int32_t* wk_divq, int32_t* wk_y,
                int32_t* wk_r, int32_t* wk_btr, int32_t* wk_tmp,
                int out_n_used[3], int out_early_stopped[3],
                int out_overflow_ch[3],
                unsigned int out_rho_q16[3],
                unsigned long long out_res_sq[3],
                unsigned long long out_d_sq[3],
                long long out_mac_count[3],
                int* sat_events, int* mac_overflow, int* out_overflow,
                int* score_overflow, int* gain_clamped,
                long long* max_abs_acc);

#ifdef __cplusplus
}   // extern "C"
#endif

#endif  // PENUMBRASIGHT_HLS_PS_FIXED_TOP_HPP
