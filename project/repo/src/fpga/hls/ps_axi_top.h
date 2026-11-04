// ============================================================================
// ps_axi_top.h —— HLS 顶层 AXI 接口层（A6）
//
// 为什么要有这一层
// ----------------------------------------------------------------------------
// 三个 HLS 核里的顶层壳（projection_engine_top / lr_forward_top /
// lr_adjoint_top / pdhg_core_top）是**主机验证签名**，不是硬件接口：
//
//   int lr_forward_top(const LrFactorsFp& F, const double* f, double* y);
//   int pdhg_core_top(const LrFactorsFp& F, ..., PdhgWorkFp& W, PdhgOutFp& out, ...);
//
// 它们有结构体传引用、有 `double*`、一条 `#pragma HLS INTERFACE` 都没有。
// 直接拿去做块设计，Vitis 会自行推断接口 —— 推断结果不可控，且**在没有
// Vivado 的机器上无法验证**。所以这里另起一层：
//
//   · 保留原顶层壳**一字不改**，主机侧 9000+ 行测试继续跑（make test-hls）；
//   · 新增本层的 4 个 `*_axi` 顶层，把入参保真成「平坦数组 + 标量」，
//     并显式绑定 m_axi / s_axilite；
//   · `*_axi` 内部只做一件事：把平坦入参拼回结构体，转调原顶层壳。
//
// 这样接口契约（地址、位宽、bundle、寄存器偏移）是**可审查、可 diff** 的，
// 而不是交给工具推断。
//
// ⚠ 本层交付的是「接口契约 + 可编译的包装」，**不是已验证的 IP**。
//    真正的综合、时序收敛、资源占用仍要等 Vivado。
//
// 一个尚未拍板的架构问题（见 docs/hls_interface_spec.md §六）
// ----------------------------------------------------------------------------
// 仓库里有两个数学层，**目前 build_hls.tcl 综合的是双精度层**：
//
//   双精度层  lr_forward_top / pdhg_core_top / projection_engine_top   ← 被综合
//             用 double，误差 4.1e-16
//   定点层    lr_forward / pdhg_core / projection_engine（*_fixed.cpp）
//             用 int16 系数码 + int32 累加，另有一套 test_*.cpp 在验
//
// XCZU9EG 的 DSP48E2 是 27x18 乘法器（全器件 2520 个，见 A2_资源预算.md），
// **双精度浮点做不了**（要软浮点或 3~4 个 DSP 拼一个 64 位乘）。
// 所以最终上板的必然是**定点层**。
// 本层按「当前 build_hls.tcl 综合的是谁」包了双精度层，**并在类型上留了
// 切换点**：见 PS_AXI_SAMPLE_T / PS_AXI_COEF_T 两个 typedef。
// 换层时只改这两个 typedef 与 §六 里列的入口，不动 bundle 与寄存器映射。
// ============================================================================

#ifndef PS_AXI_TOP_H
#define PS_AXI_TOP_H

#include <stdint.h>

// ---------------------------------------------------------------- 采样类型
// 这两个 typedef 是「换层开关」。当前 = 双精度层（与 build_hls.tcl 一致）。
// 切到定点层时改成 int32_t（采样码字）与 int16_t（Q1.15 系数码字）。
typedef double  PS_AXI_SAMPLE_T;   // 测量 / 图像样点
typedef double  PS_AXI_COEF_T;     // U / V 系数

// ---------------------------------------------------------------- 接口版本
// 块设计 tcl 与本头文件必须同版本。改了 bundle 或寄存器偏移就要 +1，
// 让 bd 脚本能检测到「IP 与脚本不匹配」而不是静默连错。
#define PS_AXI_IF_VERSION   1u

// ---------------------------------------------------------------- 寄存器映射
// 全部 s_axilite 寄存器挂在 bundle "control"，偏移按 4 字节对齐。
// **偏移冻结后不得重排**：块设计 tcl 与板端软件都按这些偏移寻址。
//
//   偏移   名称              读/写  含义
//   ------------------------------------------------------------------
//   0x000  CTRL              W       bit0 = ap_start（由 HLS 自动接管）
//                                    bit1 = 模式选择（0 = 走 AXI 顶层）
//   0x004  GIE               W       全局中断使能（HLS 自动）
//   0x008  IER               W       中断使能（HLS 自动）
//   0x00C  ISR               R       中断状态（HLS 自动）
//   0x010  IF_VER            R       接口版本 = PS_AXI_IF_VERSION
//   0x014  K_RANK            RW      秩 R
//   0x018  M_MEAS            RW      测量数 M
//   0x01C  N_HIDDEN          RW      隐藏像素数 N
//   0x020  P_CAND            RW      候选数 P
//   0x024  N_ROWS            RW       隐藏图行数（pdhg）
//   0x028  N_COLS            RW       隐藏图列数（pdhg）
//   0x02C  N_ITER            RW       迭代上限（pdhg）
//   0x030  MIN_ITERS         RW       早停下限（pdhg）
//   0x034  RES_EVERY         RW       残差评估间隔（pdhg）
//   0x038  RES_HIST_CAP      RW       残差历史容量（pdhg）
//   0x03C  LAM_Q16           RW       TV 权重 λ，Q16.16
//   0x040  TAU_Q16           RW       原始步长 τ，Q16.16
//   0x044  SIGMA_Q16         RW       对偶步长 σ，Q16.16
//   0x048  THETA_Q16         RW       惯性外推 θ，Q16.16
//   0x04C  RHO_STOP_Q16      RW       早停门限 ρ，Q16.16（<=0 不早停）
//   0x050  N_USED            R        实际迭代次数（= PdhgOutFp.n_used）
//   0x054  EARLY_STOPPED     R        1 = 因残差门限提前结束
//   0x058  N_RES             R        残差历史条数
//   0x05C  MONOTONE          R        1 = 残差历史单调不增
//   0x060  RHO_LO            R        最后一次 ρ 的低 32 位
//   0x064  RHO_HI            R        最后一次 ρ 的高 32 位
//   0x068  RHO_FIRST_LO      R        第一次 ρ 的低 32 位
//   0x06C  RHO_FIRST_HI      R        第一次 ρ 的高 32 位
//   0x070  RES_NORM_LO       R        ||Bf − d|| 低 32 位
//   0x074  RES_NORM_HI       R        ||Bf − d|| 高 32 位
//   0x078  D_NORM_LO         R        ||d|| 低 32 位
//   0x07C  D_NORM_HI         R        ||d|| 高 32 位
//   0x080  BEST_ID           R        最佳候选下标（定位）
//   0x084  MARGIN_LO         R        (s1-s2)/(s1+eps) 低 32 位（定位）
//   0x088  MARGIN_HI         R        高 32 位
//   0x08C  AP_DONE / AP_IDLE / AP_READY    R   HLS 自动
//
// 64 位的 double 结果拆成 LO/HI 两个 32 位寄存器，是为了不依赖 AXI4-Lite 的
// 64 位访问支持；板端软件按 `lo | (hi << 32)` 重组。

// ---------------------------------------------------------------- bundle 约定
//   gmem0_model : U / S / V / psm 模型体        —— IP 侧只读
//   gmem1_data  : 测量、图像、分数             —— IP 侧读写
//   gmem2_scratch : pdhg 的 8 个迭代暂存       —— IP 侧读写（见 §六 待优化项）
//   control     : 全部标量与返回码（s_axilite）

// ---------------------------------------------------------------- 返回码
// 与三个核既有的 kLr*/kProj*/kPdhg* 返回码保持同一套数值，
// 便于主机侧测试与板端软件用同一张表解读。
#define PS_AXI_OK            0
#define PS_AXI_ERR_ARGS     (-1)   // 入参非法（空指针、维度为 0、超范围）
#define PS_AXI_ERR_DIM      (-2)   // 维度组合不合法（如 K > rank_max）
#define PS_AXI_ERR_SHAPE    (-3)   // 形状与 N 不匹配

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 定位：低秩前向 Bf = U·diag(S)·(V f)
int ps_lr_forward_axi(const PS_AXI_COEF_T* U,
                      const PS_AXI_COEF_T* S,
                      const PS_AXI_COEF_T* V,
                      int K, int M, int N,
                      const PS_AXI_SAMPLE_T* f,
                      PS_AXI_SAMPLE_T* y);

// ---------------------------------------------------------------- 定位：低秩伴随 Bᵀr = Vᵀ·diag(S)·(Uᵀ r)
int ps_lr_adjoint_axi(const PS_AXI_COEF_T* U,
                      const PS_AXI_COEF_T* S,
                      const PS_AXI_COEF_T* V,
                      int K, int M, int N,
                      const PS_AXI_SAMPLE_T* r,
                      PS_AXI_SAMPLE_T* g);

// ---------------------------------------------------------------- 定位：候选打分与排序
// model_base 指向整个 .psm 模型体；h 指向其 64 字节头部。
// 结果为逐通道分数 + 总分 + 最佳候选 + 裕量，全部落回内存与寄存器。
int ps_projection_axi(const PS_AXI_SAMPLE_T* meas3,
                      const uint8_t*        model_base,
                      const void*           h,
                      int P, int K,
                      PS_AXI_SAMPLE_T* score_out,
                      PS_AXI_SAMPLE_T* score_ch,
                      int*   best_id,
                      double* margin);

// ---------------------------------------------------------------- 重建：PDHG 单通道
// 暂存全部由调用方传入（综合代码里不许有 new/delete）。
int ps_pdhg_axi(const PS_AXI_COEF_T* U,
                const PS_AXI_COEF_T* S,
                const PS_AXI_COEF_T* V,
                int K, int M, int N,
                const PS_AXI_SAMPLE_T* d,
                int n_rows, int n_cols,
                int n_iter, int min_iters, int residual_every,
                double tau, double sigma, double lam, double theta, double rho_stop,
                PS_AXI_SAMPLE_T* f,
                PS_AXI_SAMPLE_T* f_prev, PS_AXI_SAMPLE_T* fbar,
                PS_AXI_SAMPLE_T* q, PS_AXI_SAMPLE_T* grad,
                PS_AXI_SAMPLE_T* divq,
                PS_AXI_SAMPLE_T* bf, PS_AXI_SAMPLE_T* r,
                PS_AXI_SAMPLE_T* btr,
                double* res_hist, int res_hist_cap,
                // ---- 只写回寄存器/内存的输出 ----
                int* n_used, int* early_stopped, int* n_res, int* monotone,
                double* rho, double* rho_first, double* res_norm, double* d_norm);

#ifdef __cplusplus
}   // extern "C"
#endif

#endif  // PS_AXI_TOP_H
