/* ============================================================================
 * PenumbraSight · 板端 PS 侧寄存器层 API
 * board/fix_core.h
 *
 * 这一层只做一件事：把 board/ps_fix_regs.h 里的裸偏移包成
 * 「配置 -> 启动 -> 等待 -> 读结果」四个动作，四个定点核共用同一套骨架。
 *
 * ----------------------------------------------------------------------------
 * 边界（重要）
 * ----------------------------------------------------------------------------
 * 本层**不负责**：
 *   · 打开 /dev/mem 或做 mmap —— 由调用方做（见 ps_fix_core_open 的参数说明）
 *   · 分配 DDR 缓冲区、做物理地址与虚拟地址的换算
 *   · 加载模型（U/S/V/u_codes）、把相机帧搬进 meas
 *   · 打印、日志、错误上报
 * 这些都与「PS 跑的是 Linux 还是裸机」强相关，选定环境后再补。
 * 本层刻意不依赖任何 Xilinx 头文件，因此既能进 PetaLinux 的用户态程序，
 * 也能进裸机的 Xil_Out32 封装（把 window 传成对应基址即可）。
 *
 * ----------------------------------------------------------------------------
 * 未验证声明
 * ----------------------------------------------------------------------------
 * **尚未在板上跑过。** 寄存器偏移来自 IP 自述的 xps_*_fix_hw.h（与 RTL 同源），
 * 但同源不等于已实测。首次上板请用最小用例跑通
 * 「写基址 -> 写标量 -> ap_start -> ap_done -> 读回」这条链路再上真实尺寸。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_FIX_CORE_H
#define PENUMBRASIGHT_FIX_CORE_H

#include <stdint.h>
#include "ps_fix_regs.h"

/* 写序观测钩子（主机自检用）。ps_fix_set_base 在每次寄存器写**之前**调用它，
 * 于是「先 HI 再 LO」这条纪律成为可断言的 —— 影子窗口只保留最终值，看不出顺序。
 * 生产路径两字段恒为 NULL（ps_fix_core_open 清零），不产生任何副作用。 */
typedef void (*ps_fix_write_obs_fn)(void *ctx, uint32_t off, uint32_t val);

/* 一个定点核的控制句柄。两个指针指向**同一段映射窗口**里的不同偏移，
 * 只是分开存免得到处算。 */
typedef struct {
    volatile uint32_t *ctrl;        /* s_axi_control   的映射地址 */
    volatile uint32_t *ctrl_r;      /* s_axi_control_r 的映射地址 */
    uint32_t           ctrl_phys;   /* 物理基址，日志与自检用 */
    uint32_t           ctrl_r_phys;
    ps_fix_write_obs_fn write_obs;      /* 可选写序观测（NULL = 关闭） */
    void               *write_obs_ctx;  /* 传给 write_obs 的上下文 */
    uint64_t (*clock_ns)(void *); /* optional monotonic clock; timeout is wall time */
    void *clock_ctx;
    uint64_t timeout_ns;
} ps_fix_core_t;

/* 通用返回码 */
#define PS_FIX_OK              0
#define PS_FIX_ERR_TIMEOUT    -1
#define PS_FIX_ERR_ARG        -2

/* ---------------------------------------------------------------------------
 * 打开一个核
 *   ctrl_phys / ctrl_r_phys : 块设计里的基址（见 ps_fix_regs.h 的宏）
 *   window                  : 调用方 mmap 好的映射窗口首地址
 *   window_phys             : 该窗口对应的物理基址
 * 返回 PS_FIX_OK / PS_FIX_ERR_ARG。
 *
 * 例（Linux）：
 *   volatile uint32_t *w = mmap(...);           // 覆盖 0xA0000000 起的 512 KiB
 *   ps_fix_core_open(&proj, PS_FIX_CORE_PROJ_CTRL, PS_FIX_CORE_PROJ_CTRL_R, w, 0xA0000000);
 * ------------------------------------------------------------------------- */
int ps_fix_core_open(ps_fix_core_t *c, uint32_t ctrl_phys, uint32_t ctrl_r_phys,
                     volatile uint32_t *window, uint32_t window_phys);

/* 启动并等待。poll_limit 是轮询次数上限（不依赖任何 OS 计时器）。
 * 返回 PS_FIX_OK / PS_FIX_ERR_TIMEOUT。 */
int ps_fix_start_and_wait(ps_fix_core_t *c, uint32_t poll_limit);

/* 写 64 位基址（先 HI 再 LO），o 用 ps_fix_regs.h 里的 *_BASE_* 宏 */
void ps_fix_set_base(ps_fix_core_t *c, uint32_t o, uint64_t addr);

/* 读一个「数据 + 紧跟 VLD」的结果寄存器。返回 1 表示本次读到有效新值。 */
int ps_fix_read_vld(ps_fix_core_t *c, uint32_t o, uint32_t *out);

/* 读一个「64 位数据 + 后面跟 VLD」的结果。hi_off 为 VLD 所在偏移（即 o+0xC）。
 * 返回 1 表示有效。 */
int ps_fix_read_vld64(ps_fix_core_t *c, uint32_t o, uint64_t *out);

/* ==========================================================================
 * 投影核（定位）ps_projection_fix —— ctrl 0xA0000000 / ctrl_r 0xA0010000
 * ========================================================================*/

typedef struct {
    uint32_t best_id, second_id;
    int32_t  margin_q16;
    uint32_t boundary, low_conf, overflow;
    uint64_t best_score, second_score;   /* 单位与 RTL 一致 */
    uint32_t sat_events, mac_overflow, out_overflow, score_overflow, gain_clamped;
    int64_t  max_abs_acc;
} ps_fix_proj_result_t;

/* 只配基址与标量，不启动 */
int ps_fix_proj_setup(ps_fix_core_t *c,
                      uint64_t meas, uint64_t u_codes, uint64_t gain_q16,
                      uint64_t score_out, uint64_t score_ch,
                      uint32_t P, uint32_t K, uint32_t M,
                      uint32_t nx, uint32_t ny, uint32_t nz,
                      uint32_t margin_thresh_q16);

/* 启动 -> 等待 -> 读回全部带 VLD 的结果。 */
int ps_fix_proj_run(ps_fix_core_t *c, uint32_t poll_limit, ps_fix_proj_result_t *r);

/* ==========================================================================
 * 低秩前向 / 伴随 ps_lr_forward_fix / ps_lr_adjoint_fix
 *   forward  ctrl 0xA0020000 / ctrl_r 0xA0030000
 *   adjoint  ctrl 0xA0040000 / ctrl_r 0xA0050000
 *   两者**标量区布局完全相同**，只有缓冲区别名与基址不同。
 * ========================================================================*/

typedef struct {
    int64_t  mac_count;
    uint32_t sat_events, mac_overflow, out_overflow, score_overflow, gain_clamped;
    int64_t  max_abs_acc;
} ps_fix_lr_result_t;

/* 增益与指数对四个核是同一套编码（见 docs/gain_representation.md） */
typedef struct {
    int32_t  u_gain_q16, s_gain_q16, v_gain_q16;
    int32_t  u_exp, s_exp, v_exp;
} ps_fix_gain_t;

int ps_fix_lr_fwd_setup(ps_fix_core_t *c,
                        uint64_t U, uint64_t S, uint64_t V,
                        uint64_t f, uint64_t y, uint64_t wk_z, uint64_t wk_zs,
                        uint32_t K, uint32_t M, uint32_t N,
                        const ps_fix_gain_t *g);

int ps_fix_lr_fwd_run(ps_fix_core_t *c, uint32_t poll_limit, ps_fix_lr_result_t *r);

int ps_fix_lr_adj_setup(ps_fix_core_t *c,
                        uint64_t U, uint64_t S, uint64_t V,
                        uint64_t r, uint64_t g, uint64_t wk_w, uint64_t wk_ws,
                        uint32_t K, uint32_t M, uint32_t N,
                        const ps_fix_gain_t *gain);

int ps_fix_lr_adj_run(ps_fix_core_t *c, uint32_t poll_limit, ps_fix_lr_result_t *r);

/* ==========================================================================
 * 重建 ps_pdhg_fix —— ctrl 0xA0060000 / ctrl_r 0xA0070000
 * ========================================================================*/

typedef struct {
    uint32_t n_rows, n_cols, n_iter, min_iters, residual_every, tv_mode;
    int32_t  tau_q16, sigma_q16, lambda_q16, theta_q16, rho_stop_q8;
} ps_fix_pdhg_params_t;

typedef struct {
    uint32_t sat_events, mac_overflow, out_overflow, score_overflow, gain_clamped;
    int64_t  max_abs_acc;
} ps_fix_pdhg_result_t;

/* 暂存缓冲区的基址。九个 wk_* 的 depth 见 sim/results/ps_fix_regmap.md §4.4 */
typedef struct {
    uint64_t wk_f_prev, wk_fbar, wk_q, wk_grad, wk_divq;
    uint64_t wk_y, wk_r, wk_btr, wk_tmp;
} ps_fix_pdhg_scratch_t;

/* 每通道输出缓冲区的基址：都是 3 元素数组（指针形参在 HLS 里是 m_axi） */
typedef struct {
    uint64_t out_n_used, out_early_stopped, out_overflow_ch;   /* int32_t[3]  */
    uint64_t out_rho_q16;                                      /* uint32_t[3] */
    uint64_t out_res_sq, out_d_sq, out_mac_count;              /* 64 位[3]    */
} ps_fix_pdhg_outputs_t;

int ps_fix_pdhg_setup(ps_fix_core_t *c,
                      uint64_t U, uint64_t S, uint64_t V,
                      uint64_t d3, uint64_t f3,
                      const ps_fix_pdhg_scratch_t *scratch,
                      const ps_fix_pdhg_outputs_t *out,
                      uint32_t K, uint32_t M, uint32_t N,
                      const ps_fix_gain_t *gain,
                      const ps_fix_pdhg_params_t *p);

int ps_fix_pdhg_run(ps_fix_core_t *c, uint32_t poll_limit, ps_fix_pdhg_result_t *r);

/* PDHG 的每通道结果：**这些在 DDR 里，不在寄存器里**。
 * 寄存器里只有它们的基址（见 ps_fix_regs.h 的 PDHG_BASE_OUT_*），
 * 所以要先把基址按调用方自己的映射方式转成指针，再取数。 */
typedef struct {
    int32_t  n_used[3];
    int32_t  early_stopped[3];
    int32_t  overflow_ch[3];
    uint32_t rho_q16[3];
    uint64_t res_sq[3];
    uint64_t d_sq[3];
    uint64_t mac_count[3];
} ps_fix_pdhg_channels_t;

/* 从 DDR 里搬回每通道结果。七个 src 是**已映射**的指针（本层不碰 mmap）。
 * n 是通道数（<=3）。不做换算，按各自位宽逐元素拷贝。 */
void ps_fix_pdhg_collect_channels(const int32_t  *n_used,
                                  const int32_t  *early_stopped,
                                  const int32_t  *overflow_ch,
                                  const uint32_t *rho_q16,
                                  const uint64_t *res_sq,
                                  const uint64_t *d_sq,
                                  const uint64_t *mac_count,
                                  int n, ps_fix_pdhg_channels_t *dst);

#endif /* PENUMBRASIGHT_FIX_CORE_H */
