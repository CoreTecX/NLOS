/* ============================================================================
 * PenumbraSight · 板端 PS 侧寄存器映射（**定点层**四个 HLS 核）
 * board/ps_fix_regs.h
 *
 * 给 ARM(PS) 侧软件用：裸寄存器读写，不依赖 Xilinx 生成的 driver。
 * 基址来自块设计 penumbrasight_bd.tcl §7；偏移来自四个 IP **自述**的
 * `<proj>/solution1/impl/ip/drivers/<ip>_v1_0/src/xps_<ip>_hw.h`。
 *
 * ⚠ 上面那一行原来写成了「drivers + 通配符目录 + src」，也就是斜杠、星号、
 *   斜杠三个字符相邻，其中后两个正好是块注释的结束标记，**把注释提前终止了**，
 *   后面所有中文都被当成代码。gcc 报的是 stray 反斜杠转义序列，看起来像编码
 *   问题，其实不是。写注释时避开任何「星号紧跟斜杠」的字符对 —— 尤其路径通配符。
 *
 * ----------------------------------------------------------------------------
 * ⚠ 这不是双精度层的那套
 * ----------------------------------------------------------------------------
 * 双精度层的寄存器在 `_archive/legacy_cpp_20260926/src/software/fpga_control/ps_regs.hpp`
 * 与 `src/fpga/hls/ps_axi_top.h`，**与这里不是一套**：标量个数、名字、偏移都不同。
 * 定点层四个顶层各有一份自己的 hw.h，本文件是它们的汇总。
 * 若两端混用，表现为「能启动作业但结果全是垃圾」，不会报错。
 *
 * ----------------------------------------------------------------------------
 * ⚠ 每个核有**两个** AXI-Lite 从接口，不是一个
 * ----------------------------------------------------------------------------
 *   s_axi_control    （基址见下表 .CTRL）  标量：尺寸、增益、标志位、回读结果
 *   s_axi_control_r  （基址见下表 .CTRL_R）**m_axi 缓冲区基址寄存器**
 *
 * 依据：IP 的 component.xml 有两个 memoryMap。实测行号（ps_projection_fix）：
 *   s_axi_control   的 addressBlock 在 1266~2661 行，装 P/K/M/...（1575~1746 行可见）
 *   s_axi_control_r 的 addressBlock 在 2665~2912 行，装 meas/u_codes/gain_q16/score_out
 * 只接前者时 `validate_bd_design` **照样通过、一条告警都没有**（2026-09-29 实测）。
 * 后果：核拿不到缓冲区基址，只会按地址 0 读写。
 *
 * ----------------------------------------------------------------------------
 * 使用顺序（每个核）
 * ----------------------------------------------------------------------------
 *   1) 把每个缓冲区的 64 位物理地址写进 CTRL_R 里的 LO/HI 两个 32 位寄存器
 *      （LO 在低偏移、HI 在 +0x4；先写 HI 再写 LO 更稳妥，避免中途被采样到半截地址）
 *   2) 把标量（尺寸、增益、迭代参数）写进 CTRL
 *   3) 写 CTRL 的 bit0 = ap_start 启动
 *   4) 轮询 CTRL 的 bit1 = ap_done（或等中断）
 *   5) 读回 CTRL 里带 `_VLD` 的结果寄存器：**先读数据再读 VLD**（VLD 是 COR，读清）
 *
 * ----------------------------------------------------------------------------
 * 未验证声明
 * ----------------------------------------------------------------------------
 * 本文件**尚未在板上跑过** —— PS 侧板端软件还没写。偏移来自 IP 自述头文件，
 * 与 RTL 同源，但「同源」不等于「已实测」。首次上板时建议用一个最小用例
 * （单帧、K/M/N 取小值）先验证「写基址 -> 启动 -> done -> 读回」这条链路。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_PS_FIX_REGS_H
#define PENUMBRASIGHT_PS_FIX_REGS_H

#include <stdint.h>

/* 块设计里四个核的控制段基址。**必须与 penumbrasight_bd.tcl 的 §7 一致**：
 * 核 i 占两段，s_axi_control 在 BASE+(2i)*0x10000、s_axi_control_r 在 +0x10000。
 * 段大小各 64 KiB。顺序 = bd 脚本里 core_names 的顺序。 */
#define PS_FIX_CORE_PROJ_CTRL      0xA0000000u   /* ps_projection_fix  */
#define PS_FIX_CORE_PROJ_CTRL_R    0xA0010000u
#define PS_FIX_CORE_LR_FWD_CTRL    0xA0020000u   /* ps_lr_forward_fix  */
#define PS_FIX_CORE_LR_FWD_CTRL_R  0xA0030000u
#define PS_FIX_CORE_LR_ADJ_CTRL    0xA0040000u   /* ps_lr_adjoint_fix  */
#define PS_FIX_CORE_LR_ADJ_CTRL_R  0xA0050000u
#define PS_FIX_CORE_PDHG_CTRL      0xA0060000u   /* ps_pdhg_fix        */
#define PS_FIX_CORE_PDHG_CTRL_R    0xA0070000u

/* 与 src/fpga/hls/ps_fixed_top.hpp 的 PS_FIX_IF_VERSION 同步。
 * 板端软件启动时应能自报这个值，便于与 FPGA 侧比对。 */
#define PS_FIX_IF_VERSION          1

/* ============================================================ 公共寄存器 */
/* 四个核的 s_axi_control 里，这几个偏移是 HLS 固定的 */
#define PS_FIX_REG_CTRL            0x000u  /* bit0 ap_start(RW) bit1 ap_done(COR)  */
                                           /* bit2 ap_idle(R)   bit3 ap_ready(COR) */
                                           /* bit7 auto_restart(RW) bit9 interrupt(R) */
#define PS_FIX_REG_GIE             0x004u  /* bit0 全局中断使能(RW) */
#define PS_FIX_REG_IER             0x008u  /* bit0 ap_done, bit1 ap_ready(RW) */
#define PS_FIX_REG_ISR             0x00Cu  /* 同上，读清(TOW) */
#define PS_FIX_REG_AP_RETURN       0x010u  /* 只读，函数返回值 */

#define PS_FIX_CTRL_AP_START       0x00000001u
#define PS_FIX_CTRL_AP_DONE        0x00000002u
#define PS_FIX_CTRL_AP_IDLE        0x00000004u
#define PS_FIX_CTRL_AP_READY       0x00000008u
#define PS_FIX_CTRL_AUTO_RESTART   0x00000080u

/* 每个「带 _VLD 的结果」都是 数据(+0x0) / VLD(+0x4) 相邻两个寄存器 */
#define PS_FIX_VLD_OFF             0x004u

/* 64 位量的两个 32 位半字：LO 在低偏移、HI 在 +0x4 */
#define PS_FIX_64_LO               0x000u
#define PS_FIX_64_HI               0x004u

/* ==========================================================================
 * 1) ps_projection_fix   —— 投影核（定位）
 *    s_axi_control   0xA0000000 + off
 *    s_axi_control_r 0xA0010000 + off   （缓冲区基址）
 * ========================================================================*/

/* --- s_axi_control：输入标量 --- */
#define PROJ_P                     0x018u
#define PROJ_K                     0x020u
#define PROJ_M                     0x028u
#define PROJ_NX                    0x030u
#define PROJ_NY                    0x038u
#define PROJ_NZ                    0x040u
#define PROJ_MARGIN_THRESH_Q16     0x048u

/* --- s_axi_control：回读结果（数据 + 紧跟的 VLD） --- */
#define PROJ_BEST_ID               0x050u
#define PROJ_SECOND_ID             0x060u
#define PROJ_MARGIN_Q16            0x070u
#define PROJ_BOUNDARY              0x080u
#define PROJ_LOW_CONF              0x090u
#define PROJ_OVERFLOW              0x0A0u
#define PROJ_BEST_SCORE            0x0B0u   /* uint64，LO/HI */
#define PROJ_SECOND_SCORE          0x0C8u   /* uint64，LO/HI */
#define PROJ_SAT_EVENTS            0x0E0u
#define PROJ_MAC_OVERFLOW          0x0F0u
#define PROJ_OUT_OVERFLOW          0x100u
#define PROJ_SCORE_OVERFLOW        0x110u
#define PROJ_GAIN_CLAMPED          0x120u
#define PROJ_MAX_ABS_ACC           0x130u   /* int64，LO/HI */

/* --- s_axi_control_r：m_axi 缓冲区基址（64 位，LO/HI） --- */
#define PROJ_BASE_MEAS             0x010u   /* const int32_t  meas[27360]      */
#define PROJ_BASE_U_CODES          0x01Cu   /* const int16_t  u_codes[17729280] */
#define PROJ_BASE_GAIN_Q16         0x028u   /* const int32_t  gain_q16[1024]   */
#define PROJ_BASE_SCORE_OUT        0x034u   /* uint64_t       score_out[1024]  */
#define PROJ_BASE_SCORE_CH         0x040u   /* uint64_t       score_ch[3072]   */

/* ==========================================================================
 * 2) ps_lr_forward_fix   —— 低秩前向 Bx
 *    s_axi_control   0xA0020000 + off
 *    s_axi_control_r 0xA0030000 + off
 * ========================================================================*/

#define LRF_K                      0x018u
#define LRF_M                      0x020u
#define LRF_N                      0x028u
#define LRF_U_GAIN_Q16             0x030u
#define LRF_S_GAIN_Q16             0x038u
#define LRF_V_GAIN_Q16             0x040u
#define LRF_U_EXP                  0x048u
#define LRF_S_EXP                  0x050u
#define LRF_V_EXP                  0x058u

#define LRF_MAC_COUNT              0x060u   /* int64，LO/HI */
#define LRF_SAT_EVENTS             0x078u
#define LRF_MAC_OVERFLOW           0x088u
#define LRF_OUT_OVERFLOW           0x098u
#define LRF_SCORE_OVERFLOW         0x0A8u
#define LRF_GAIN_CLAMPED           0x0B8u
#define LRF_MAX_ABS_ACC            0x0C8u   /* int64，LO/HI */

#define LRF_BASE_U                 0x010u   /* const int16_t  U[583680]  */
#define LRF_BASE_S                 0x01Cu   /* const uint16_t S[96]      */
#define LRF_BASE_V                 0x028u   /* const int16_t  V[27648]   */
#define LRF_BASE_F                 0x034u   /* const int32_t  f[1024]    */
#define LRF_BASE_Y                 0x040u   /* int32_t        y[16384]   */
#define LRF_BASE_WK_Z              0x04Cu   /* int32_t        wk_z[96]   */
#define LRF_BASE_WK_ZS             0x058u   /* int32_t        wk_zs[96]  */

/* ==========================================================================
 * 3) ps_lr_adjoint_fix   —— 低秩伴随 B^T y
 *    s_axi_control   0xA0040000 + off     （标量布局与 LRF **完全相同**）
 *    s_axi_control_r 0xA0050000 + off
 * ========================================================================*/

#define LRA_K                      0x018u
#define LRA_M                      0x020u
#define LRA_N                      0x028u
#define LRA_U_GAIN_Q16             0x030u
#define LRA_S_GAIN_Q16             0x038u
#define LRA_V_GAIN_Q16             0x040u
#define LRA_U_EXP                  0x048u
#define LRA_S_EXP                  0x050u
#define LRA_V_EXP                  0x058u
#define LRA_MAC_COUNT              0x060u
#define LRA_SAT_EVENTS             0x078u
#define LRA_MAC_OVERFLOW           0x088u
#define LRA_OUT_OVERFLOW           0x098u
#define LRA_SCORE_OVERFLOW         0x0A8u
#define LRA_GAIN_CLAMPED           0x0B8u
#define LRA_MAX_ABS_ACC            0x0C8u

#define LRA_BASE_U                 0x010u
#define LRA_BASE_S                 0x01Cu
#define LRA_BASE_V                 0x028u
#define LRA_BASE_R                 0x034u   /* const int32_t r[16384] */
#define LRA_BASE_G                 0x040u   /* int32_t       g[1024]  */
#define LRA_BASE_WK_W              0x04Cu   /* int32_t       wk_w[96] */
#define LRA_BASE_WK_WS             0x058u   /* int32_t       wk_ws[96] */

/* ==========================================================================
 * 4) ps_pdhg_fix   —— PDHG 重建迭代
 *    s_axi_control   0xA0060000 + off
 *    s_axi_control_r 0xA0070000 + off
 * ========================================================================*/

#define PDHG_K                     0x018u
#define PDHG_M                     0x020u
#define PDHG_N                     0x028u
#define PDHG_U_GAIN_Q16            0x030u
#define PDHG_S_GAIN_Q16            0x038u
#define PDHG_V_GAIN_Q16            0x040u
#define PDHG_U_EXP                 0x048u
#define PDHG_S_EXP                 0x050u
#define PDHG_V_EXP                 0x058u
#define PDHG_N_ROWS                0x060u
#define PDHG_N_COLS                0x068u
#define PDHG_N_ITER                0x070u
#define PDHG_MIN_ITERS             0x078u
#define PDHG_RESIDUAL_EVERY        0x080u
#define PDHG_TV_MODE               0x088u
#define PDHG_TAU_Q16               0x090u
#define PDHG_SIGMA_Q16             0x098u
#define PDHG_LAMBDA_Q16            0x0A0u
#define PDHG_THETA_Q16             0x0A8u
#define PDHG_RHO_STOP_Q8           0x0B0u

#define PDHG_SAT_EVENTS            0x0B8u
#define PDHG_MAC_OVERFLOW          0x0C8u
#define PDHG_OUT_OVERFLOW          0x0D8u
#define PDHG_SCORE_OVERFLOW        0x0E8u
#define PDHG_GAIN_CLAMPED          0x0F8u
#define PDHG_MAX_ABS_ACC           0x108u   /* int64，LO/HI */

/* 每通道结果是**指针参数**（`int out_n_used[3]` 之类的数组形参在 HLS 里变成 m_axi），
 * 所以它们在 s_axi_control_r 里是**基址**，不是数值。真实内容在 DDR 里，去基址读。
 * 按位宽选读取类型：
 *   out_n_used / out_early_stopped / out_overflow_ch -> int32_t[3]
 *   out_rho_q16                                       -> uint32_t[3]
 *   out_res_sq / out_d_sq / out_mac_count             -> 64 位[3]（unsigned long long / long long） */
#define PDHG_BASE_OUT_N_USED       0x0B8u
#define PDHG_BASE_OUT_EARLY_STOPPED 0x0C4u
#define PDHG_BASE_OUT_OVERFLOW_CH  0x0D0u
#define PDHG_BASE_OUT_RHO_Q16      0x0DCu
#define PDHG_BASE_OUT_RES_SQ       0x0E8u
#define PDHG_BASE_OUT_D_SQ         0x0F4u
#define PDHG_BASE_OUT_MAC_COUNT    0x100u

#define PDHG_BASE_U                0x010u
#define PDHG_BASE_S                0x01Cu
#define PDHG_BASE_V                0x028u
#define PDHG_BASE_D3               0x034u
#define PDHG_BASE_F3               0x040u
#define PDHG_BASE_WK_F_PREV        0x04Cu
#define PDHG_BASE_WK_FBAR          0x058u
#define PDHG_BASE_WK_Q             0x064u
#define PDHG_BASE_WK_GRAD          0x070u
#define PDHG_BASE_WK_DIVQ          0x07Cu
#define PDHG_BASE_WK_Y             0x088u
#define PDHG_BASE_WK_R             0x094u
#define PDHG_BASE_WK_BTR           0x0A0u
#define PDHG_BASE_WK_TMP           0x0ACu

static inline void ps_fix_write32(volatile uint32_t *base, uint32_t off, uint32_t v)
{
    base[off >> 2] = v;
}

static inline uint32_t ps_fix_read32(volatile uint32_t *base, uint32_t off)
{
    return base[off >> 2];
}

/* 写 64 位缓冲区基址：**先写 HI 再写 LO**。
 * 理由：核在 ap_start 后的第一个时钟才开始取地址，但若先写 LO，硬件可能先看到
 * 「新 LO + 旧 HI」这种半截地址。先写 HI 能保证 LO 落笔时 HI 已是新值。 */
static inline void ps_fix_write_base64(volatile uint32_t *ctrl_r, uint32_t off, uint64_t addr)
{
    ps_fix_write32(ctrl_r, off + PS_FIX_64_HI, (uint32_t)(addr >> 32));
    ps_fix_write32(ctrl_r, off + PS_FIX_64_LO, (uint32_t)(addr & 0xFFFFFFFFu));
}

/* 读「带 VLD 的结果寄存器」：返回 1 表示本次读到有效新值。
 * VLD 是 ap_vld，语义为「COR：读清」，故**先读数据、再读 VLD**。 */
static inline int ps_fix_read_result(volatile uint32_t *ctrl, uint32_t off, uint32_t *out)
{
    uint32_t d = ps_fix_read32(ctrl, off + PS_FIX_64_LO);
    uint32_t v = ps_fix_read32(ctrl, off + PS_FIX_VLD_OFF);
    if (out) { *out = d; }
    return (v & 0x1u) ? 1 : 0;
}

#endif /* PENUMBRASIGHT_PS_FIX_REGS_H */
