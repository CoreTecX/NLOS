/* ============================================================================
 * PenumbraSight · 裸机最小系统（JTAG / XSCT 下运行）· 编译期配置
 * src/software/baremetal/baremetal_app.h
 *
 * 本头只放「可以用命令行 -D 覆盖的开关」与「判据里用到的期望常量」。
 * 所有寄存器偏移与基址一律来自 board/ps_fix_regs.h 与 board/ps_pl_regs.h，
 * 本文件**不复制**任何偏移或基址 —— 那正是那次「抄错一个偏移就一起错」的教训。
 *
 * 未上板声明
 * ---------------------------------------------------------------------------
 * 本程序与本节所有期望常量**尚未在板上跑过**。期望值由 HLS 定点核的源码逐级
 * 手算得到（推导见下），本文件不含任何实测结论。
 * ==========================================================================*/
#ifndef PENUMBRASIGHT_BAREMETAL_APP_H
#define PENUMBRASIGHT_BAREMETAL_APP_H

#include <stdint.h>

/* ---------------------------------------------------------------- 覆盖开关 */

/* 版本读回的地址探测开关。PL 控制页（ps_regs）在 2026-09-30 的位流里不存在，
 * 访问未映射的 AXI 地址可能触发 Data Abort。置 0 可在旧位流上跳过触页。
 * 默认 1（按任务要求做真正的版本读回）。 */
#ifndef BARE_PROBE_PL_PAGE
#define BARE_PROBE_PL_PAGE 1
#endif

/* Data Abort 兜底：探测 PL 控制页时若发生同步异常，捕获后跳过该条 load，
 * 把结果标成 UNMAPPED，而不是让程序停在异常里。仅 aarch64 + Xilinx BSP 下编译。
 * 默认 1。 */
#ifndef BARE_ABORT_GUARD
#define BARE_ABORT_GUARD 1
#endif

/* pl_clk0 的自由运行计数器地址（PL 侧）。
 * **本寄存器当前不存在**：board/ps_pl_regs.h 的 0xA00A0000 页里没有「按 pl_clk0
 * 递增」的寄存器（只有 frame_done 计数，见 PS_PL_CTR_RAW..PS_PL_CTR_DISP）。
 * 因此默认留空 —— 空格子意味着「无法测频、如实报 SKIPPED」，而不是编一个数。
 * 若并行任务在 PL 控制页里补一个自由运行计数器，用 -DBARE_PL_COUNTER_ADDR=0xA00AXXXX
 * 打开（见 README「pl_clk0 的测法」与报告 §3）。 */
#ifdef BARE_PL_COUNTER_ADDR
#define BARE_HAS_PL_COUNTER 1
#else
#define BARE_HAS_PL_COUNTER 0
#endif

/* 测频门宽（毫秒）。越大越准；10 ms 在 100 MHz 参考下已有足够余量，
 * 1 s 可给出决定性判据（分析见 README 与报告 §3）。 */
#ifndef BARE_PL_GATE_MS
#define BARE_PL_GATE_MS 1000u
#endif

/* 无 PL 计数器时，退化诊断测多少次 AXI-Lite 读（只为证明 PL 时钟域活着，
 * 不构成频率结论）。设为 0 可关闭。 */
#ifndef BARE_AXI_DIAG_READS
#define BARE_AXI_DIAG_READS 100000u
#endif

/* 单核最小用例选的是 ps_lr_forward_fix（见报告 §4 的选择理由）。 */
#ifndef BARE_MIN_POLL_LIMIT
#define BARE_MIN_POLL_LIMIT 20000000u   /* 纯忙等轮询次数上限，非时间 */
#endif
#ifndef BARE_FOURCORE_POLL_LIMIT
#define BARE_FOURCORE_POLL_LIMIT 20000000u
#endif

/* 硬件的 LANES。ps_fixed_top.hpp:87-89 默认 16；用于算 mac_count 期望值。
 * 位流若按 -DPS_FIX_LANES=8/32 综合，这里改同名宏即可。 */
#ifndef PS_FIX_LANES
#define PS_FIX_LANES 16
#endif

/* ---------------------------------------------------------------- 版本判据 */

/* board/ps_fix_regs.h:71 与 src/fpga/hls/ps_fixed_top.hpp:82 均为 1。
 * PL 控制页的 VERSION 是 RTL 常量 1（pl_ctrl_regs.v:379/745），ID 是
 * 0x50530001（pl_ctrl_regs.v:378/744；board/ps_pl_regs.h:89-90）。 */
#define BARE_EXPECT_IF_VERSION   1u
#define BARE_EXPECT_PL_ID        0x50530001u
#define BARE_EXPECT_PL_VERSION   1u

/* ---------------------------------------------------------------- 最小用例
 *
 * 选 ps_lr_forward_fix，几何 K=1 / M=4 / N=1，全部增益走 Q16.16 支路
 * （*_exp 写 kGainExpInvalid = 0x7FFFFFFF，见 ps_fixed.hpp:431），
 * 且三个增益都取 65536（= kGainSafeMax，ps_fixed.hpp:81，恰好不被钳）。
 *
 * 逐级推导（依据 lr_operator_fixed.cpp:140-181 与 lr_operator.hpp:42-55）：
 *   阶段1 rows : acc = V[0]*f[0] = 1*1 = 1
 *                z   = mul_shift_round(1, 65536, 16) = 1
 *   阶段2 S∘z  : t = z*S[0] = 1*16384 = 16384
 *                zs  = mul_shift_round(16384, 65536, 16) = 16384
 *   阶段3 cols : acc = U[m]*zs[0] = U[m]*16384
 *                y[m]= mul_shift_round(acc, 65536, 18) = round(acc/4)
 *                    = U[m]*4096                      （整除，无舍入）
 * 取 U = {1,2,3,4} -> y = {4096, 8192, 12288, 16384}。
 * 上界检查：max(y)=16384 <= kMeasCodeMax=131071（ps_fixed.hpp:73），不饱和。
 *
 * mac_count：rank_major_rows 加 L=N 次；rank_major_cols_gain 每个 l 分块对每行
 * 加 LANES 次（lr_operator_fixed.cpp:60,89）。M=4 < LANES 时只有 1 个分块，
 * 故 mac_count = N + K*LANES = 1 + 16 = 17（LANES=16）。
 */
#define BARE_MIN_K   1u
#define BARE_MIN_M   4u
#define BARE_MIN_N   1u

#define BARE_MIN_S_CODE     16384u   /* 1.0 的 Q2.14（lr_operator.hpp:23） */
#define BARE_MIN_GAIN_Q16   65536    /* 1.0 的 Q16.16；gain_safe 上界，不钳 */
#define BARE_MIN_GAIN_EXP   ((int32_t)0x7FFFFFFF)  /* kGainExpInvalid */

#define BARE_MIN_Y0_EXPECT  4096
#define BARE_MIN_Y1_EXPECT  8192
#define BARE_MIN_Y2_EXPECT  12288
#define BARE_MIN_Y3_EXPECT  16384
#define BARE_MIN_MAC_EXPECT (BARE_MIN_N + (BARE_MIN_K * (uint32_t)PS_FIX_LANES))

/* ---------------------------------------------------------------- 四核串联 */

/* board_regseq.c 实际生成的条目数（本程序按同序逐条回放，只是尺寸取小值）。
 * 逐条清点：版本读 1 + proj 19 + fwd 25 + adj 25 + pdhg 64 = 134 条
 * （其中 WRITE 129、POLL 4、READ 1；40 个 64 位基址）。
 * 注意：docs/board_top_app.md §四 的「130 步」与实际不符，见报告 §5。 */
#define BARE_REGSEQ_READ_STEPS   1u
#define BARE_REGSEQ_PROJ_STEPS   19u
#define BARE_REGSEQ_FWD_STEPS    25u
#define BARE_REGSEQ_ADJ_STEPS    25u
#define BARE_REGSEQ_PDHG_STEPS   64u
#define BARE_REGSEQ_TOTAL_STEPS  (BARE_REGSEQ_READ_STEPS + BARE_REGSEQ_PROJ_STEPS + \
                                  BARE_REGSEQ_FWD_STEPS + BARE_REGSEQ_ADJ_STEPS + \
                                  BARE_REGSEQ_PDHG_STEPS)

/* DDR 缓冲池：静态数组，4096 对齐；standalone 的 MMU 把 DDR 恒等映射，
 * 数组地址即 64 位物理地址，PL 经 HP 端口可直接访问（regmap §六 第 4 条）。 */
#define BARE_ARENA_BYTES (256u * 1024u)

#endif /* PENUMBRASIGHT_BAREMETAL_APP_H */
