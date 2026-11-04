/* ============================================================================
 * PenumbraSight · 板端顶层应用「寄存器初始化序列」——可读清单
 * src/software/board_app/board_regseq.h
 *
 * 这一层把「上板前逐条核对」需要的东西做成一等公民：一条序列 = 一串带名字的
 * 步骤（写/读/轮询/分隔），每条都写明**顺序、绝对地址、写值或期望值**。
 * 它不产生任何硬件行为（除 ps_board_regseq_apply_writes 在影子窗口上回放）；
 * 它是给人和给主机自检共同看的同一份清单。
 *
 * 为什么要有它，而不是「照 fix_core_driver 的代码看一遍」
 * ---------------------------------------------------------------------------
 * fix_core_driver 只把寄存器写对，不负责**说清**写了什么、按什么顺序。板上
 * 逐条核对时，人手里需要一张表：第几步、哪个核、哪个段、偏移、写什么值。
 * 这张表若是从驱动代码「临场脑补」，与代码一漂移就又变成现场排查。
 * 所以这里让清单**机器生成**，并在主机侧与驱动实际写入的寄存器逐条比对
 * （见 src/software/test_board_top.c 的 C 段），使「清单 = 代码」成为可判据。
 *
 * 顺序约束（与 board/ps_fix_regs.h 的「使用顺序」一致）
 * ---------------------------------------------------------------------------
 *   1) 每个 64 位基址拆成 HI(+0x4) / LO(+0x0) 两步，**先 HI 再 LO**；
 *   2) 基址写进 s_axi_control_r 段（核基址 +0x10000），标量写进 s_axi_control 段；
 *   3) 标量在基址之后写；
 *   4) 每个核最后写 CTRL 的 ap_start；
 *   5) PS_FIX_IF_VERSION 读回步骤排在整段序列最前（上电第一件事）。
 *
 * 依赖纪律
 * ---------------------------------------------------------------------------
 *   只依赖 <stdint.h>/<stddef.h> + fix_core_driver.h（作业结构、窗口、四核 ID）。
 *   不做 I/O、不分配、不打印到 stdout（打印走 ps_board_regseq_print 的缓冲区）。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_BOARD_REGSEQ_H
#define PENUMBRASIGHT_BOARD_REGSEQ_H

#include <stddef.h>
#include <stdint.h>

#include "fix_core_driver.h"   /* 作业结构 / 窗口 / 四核 ID / 段基址宏 */

#ifdef __cplusplus
extern "C" {
#endif

/* PL 控制页（块设计里的 `ps_regs` 单元，源 `src/fpga/rtl/pl_ctrl_regs.v`）。
 * 段基址接在四核八段 + 两个 DMA 段之后 = 0xA00A0000；寄存器表见 pl_ctrl_regs.v：
 *   0x000 RO   ID      = 0x5053_0001
 *   0x004 RO   VERSION = PS_FIX_IF_VERSION
 * 版本的读回步骤指向 0xA00A0004（VERSION），存在性探测可先读 0xA00A0000（ID）。
 *
 * ⚠ 该 PL 控制页是 2026-09-30 的块设计脚本（工作区）新增的；**已生成的 17 MB 位流
 *   早于它**，因此在那份位流上这两个读回都不可达（UNMAPPED）。要真正读回，需要
 *   重新综合/实现位流，并把窗口开到覆盖 0xA00A0000 段。详见 docs/board_top_app.md §4。 */
#define PS_BOARD_REGSEQ_PL_ID_ADDR    0xA00A0000u
#define PS_BOARD_REGSEQ_IFVER_ADDR    0xA00A0004u
#define PS_BOARD_PL_ID_EXPECT         0x50530001u

/* ---------------------------------------------------------------- 步骤类型 */

#define PS_REGSEQ_WRITE 0u   /* 写寄存器：value = 写入值 */
#define PS_REGSEQ_READ  1u   /* 读寄存器：value = 期望值 */
#define PS_REGSEQ_POLL  2u   /* 轮询直到置位：value = 等待的位掩码 */
#define PS_REGSEQ_MARK  3u   /* 分隔/注释，不产生总线动作 */

/* 地址空间标签（只用于打印与人工核对；实际寻址一律用 step.addr）。 */
#define PS_REGSEQ_SP_CTRL   0u   /* 核的 s_axi_control 段 */
#define PS_REGSEQ_SP_CTRL_R 1u   /* 核的 s_axi_control_r 段 */
#define PS_REGSEQ_SP_ABS    2u   /* 绝对物理地址（预留的只读自检寄存器等） */

/* ---------------------------------------------------------------- 步骤条目 */

typedef struct {
    uint32_t    kind;     /* PS_REGSEQ_* */
    int         core;     /* PS_FIX_CORE_ID_*；全局/预留寄存器为 -1 */
    uint32_t    space;    /* PS_REGSEQ_SP_* */
    uint64_t    addr;     /* 绝对物理地址 */
    uint32_t    offset;   /* 段内偏移；SP_ABS 时为 0 */
    uint32_t    value;    /* 写值 / 期望值 / 轮询掩码 */
    const char *name;     /* 寄存器名（来自 ps_fix_regs.h 的宏名，可直接搜） */
} ps_board_regseq_step_t;

/* 上限：四核全量初始化约 130 步（含每个 64 位基址拆两步），留足余量。 */
#define PS_BOARD_REGSEQ_MAX 320

typedef struct {
    ps_board_regseq_step_t step[PS_BOARD_REGSEQ_MAX];
    int                    n;
    int                    overflow;   /* 1 = 步骤超上限，清单不完整 */
} ps_board_regseq_t;

/* ---------------------------------------------------------------- 生成配置 */

/* 指针为 NULL 表示该核不加入序列（与 ps_fix_run_sequence 的跳过语义一致）。 */
typedef struct {
    const ps_fix_proj_job_t *proj;
    const ps_fix_lrf_job_t  *fwd;
    const ps_fix_lra_job_t  *adj;
    const ps_fix_pdhg_job_t *pdhg;
    int                      include_version;  /* 1 = 序列最前加版本读回步骤 */
    uint32_t                 if_version;       /* 该读回步骤的期望值 */
} ps_board_regseq_cfg_t;

/* ---------------------------------------------------------------- API */

/* 清空序列。 */
void ps_board_regseq_reset(ps_board_regseq_t *seq);

/* 按固定顺序 proj -> fwd -> adj -> pdhg 生成清单。
 * 返回 0；seq/cfg 为空返回 -1；步骤超上限时把 seq->overflow 置 1（清单不完整，
 * 调用方应据此判失败，而不是用一个被截断的清单去核对）。 */
int ps_board_regseq_build(ps_board_regseq_t *seq, const ps_board_regseq_cfg_t *cfg);

/* 把序列里的 WRITE 步骤回放到映射窗口（主机影子窗口或板上 mmap 窗口）。
 * window 覆盖不到的地址被**跳过并计数**（不静默：返回值给出实际写入条数，
 * 调用方可用 ps_board_regseq_count_kind 算出应有条数再比对）。
 * READ / POLL / MARK 不产生写动作。返回实际写入的条数；win 为空返回 -1。 */
int ps_board_regseq_apply_writes(const ps_board_regseq_t *seq, ps_fix_window_t *win);

/* 统计某类步骤的条数（用于「应有 vs 实际」核对）。 */
int ps_board_regseq_count_kind(const ps_board_regseq_t *seq, uint32_t kind);

/* 把清单打印成一张表（每行：序号 类型 核 段 偏移 地址 值 名称）。
 * 写入 out（含结尾 NUL），返回写入长度；out 为空或 cap==0 返回 -1；
 * 缓冲区不够时截断到能放下并返回写入长度（调用方按 max_len 判是否被截断）。 */
int ps_board_regseq_print(const ps_board_regseq_t *seq, char *out, size_t cap);

/* 返回某核 s_axi_control 段的绝对基址；core 非法时返回 0。 */
uint64_t ps_board_core_ctrl_base(int core);
/* 返回某核 s_axi_control_r 段的绝对基址；core 非法时返回 0。 */
uint64_t ps_board_core_ctrl_r_base(int core);

#ifdef __cplusplus
}
#endif

#endif /* PENUMBRASIGHT_BOARD_REGSEQ_H */
