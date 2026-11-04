/* ============================================================================
 * PenumbraSight · 板端顶层应用核心
 * src/software/board_app/board_top.h
 *
 * 这一层补的是「SD 卡读文件 -> 写 DDR -> 起 PL 预处理链（经 DMA）-> 四核 ->
 * 结果回 DDR -> 帧缓冲 -> DP 显示」这条链里**顶层编排**的缺口：既有
 * fix_core_driver 只有核编排，没有把整条链串起来的东西。
 *
 * 运行环境：PetaLinux 用户态（老师那份资料的三套教程即 基础/驱动/应用）。
 * 理由见 docs/board_top_app.md §1：板载 mini-DP 接在 PS 侧、显示走 PS 的 DRM
 * 驱动（linux-xlnx 的 xlnx_dp），SD 卡也由 Linux 的 mmcblk 暴露为普通文件；
 * 这两件事在裸机上都要自己写驱动，收益不成比例。故本层按用户态写。
 *
 * 本层可分成两半
 * ---------------------------------------------------------------------------
 *   · 可移植核心（本文件 + board_top.c）：DDR 缓冲账本、缓冲布局计算、
 *     接口版本校验、寄存器清单回放、最小用例（写基址 -> 写标量 -> ap_start ->
 *     ap_done -> 读回）、.psf 装载、四核串联。**不依赖任何 OS 头文件**，
 *     因此主机侧能逐条测（见 src/software/test_board_top.c）。
 *   · 板端后端（board_main.c）：open/mmap DDR 与 UIO、DP 帧缓冲落地。
 *     这一半**未在板上跑过**，见文件尾「未验证声明」。
 *
 * DDR 缓冲区为什么要一个显式账本
 * ---------------------------------------------------------------------------
 * 设计里最大的一块 u_codes 约 34 MB，加模型、meas、暂存、帧缓冲，一帧不到
 * 100 MB，但**每一块都必须 4096 对齐**（m_axi 突发跨 4 KB 边界会被拆包，
 * §5.3 的带宽假设随之作废）。所以这里不散落 malloc，改成一个 bump 账本：
 * 一次算清、一次分配、每个地址都从同一条对齐规则来，主机侧可判。
 *
 * 未验证声明
 * ---------------------------------------------------------------------------
 *   · 主机侧已跑通的部分（见 test_board_top.c）：缓冲账本与布局、寄存器清单
 *     生成与「与驱动写入逐条一致」的交叉比对、接口版本读回的三种结果、最小
 *     用例（影子窗口）、四核串联、.psf 装载、帧缓冲格式转换。
 *   · **未在板上跑过**：真实 AXI 读写、ap_done 到达时间、DMA 搬运、SD 驱动、
 *     DP 扫描输出、版本寄存器读回（该寄存器当前不存在）。这些都只在板上成立。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_BOARD_TOP_H
#define PENUMBRASIGHT_BOARD_TOP_H

#include <stddef.h>
#include <stdint.h>

#include "fix_core_driver.h"   /* 作业结构 / 窗口 / 四核 / 段基址 */
#include "board_regseq.h"      /* 寄存器清单 / PS_BOARD_REGSEQ_IFVER_ADDR */
#include "sd_frame.h"          /* .psf 读取层 */
#include "ps_cache.h"          /* PL 访问 DDR 的 cache 维护（成对守卫） */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 返回码 */

#define PS_BOARD_OK                 0
#define PS_BOARD_ERR_ARGS        (-1)
#define PS_BOARD_ERR_ALIGN       (-2)   /* DDR 基址未 4096 对齐 */
#define PS_BOARD_ERR_NOSPACE     (-3)   /* 账本装不下 */
#define PS_BOARD_ERR_IO          (-4)
#define PS_BOARD_ERR_PSF         (-5)   /* .psf 读或校验失败 */
#define PS_BOARD_ERR_TIMEOUT     (-6)   /* 等 ap_done 超时 */
#define PS_BOARD_IFVER_UNMAPPED  (-7)   /* 版本寄存器不在映射窗口内 */
#define PS_BOARD_IFVER_MISMATCH  (-8)   /* 版本寄存器读回值与期望不符 */
#define PS_BOARD_ERR_CACHE       (-9)   /* cache 维护守卫失败（配对/对齐/后端缺失） */

/* 期望的接口版本：与 board/ps_fix_regs.h 的 PS_FIX_IF_VERSION 比对。
 * 板上读回校验在补齐只读版本寄存器之前只能退化成这个编译期比对。 */
#ifndef PS_BOARD_EXPECT_IF_VERSION
#define PS_BOARD_EXPECT_IF_VERSION 1
#endif

/* ---------------------------------------------------------------- 对齐 */

#define PS_BOARD_ALIGN 4096u

uint64_t ps_board_align_up(uint64_t v, uint64_t a);

/* ---------------------------------------------------------------- DDR 账本 */

/* base 是**物理**基址（4096 对齐）；host 是 base 在进程里可直接读写的映射首地址
 * （主机影子：一块静态内存；板上：mmap 出来的虚拟地址）。host 可为 NULL，
 * 此时只记账、不提供主机指针（纯地址规划）。 */
typedef struct {
    uint64_t base;   /* 物理基址 */
    uint64_t size;   /* 总字节数 */
    uint64_t used;   /* 已分配（对齐后累计） */
    void    *host;   /* base 对应的主机/映射首地址，或 NULL */
} ps_board_arena_t;

/* 初始化：base 必须 4096 对齐，size > 0。host 可为 NULL。 */
int ps_board_arena_init(ps_board_arena_t *a, uint64_t base, uint64_t size, void *host);

/* 分配 bytes（内部向上取整到 PS_BOARD_ALIGN），写出 4096 对齐的物理地址。
 * 装不下返回 PS_BOARD_ERR_NOSPACE（账本不变）。 */
int ps_board_arena_alloc(ps_board_arena_t *a, uint64_t bytes, uint64_t *out_phys);

/* 已用/总量。 */
uint64_t ps_board_arena_used(const ps_board_arena_t *a);

/* 由物理地址取主机可读写指针；不在 [base, base+used) 内或 host==NULL 时返回 NULL。 */
void *ps_board_arena_ptr(const ps_board_arena_t *a, uint64_t phys);

/* ---------------------------------------------------------------- 缓冲布局 */

/* 一帧链路的几何与规模（B 档默认值见 docs/board_top_app.md §3）。 */
typedef struct {
    uint32_t meas_h, meas_w;       /* 差分帧尺寸（vertical_diff 输出） */
    uint32_t hidden_h, hidden_w;   /* 隐藏图像尺寸（z） */
    uint32_t channels;             /* 恒为 3 */
    uint32_t candidates;           /* 候选数（默认 243） */
    uint32_t loc_rank;             /* 定位秩 K（默认 8） */
    uint32_t rank_max;             /* 模型最大秩 R（默认 64） */
    uint32_t frame_side;           /* .psf 帧边长（像素，默认 1920） */
} ps_board_layout_cfg_t;

/* 各缓冲的**对齐后**字节数，以及总量。 */
typedef struct {
    uint64_t raw_frame;    /* .psf payload：frame_side^2 个 16 bit 码字 */
    uint64_t rtl_beats;    /* RTL 输出拍：meas_h*meas_w*channels 个 32 bit 字 */
    uint64_t meas;         /* 核的 meas：channels*meas_h*meas_w 个 int32 */
    uint64_t u_codes;      /* U 码字：candidates*loc_rank*meas_len 个 int16 */
    uint64_t gain_q16;     /* candidates 个 int32 */
    uint64_t score_out;    /* candidates 个 uint64 */
    uint64_t score_ch;     /* candidates*channels 个 uint64 */
    uint64_t model;        /* 模型体：candidates*record_bytes + 64 B 头 */
    uint64_t fb;           /* 帧缓冲：frame_side^2 个 uint32（XRGB8888 近似方形） */
    uint64_t total;        /* 上述各项对齐后之和 */
} ps_board_layout_t;

int ps_board_layout_compute(const ps_board_layout_cfg_t *cfg, ps_board_layout_t *out);

/* 用 B 档论文几何（1920 帧 / 95x96 差分 / 18x24 隐藏 / 243 候选 / K=8 / R=64）
 * 填一份默认配置。 */
void ps_board_layout_default_b(ps_board_layout_cfg_t *cfg);

/* ---------------------------------------------------------------- 接口版本 */

/* 编译期比对：PS_FIX_IF_VERSION（board/ps_fix_regs.h）== PS_BOARD_EXPECT_IF_VERSION
 * 时返回 1，否则 0。 */
int ps_board_if_version_compile_check(void);

/* 读 PL 控制页（ps_regs）的 VERSION 寄存器（0xA00A0004）。
 * 返回 PS_BOARD_OK 并写出 *out；窗口覆盖不到该地址返回 PS_BOARD_IFVER_UNMAPPED
 * （2026-09-30 已生成的位流即如此 —— 该控制页是之后才加进块设计脚本的）；
 * 参数为空返回 PS_BOARD_ERR_ARGS。 */
int ps_board_if_version_read(const ps_fix_window_t *w, uint32_t *out);

/* 读 PL 控制页的 ID 寄存器（0xA00A0000，应为 0x5053_0001）：链路存在性探测。
 * 与 VERSION 同样的三态：OK / UNMAPPED / ARGS。 */
int ps_board_pl_id_read(const ps_fix_window_t *w, uint32_t *out);

/* 完整校验：先编译期比对，再尝试读回。
 *   编译期不符          -> PS_BOARD_IFVER_MISMATCH（reason 说明是编译期不符）
 *   读回不可达          -> PS_BOARD_IFVER_UNMAPPED（reason 说明寄存器未实现）
 *   读回值 != expected  -> PS_BOARD_IFVER_MISMATCH
 *   一切相符            -> PS_BOARD_OK
 * reason 允许为 NULL/0。 */
int ps_board_if_version_verify(const ps_fix_window_t *w, uint32_t expected,
                               char *reason, size_t reason_cap);

/* ---------------------------------------------------------------- 寄存器访问 */

int      ps_board_window_covers(const ps_fix_window_t *w, uint64_t addr);
uint32_t ps_board_read_reg(const ps_fix_window_t *w, uint64_t addr);
int      ps_board_write_reg(ps_fix_window_t *w, uint64_t addr, uint32_t v);
uint32_t ps_board_read_ctrl(const ps_fix_window_t *w, int core);

/* ---------------------------------------------------------------- 最小用例 */

/* 一个核的「写基址 -> 写标量 -> ap_start -> ap_done -> 读回」逐条结果。
 * 全部字段为 1/0，便于打印与断言。 */
typedef struct {
    int      core;
    int      writes_applied;      /* 实际写入的寄存器条数 */
    int      base_readback_ok;    /* 写回的 5 个 meas/U/gain/score 基址 LO/HI 读回一致 */
    int      scalar_readback_ok;  /* P/K/M/nx/ny/nz/margin 读回一致 */
    int      done_seen;           /* 等到（或影子替身置上）ap_done */
    uint32_t ctrl_after;          /* 收尾时 CTRL 的值 */
} ps_board_min_report_t;

/* 最小用例：用 DDR 里一小段合成缓冲跑通**一个核**（投影核）的整条寄存器链路。
 * shadow=1 时没有 PL 去置 ap_done，收尾用「影子替身」直接把 done 置上（与
 * fix_core_driver 的影子路径同一约定）——这只覆盖寄存器协议，不覆盖时序。
 * shadow=0 时按 poll_limit 轮询真实 CTRL.ap_done，超时返回 PS_BOARD_ERR_TIMEOUT。 */
int ps_board_min_case(ps_fix_window_t *w, const ps_fix_proj_job_t *job,
                      int shadow, uint32_t poll_limit, ps_board_min_report_t *rep);

/* ---------------------------------------------------------------- .psf 装载 */

/* 从 SD 卡上的 .psf 读一帧到 DDR：内部向 arena 申请 payload 容量，读出并校验
 * （crc32/sum/sumsq/min/max 与头比对），把码字写进缓冲。
 * hdr/check/codes_phys 允许为 NULL。失败写 reason（可为 NULL）。
 * 返回 PS_BOARD_OK；文件或校验问题返回 PS_BOARD_ERR_PSF；装不下返回 NOSPACE。 */
int ps_board_load_psf_file(const char *path, ps_board_arena_t *arena,
                           ps_sd_frame_header_t *hdr,
                           uint64_t *codes_phys, uint16_t **codes_ptr,
                           ps_sd_frame_check_t *check,
                           char *reason, size_t reason_cap);

/* 合成一段 meas 缓冲（无相机、无 RTL 时跑最小用例用）。dst 是主机可写指针
 * （由 ps_board_arena_ptr 得到），words 个 int32。内容确定、非全零，便于读回比对。 */
void ps_board_fill_synth_meas(int32_t *dst, uint32_t words, int32_t seed);

/* ---------------------------------------------------------------- 四核 */

/* 四核串联：先生成清单（供核对/打印），再按 proj -> fwd -> adj -> pdhg 执行。
 * regseq 允许为 NULL（不生成清单）；regseq_writes 允许为 NULL。 */
int ps_board_four_core_case(ps_fix_pipeline_t *p, const ps_fix_sequence_t *seq,
                            uint32_t poll_limit, ps_fix_sequence_result_t *out,
                            ps_board_regseq_t *regseq, int *regseq_writes,
                            uint32_t if_version);

/* ---------------------------------------------------------------- 四核 + cache 维护 */

/* 一次 PL 作业前后的 cache 维护跨度（见 ps_cache.h）。
 *   input_virt/input_bytes   PS 写好、PL 要读的区间：提交前 flush
 *   output_virt/output_bytes PL 写好、PS 要读的区间：读回前 invalidate
 * coherent = PS_CACHE_COHERENT（非缓存映射，如板端 udmabuf）时两次都不产生动作，
 * 但仍强制「先 input 后 output」的成对关系。ops 可为 NULL（相干时）。
 * 板端 Linux 用户态**不能**对可缓存用户页做 dc cvac，故非相干时必须给出 ops，
 * 否则 ps_cache_guard_input 返回 NO_OPS —— 那条路本层拒绝提交。 */
typedef struct {
    const ps_cache_ops_t *ops;
    int      coherent;
    void    *input_virt;   size_t input_bytes;
    void    *output_virt;  size_t output_bytes;
} ps_board_cache_span_t;

/* 带 cache 维护的四核串联。cache 为 NULL 时等价于 ps_board_four_core_case。
 * 维护失败返回 PS_BOARD_ERR_CACHE（且在启动任何核之前拒绝，见实现）。 */
int ps_board_four_core_case_cached(ps_fix_pipeline_t *p, const ps_fix_sequence_t *seq,
                                   uint32_t poll_limit, ps_fix_sequence_result_t *out,
                                   ps_board_regseq_t *regseq, int *regseq_writes,
                                   uint32_t if_version,
                                   const ps_board_cache_span_t *cache);

#ifdef __cplusplus
}
#endif

#endif /* PENUMBRASIGHT_BOARD_TOP_H */
