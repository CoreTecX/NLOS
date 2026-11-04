/* ============================================================================
 * PenumbraSight · PL 访问 DDR 缓冲的 cache 一致性
 * src/software/include/ps_cache.h
 *
 * 为什么需要这一层
 * ---------------------------------------------------------------------------
 * PL（经 HP 端口做 m_axi 读写）与 PS（Cortex-A53）是两个 master，共用 DDR。
 * 若缓冲落在**可缓存**的 normal memory 上，PS 写进去的数据可能还停在 D-cache
 * 里没到 DDR，PL 读到的是旧值；PL 写回的结果也可能被 PS 的 D-cache 旧副本
 * 遮住。两种都表现为「数字看着合理但整体错」，不报错。
 *
 * 两条可选路径（本仓库的选择见 ps_cache.c 文首）
 *   (a) 提交 PL 前 flush（clean）、读回结果前 invalidate：裸机唯一可用，靠
 *       Xil_DCacheFlushRange / Xil_DCacheInvalidateRange；
 *   (b) 干脆用**非缓存/相干映射**：Linux 用户态走 udmabuf / dma-heap，由内核
 *       把这段 DDR 映射成不可缓存，于是 PL 与 PS 天然相干，不需要任何维护。
 *
 * 本层把两者收在同一张可注入函数表后面，并用 ps_cache_guard_t 强制
 * 「输入侧 flush（提交前）-> 输出侧 invalidate（读回前）」**成对且有序**，
 * 同时把两个区间都对齐到 cache line。主机上跑不了真 cache 指令，故真后端由
 * 调用方注入；单测注入记录型后端，断言调用序列与对齐区间
 * （见 src/software/test_board_top.c 的 cache 段）。
 *
 * 未验证声明
 * ---------------------------------------------------------------------------
 * 真 cache 指令（Xil_DCacheFlushRange / Xil_DCacheInvalidateRange）与
 * 「udmabuf / dma-heap 的 mmap 到底是不是非缓存」属**板端实测**，本层不代跑。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_PS_CACHE_H
#define PENUMBRASIGHT_PS_CACHE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 返回码 */
#define PS_CACHE_OK            0
#define PS_CACHE_ERR_ARGS    (-1)   /* 空指针 / bytes==0 */
#define PS_CACHE_ERR_STATE   (-2)   /* 顺序/配对错误：重复调用、或输出侧先于输入侧 */
#define PS_CACHE_ERR_ALIGN   (-3)   /* line_bytes==0/非 2 的幂，给不出对齐区间 */
#define PS_CACHE_ERR_NO_OPS  (-4)   /* 非相干映射但没提供维护后端（Linux 用户态补不了） */

/* ---------------------------------------------------------------- 相干性 */
/* PS 与 PL 对同一段 DDR 的可见性。coherent=1 时本层不产生任何 cache 动作。 */
#define PS_CACHE_COHERENT    1   /* 非缓存 / 相干映射（如 udmabuf、dma-heap 的 coherent 映射） */
#define PS_CACHE_NONCOHERENT 0   /* 可缓存 normal memory，需要显式维护 */

/* ---------------------------------------------------------------- 后端表 */

/* 一张表 = 一个 cache 维护后端。
 *   flush      clean：把 CPU 侧写的数据推到 DDR（**提交给 PL 之前**调用）
 *   invalidate invalidate：丢弃 CPU 侧副本，逼下次读回内存（**读 PL 结果之前**调用）
 * 两个函数都必须对 [virt, virt+bytes) 生效；区间已经由本层对齐到 line 边界。 */
typedef struct ps_cache_ops {
    const char *name;
    unsigned    line_bytes;  /* cache line 字节数（维护粒度）；0 = 未知 */
    void (*flush)(void *ctx, void *virt, size_t bytes);
    void (*invalidate)(void *ctx, void *virt, size_t bytes);
} ps_cache_ops_t;

/* 默认后端：两个动作都是空操作，line = 64。
 * 用于「相干/非缓存映射」与「主机影子内存（没有真 PL）」两种场合。 */
const ps_cache_ops_t *ps_cache_ops_none(void);

/* ---------------------------------------------------------------- 区间对齐 */

/* 把 [virt, virt+bytes) 扩到 cache line 边界：[start, start+len)，两端都对齐。
 * line_bytes 必须是 2 的幂。line 非 2 的幂返回 ERR_ALIGN；virt==NULL 或
 * bytes==0 返回 ERR_ARGS。
 * 为什么必须扩：clean/invalidate 按整行操作，只维护用户给的子区间会漏掉与该
 * 区间共享一行的邻居字节，PL 可能读到半新半旧。 */
int ps_cache_align_range(const void *virt, size_t bytes, unsigned line_bytes,
                         uintptr_t *start_out, size_t *len_out);

/* 单次维护（内部先对齐再调后端，ctx 恒为 NULL）。ops 为 NULL 返回 ERR_ARGS。 */
int ps_cache_flush_range(const ps_cache_ops_t *ops, void *virt, size_t bytes);
int ps_cache_invalidate_range(const ps_cache_ops_t *ops, void *virt, size_t bytes);

/* ---------------------------------------------------------------- 成对守卫 */

/* 一次 PL 作业的 cache 序列：
 *     ps_cache_guard_input()  （提交前，flush 输入）
 *         -> 写寄存器 + ap_start + 等 done
 *     ps_cache_guard_output() （读回前，invalidate 输出）
 * coherent=1 时两次都不产生 cache 动作，但仍强制「先 input 后 output」的成对关系。
 * 非相干时 input 调 flush、output 调 invalidate，各自对齐到同一 cache line 粒度。 */
typedef struct {
    const ps_cache_ops_t *ops;
    int       coherent;
    int       input_done;    /* 1 = 输入侧已维护 */
    int       output_done;   /* 1 = 输出侧已维护 */
    uintptr_t input_start;   size_t input_bytes;
    uintptr_t output_start;  size_t output_bytes;
} ps_cache_guard_t;

/* 提交 PL 之前调用（flush 输入侧）。
 *   coherent=1：不动作，返回 OK（ops 可为 NULL）。
 *   coherent=0：必须有 ops->flush/invalidate 且 ops->line_bytes 为 2 的幂；否则
 *     没有 ops        -> PS_CACHE_ERR_NO_OPS（Linux 用户态无法事后补做）
 *     line 非 2 的幂  -> PS_CACHE_ERR_ALIGN
 * 重复 input 返回 PS_CACHE_ERR_STATE。 */
int ps_cache_guard_input(ps_cache_guard_t *g, const ps_cache_ops_t *ops,
                         int coherent, void *virt, size_t bytes);

/* 读回 PL 结果之前调用（invalidate 输出侧）。
 * 未先 input 就 output 返回 PS_CACHE_ERR_STATE（说明漏了提交前的 flush）。
 * 重复 output 同样返回 STATE。调用后守卫结束。 */
int ps_cache_guard_output(ps_cache_guard_t *g, const ps_cache_ops_t *ops,
                          int coherent, void *virt, size_t bytes);

#ifdef __cplusplus
}
#endif

#endif /* PENUMBRASIGHT_PS_CACHE_H */
