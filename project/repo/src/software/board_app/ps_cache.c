/* ============================================================================
 * PenumbraSight · PL 访问 DDR 缓冲的 cache 一致性（可移植核心）
 * src/software/board_app/ps_cache.c
 *
 * 见 ps_cache.h 的接口说明。本文件不含任何 OS / BSP 头文件，因此主机与板端
 * 编出同样的语义；真正的 cache 指令由调用方通过 ps_cache_ops_t 注入。
 *
 * 本仓库的路径选择（A6 的结论）
 * ---------------------------------------------------------------------------
 * 生产目标是 PetaLinux **用户态**（见 docs/board_top_app.md）。用户态没有可移植
 * 的方式对任意用户虚拟地址做 dc cvac/civac（要 privileged 指令或内核驱动），
 * 所以 Linux 侧**只能走非缓存/相干映射**：DDR 缓冲由 ps_pmem_alloc 取
 * udmabuf / dma-heap（内核按 DMA 属性映射，不缓存），此时 guard 的 coherent=1、
 * 不产生 cache 动作；寄存器页本来就以 O_SYNC 映射（Device，不可缓存）。
 *
 * 裸机（src/software/baremetal）没有 MMU 属性可调、缓冲是普通可缓存内存，
 * 只能走显式维护：注入 Xil_DCacheFlushRange / Xil_DCacheInvalidateRange，
 * guard 的 coherent=0。
 *
 * 两条路都收在本文件的对齐 + 成对守卫之下，于是
 * 「flush 在提交前、invalidate 在读回前、两个区间都对齐到 cache line」这条约束
 * 只有一个实现点、一个断言点。
 * ==========================================================================*/

#include "ps_cache.h"

/* ---------------------------------------------------------------- 默认后端 */

static void noop_range(void *ctx, void *virt, size_t bytes) {
    (void)ctx; (void)virt; (void)bytes;
}

static const ps_cache_ops_t kOpsNone = {
    "none", 64u, noop_range, noop_range
};

const ps_cache_ops_t *ps_cache_ops_none(void) {
    return &kOpsNone;
}

/* ---------------------------------------------------------------- 区间对齐 */

static int is_pow2(unsigned v) {
    return (v != 0u) && ((v & (v - 1u)) == 0u);
}

int ps_cache_align_range(const void *virt, size_t bytes, unsigned line_bytes,
                         uintptr_t *start_out, size_t *len_out) {
    uintptr_t v, start;
    size_t len;

    if (!virt || bytes == 0u || !start_out || !len_out) return PS_CACHE_ERR_ARGS;
    if (!is_pow2(line_bytes)) return PS_CACHE_ERR_ALIGN;

    v     = (uintptr_t)virt;
    /* 向下取整到行首、向上取整到行尾。line 是 2 的幂，故用位运算。 */
    start = v & ~((uintptr_t)line_bytes - 1u);
    len   = ((v - start) + bytes + (size_t)line_bytes - 1u) &
            ~((size_t)line_bytes - 1u);
    if (len == 0u) len = (size_t)line_bytes;   /* 理论到不了；防御 */

    *start_out = start;
    *len_out   = len;
    return PS_CACHE_OK;
}

/* ---------------------------------------------------------------- 单次维护 */

int ps_cache_flush_range(const ps_cache_ops_t *ops, void *virt, size_t bytes) {
    uintptr_t start = 0u;
    size_t    len = 0u;
    int rc;
    if (!ops || !ops->flush) return PS_CACHE_ERR_ARGS;
    rc = ps_cache_align_range(virt, bytes, ops->line_bytes, &start, &len);
    if (rc != PS_CACHE_OK) return rc;
    ops->flush(NULL, (void *)start, len);
    return PS_CACHE_OK;
}

int ps_cache_invalidate_range(const ps_cache_ops_t *ops, void *virt, size_t bytes) {
    uintptr_t start = 0u;
    size_t    len = 0u;
    int rc;
    if (!ops || !ops->invalidate) return PS_CACHE_ERR_ARGS;
    rc = ps_cache_align_range(virt, bytes, ops->line_bytes, &start, &len);
    if (rc != PS_CACHE_OK) return rc;
    ops->invalidate(NULL, (void *)start, len);
    return PS_CACHE_OK;
}

/* ---------------------------------------------------------------- 成对守卫 */

/* 非相干时校验后端可用，并算出对齐区间；相干时只记录不做动作。 */
static int guard_prepare(const ps_cache_ops_t *ops, int coherent,
                         void *virt, size_t bytes,
                         uintptr_t *start_out, size_t *len_out, int *coherent_out) {
    if (!virt || bytes == 0u) return PS_CACHE_ERR_ARGS;
    if (coherent == PS_CACHE_COHERENT) {
        /* 非缓存/相干映射：区间不要求对齐（不做任何行级动作），原样记录。 */
        *start_out    = (uintptr_t)virt;
        *len_out      = bytes;
        *coherent_out = PS_CACHE_COHERENT;
        return PS_CACHE_OK;
    }
    if (!ops || !ops->flush || !ops->invalidate) return PS_CACHE_ERR_NO_OPS;
    {
        const int rc = ps_cache_align_range(virt, bytes, ops->line_bytes,
                                            start_out, len_out);
        if (rc != PS_CACHE_OK) return rc;
    }
    *coherent_out = PS_CACHE_NONCOHERENT;
    return PS_CACHE_OK;
}

int ps_cache_guard_input(ps_cache_guard_t *g, const ps_cache_ops_t *ops,
                         int coherent, void *virt, size_t bytes) {
    uintptr_t start = 0u;
    size_t    len = 0u;
    int       coh = PS_CACHE_NONCOHERENT;
    int rc;

    if (!g) return PS_CACHE_ERR_ARGS;
    if (g->input_done || g->output_done) return PS_CACHE_ERR_STATE;

    rc = guard_prepare(ops, coherent, virt, bytes, &start, &len, &coh);
    if (rc != PS_CACHE_OK) return rc;

    g->ops          = ops;
    g->coherent     = coh;
    g->input_start  = start;
    g->input_bytes  = len;
    g->output_start = 0u;
    g->output_bytes = 0u;
    g->output_done  = 0;

    if (coh == PS_CACHE_NONCOHERENT) {
        ops->flush(NULL, (void *)start, len);
    }
    g->input_done = 1;
    return PS_CACHE_OK;
}

int ps_cache_guard_output(ps_cache_guard_t *g, const ps_cache_ops_t *ops,
                          int coherent, void *virt, size_t bytes) {
    uintptr_t start = 0u;
    size_t    len = 0u;
    int       coh = PS_CACHE_NONCOHERENT;
    int rc;

    if (!g) return PS_CACHE_ERR_ARGS;
    /* 没有输入侧的 flush 就先做输出侧的 invalidate = 漏了提交前维护，判 STATE。 */
    if (!g->input_done || g->output_done) return PS_CACHE_ERR_STATE;

    /* 相干性必须与 input 一致；ops 也沿用 input 那次（避免前后用了不同后端）。 */
    (void)coherent;
    rc = guard_prepare(ops ? ops : g->ops, g->coherent, virt, bytes,
                       &start, &len, &coh);
    if (rc != PS_CACHE_OK) return rc;

    g->output_start = start;
    g->output_bytes = len;
    if (coh == PS_CACHE_NONCOHERENT) {
        g->ops->invalidate(NULL, (void *)start, len);
    }
    g->output_done = 1;
    return PS_CACHE_OK;
}
