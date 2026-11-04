/* ============================================================================
 * PenumbraSight · 板端顶层应用核心实现
 * src/software/board_app/board_top.c
 *
 * 见 board_top.h 的分工与「未验证声明」。本文件是**可移植核心**，不包含任何
 * OS 头文件（<fcntl.h>/<sys/mman.h> 都在 board_main.c）；因此同一份代码在主机
 * （MinGW gcc）与板端（aarch64-linux-gnu-gcc）上编出同样的语义。
 * ==========================================================================*/

#include "board_top.h"

#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------- 对齐 */

uint64_t ps_board_align_up(uint64_t v, uint64_t a)
{
    if (a == 0ull) { return v; }
    if (v > UINT64_MAX - (a - 1ull)) { return 0ull; }
    return ((v + a - 1ull) / a) * a;
}

/* ---------------------------------------------------------------- DDR 账本 */

int ps_board_arena_init(ps_board_arena_t *a, uint64_t base, uint64_t size, void *host)
{
    if (!a || size == 0ull) { return PS_BOARD_ERR_ARGS; }
    if ((base % (uint64_t)PS_BOARD_ALIGN) != 0ull) { return PS_BOARD_ERR_ALIGN; }
    a->base = base;
    a->size = size;
    a->used = 0ull;
    a->host = host;
    return PS_BOARD_OK;
}

int ps_board_arena_alloc(ps_board_arena_t *a, uint64_t bytes, uint64_t *out_phys)
{
    uint64_t aligned;
    if (!a || !out_phys || bytes == 0ull) { return PS_BOARD_ERR_ARGS; }
    aligned = ps_board_align_up(bytes, (uint64_t)PS_BOARD_ALIGN);
    if (aligned == 0ull || a->used > a->size || aligned > a->size - a->used ||
        a->base > UINT64_MAX - a->used || a->base + a->used > UINT64_MAX - aligned) { return PS_BOARD_ERR_NOSPACE; }
    *out_phys = a->base + a->used;
    a->used += aligned;
    return PS_BOARD_OK;
}

uint64_t ps_board_arena_used(const ps_board_arena_t *a)
{
    return a ? a->used : 0ull;
}

void *ps_board_arena_ptr(const ps_board_arena_t *a, uint64_t phys)
{
    if (!a || !a->host) { return NULL; }
    if (phys < a->base || phys - a->base >= a->used) { return NULL; }
    return (void *)((uint8_t *)a->host + (phys - a->base));
}

/* ---------------------------------------------------------------- 缓冲布局 */

void ps_board_layout_default_b(ps_board_layout_cfg_t *cfg)
{
    if (!cfg) { return; }
    cfg->meas_h   = PS_PSM_MEAS_H;    /* 95 */
    cfg->meas_w   = PS_PSM_MEAS_W;    /* 96 */
    cfg->hidden_h = PS_PSM_HIDDEN_H;  /* 18 */
    cfg->hidden_w = PS_PSM_HIDDEN_W;  /* 24 */
    cfg->channels = 3u;
    cfg->candidates = PS_PSM_CANDIDATES; /* 243 */
    cfg->loc_rank   = PS_PSM_LOC_RANK;   /* 8 */
    cfg->rank_max   = PS_PSM_RANK_MAX;   /* 64 */
    cfg->frame_side = 1920u;             /* 2*BLOCK*WALL_N = 2*10*96 */
}

int ps_board_layout_compute(const ps_board_layout_cfg_t *cfg, ps_board_layout_t *out)
{
    uint64_t meas_len, hidden_len, record_bytes, model_raw, fb_raw;
    uint64_t sizes[9];
    int i;

    if (!cfg || !out) { return PS_BOARD_ERR_ARGS; }
    if (cfg->meas_h == 0u || cfg->meas_w == 0u || cfg->hidden_h == 0u ||
        cfg->hidden_w == 0u || cfg->channels == 0u || cfg->candidates == 0u ||
        cfg->loc_rank == 0u || cfg->rank_max == 0u || cfg->frame_side == 0u) {
        return PS_BOARD_ERR_ARGS;
    }

    meas_len   = (uint64_t)cfg->meas_h * (uint64_t)cfg->meas_w;
    hidden_len = (uint64_t)cfg->hidden_h * (uint64_t)cfg->hidden_w;
    /* 单条候选记录字节数走 psm_format.h 的派生函数，不写死（含内部对齐）。 */
    record_bytes = ps_psm_record_bytes((uint16_t)cfg->rank_max, meas_len, hidden_len);
    model_raw = (uint64_t)PS_PSM_HEADER_BYTES +
                (uint64_t)cfg->candidates * record_bytes;
    fb_raw = (uint64_t)cfg->frame_side * (uint64_t)cfg->frame_side * 4ull;

    out->raw_frame = ps_board_align_up((uint64_t)cfg->frame_side * cfg->frame_side * 2ull,
                                       PS_BOARD_ALIGN);
    out->rtl_beats = ps_board_align_up(meas_len * cfg->channels * 4ull, PS_BOARD_ALIGN);
    out->meas      = ps_board_align_up(meas_len * cfg->channels * 4ull, PS_BOARD_ALIGN);
    out->u_codes   = ps_board_align_up((uint64_t)cfg->candidates *
                                       (uint64_t)cfg->loc_rank * meas_len * 2ull,
                                       PS_BOARD_ALIGN);
    out->gain_q16  = ps_board_align_up((uint64_t)cfg->candidates * 4ull, PS_BOARD_ALIGN);
    out->score_out = ps_board_align_up((uint64_t)cfg->candidates * 8ull, PS_BOARD_ALIGN);
    out->score_ch  = ps_board_align_up((uint64_t)cfg->candidates *
                                       cfg->channels * 8ull, PS_BOARD_ALIGN);
    out->model     = ps_board_align_up(model_raw, PS_BOARD_ALIGN);
    out->fb        = ps_board_align_up(fb_raw, PS_BOARD_ALIGN);

    sizes[0] = out->raw_frame; sizes[1] = out->rtl_beats; sizes[2] = out->meas;
    sizes[3] = out->u_codes;   sizes[4] = out->gain_q16;  sizes[5] = out->score_out;
    sizes[6] = out->score_ch;  sizes[7] = out->model;     sizes[8] = out->fb;
    out->total = 0ull;
    for (i = 0; i < 9; ++i) { out->total += sizes[i]; }
    return PS_BOARD_OK;
}

/* ---------------------------------------------------------------- 接口版本 */

int ps_board_if_version_compile_check(void)
{
    /* PS_FIX_IF_VERSION 来自 board/ps_fix_regs.h（经 fix_core_driver.h -> fix_core.h）。
     * 与 HLS 顶层 ps_fixed_top.hpp 的同名宏同源，块设计脚本会在综合前强校验它们
     * 一致；这里是 PS 侧软件再自证一次。 */
#if (PS_FIX_IF_VERSION == PS_BOARD_EXPECT_IF_VERSION)
    return 1;
#else
    return 0;
#endif
}

int ps_board_if_version_read(const ps_fix_window_t *w, uint32_t *out)
{
    if (!w || !w->window || !out) { return PS_BOARD_ERR_ARGS; }
    if (!ps_board_window_covers(w, (uint64_t)PS_BOARD_REGSEQ_IFVER_ADDR)) {
        return PS_BOARD_IFVER_UNMAPPED;
    }
    *out = ps_board_read_reg(w, (uint64_t)PS_BOARD_REGSEQ_IFVER_ADDR);
    return PS_BOARD_OK;
}

int ps_board_pl_id_read(const ps_fix_window_t *w, uint32_t *out)
{
    if (!w || !w->window || !out) { return PS_BOARD_ERR_ARGS; }
    if (!ps_board_window_covers(w, (uint64_t)PS_BOARD_REGSEQ_PL_ID_ADDR)) {
        return PS_BOARD_IFVER_UNMAPPED;
    }
    *out = ps_board_read_reg(w, (uint64_t)PS_BOARD_REGSEQ_PL_ID_ADDR);
    return PS_BOARD_OK;
}

int ps_board_if_version_verify(const ps_fix_window_t *w, uint32_t expected,
                               char *reason, size_t reason_cap)
{
    uint32_t got = 0u;
    int rc;

    if (reason && reason_cap > 0u) { reason[0] = '\0'; }

    if (!ps_board_if_version_compile_check()) {
        if (reason && reason_cap > 0u) {
            (void)snprintf(reason, reason_cap,
                           "编译期不符：PS_FIX_IF_VERSION=%d，期望 %d",
                           (int)PS_FIX_IF_VERSION, (int)PS_BOARD_EXPECT_IF_VERSION);
        }
        return PS_BOARD_IFVER_MISMATCH;
    }

    rc = ps_board_if_version_read(w, &got);
    if (rc == PS_BOARD_IFVER_UNMAPPED || rc == PS_BOARD_ERR_ARGS) {
        if (reason && reason_cap > 0u) {
            (void)snprintf(reason, reason_cap,
                           "读回不可达：PL 控制页 VERSION 寄存器（0x%08X）不在映射窗口内；"
                           "2026-09-30 的位流早于该控制页，需重新出位流并映射该段。"
                           "当前只能退化为编译期比对（通过）",
                           (unsigned)PS_BOARD_REGSEQ_IFVER_ADDR);
        }
        return PS_BOARD_IFVER_UNMAPPED;
    }
    if (got != expected) {
        if (reason && reason_cap > 0u) {
            (void)snprintf(reason, reason_cap,
                           "读回值 0x%08X != 期望 0x%08X（位流与驱动可能不是同一次构建）",
                           got, expected);
        }
        return PS_BOARD_IFVER_MISMATCH;
    }
    return PS_BOARD_OK;
}

/* ---------------------------------------------------------------- 寄存器访问 */

int ps_board_window_covers(const ps_fix_window_t *w, uint64_t addr)
{
    if (!w || !w->window) { return 0; }
    if (addr < (uint64_t)w->window_phys) { return 0; }
    return (addr - (uint64_t)w->window_phys) + 4ull <= (uint64_t)w->window_bytes;
}

uint32_t ps_board_read_reg(const ps_fix_window_t *w, uint64_t addr)
{
    if (!ps_board_window_covers(w, addr)) { return 0u; }
    return w->window[(uint32_t)((addr - (uint64_t)w->window_phys) >> 2)];
}

int ps_board_write_reg(ps_fix_window_t *w, uint64_t addr, uint32_t v)
{
    if (!ps_board_window_covers(w, addr)) { return PS_BOARD_ERR_ARGS; }
    w->window[(uint32_t)((addr - (uint64_t)w->window_phys) >> 2)] = v;
    return PS_BOARD_OK;
}

uint32_t ps_board_read_ctrl(const ps_fix_window_t *w, int core)
{
    uint64_t base = ps_board_core_ctrl_base(core);
    if (base == 0ull) { return 0u; }
    return ps_board_read_reg(w, base + PS_FIX_REG_CTRL);
}

/* ---------------------------------------------------------------- 最小用例 */

int ps_board_min_case(ps_fix_window_t *w, const ps_fix_proj_job_t *job,
                      int shadow, uint32_t poll_limit, ps_board_min_report_t *rep)
{
    ps_board_regseq_t seq;
    ps_board_regseq_cfg_t cfg;
    const uint32_t cb = (uint32_t)PS_FIX_CORE_PROJ_CTRL;
    const uint32_t cr = (uint32_t)PS_FIX_CORE_PROJ_CTRL_R;
    int wrote, expected_writes, i;

    if (!w || !job || !rep) { return PS_BOARD_ERR_ARGS; }
    memset(rep, 0, sizeof(*rep));
    rep->core = PS_FIX_CORE_ID_PROJ;

    /* 1) 生成清单并回放到窗口。清单与驱动同源（都从 ps_fix_regs.h 的宏来），
     *    主机侧另有一条交叉比对证明这一点（test_board_top.c）。 */
    memset(&cfg, 0, sizeof(cfg));
    cfg.proj = job;
    cfg.include_version = 0;
    if (ps_board_regseq_build(&seq, &cfg) != 0) { return PS_BOARD_ERR_ARGS; }
    wrote = ps_board_regseq_apply_writes(&seq, w);
    expected_writes = ps_board_regseq_count_kind(&seq, PS_REGSEQ_WRITE);
    rep->writes_applied = wrote;
    if (wrote < 0 || wrote != expected_writes) { return PS_BOARD_ERR_ARGS; }

    /* 2) 写基址 -> 读回：5 个 64 位基址的 LO/HI 两半字都要一致。 */
    {
        struct { uint32_t off; uint64_t val; } bases[5];
        int ok = 1;
        bases[0].off = PROJ_BASE_MEAS;     bases[0].val = job->meas;
        bases[1].off = PROJ_BASE_U_CODES;  bases[1].val = job->u_codes;
        bases[2].off = PROJ_BASE_GAIN_Q16; bases[2].val = job->gain_q16;
        bases[3].off = PROJ_BASE_SCORE_OUT; bases[3].val = job->score_out;
        bases[4].off = PROJ_BASE_SCORE_CH;  bases[4].val = job->score_ch;
        for (i = 0; i < 5 && ok; ++i) {
            uint32_t lo = ps_board_read_reg(w, (uint64_t)cr + bases[i].off + PS_FIX_64_LO);
            uint32_t hi = ps_board_read_reg(w, (uint64_t)cr + bases[i].off + PS_FIX_64_HI);
            if (lo != (uint32_t)(bases[i].val & 0xFFFFFFFFu)) { ok = 0; }
            if (hi != (uint32_t)(bases[i].val >> 32)) { ok = 0; }
        }
        rep->base_readback_ok = ok;
    }

    /* 3) 写标量 -> 读回。 */
    {
        int ok = 1;
        if (ps_board_read_reg(w, (uint64_t)cb + PROJ_P) != job->P) { ok = 0; }
        if (ps_board_read_reg(w, (uint64_t)cb + PROJ_K) != job->K) { ok = 0; }
        if (ps_board_read_reg(w, (uint64_t)cb + PROJ_M) != job->M) { ok = 0; }
        if (ps_board_read_reg(w, (uint64_t)cb + PROJ_NX) != job->nx) { ok = 0; }
        if (ps_board_read_reg(w, (uint64_t)cb + PROJ_NY) != job->ny) { ok = 0; }
        if (ps_board_read_reg(w, (uint64_t)cb + PROJ_NZ) != job->nz) { ok = 0; }
        if (ps_board_read_reg(w, (uint64_t)cb + PROJ_MARGIN_THRESH_Q16) !=
            job->margin_thresh_q16) { ok = 0; }
        rep->scalar_readback_ok = ok;
    }

    /* 4) ap_start -> ap_done。 */
    (void)ps_board_write_reg(w, (uint64_t)cb + PS_FIX_REG_CTRL, PS_FIX_CTRL_AP_START);
    if (shadow) {
        /* 影子窗口没有 PL 置 ap_done，用替身置上（与 fix_core_driver 同约定）。 */
        (void)ps_board_write_reg(w, (uint64_t)cb + PS_FIX_REG_CTRL,
                                 PS_FIX_CTRL_AP_START | PS_FIX_CTRL_AP_DONE);
        rep->done_seen = 1;
    } else {
        uint32_t n = poll_limit;
        rep->done_seen = 0;
        while (n-- > 0u) {
            if ((ps_board_read_ctrl(w, PS_FIX_CORE_ID_PROJ) & PS_FIX_CTRL_AP_DONE) != 0u) {
                rep->done_seen = 1;
                break;
            }
        }
        if (!rep->done_seen) {
            rep->ctrl_after = ps_board_read_ctrl(w, PS_FIX_CORE_ID_PROJ);
            return PS_BOARD_ERR_TIMEOUT;
        }
    }
    rep->ctrl_after = ps_board_read_ctrl(w, PS_FIX_CORE_ID_PROJ);
    return PS_BOARD_OK;
}

/* ---------------------------------------------------------------- .psf 装载 */

int ps_board_load_psf_file(const char *path, ps_board_arena_t *arena,
                           ps_sd_frame_header_t *hdr,
                           uint64_t *codes_phys, uint16_t **codes_ptr,
                           ps_sd_frame_check_t *check,
                           char *reason, size_t reason_cap)
{
    ps_sd_frame_header_t local;
    uint64_t words, bytes, phys;
    uint16_t *dst;
    int rc;

    if (!path || !arena) { return PS_BOARD_ERR_ARGS; }
    if (hdr) { memset(hdr, 0, sizeof(*hdr)); }
    if (reason && reason_cap > 0u) { reason[0] = '\0'; }

    /* 第一遍：只解析头 + 校验 payload，拿尺寸（dst=NULL，不搬运）。 */
    rc = ps_sd_frame_read_file(path, &local, NULL, 0u, check, reason, reason_cap);
    if (rc != PS_SD_OK) { return PS_BOARD_ERR_PSF; }

    words = (uint64_t)local.width * (uint64_t)local.height;
    bytes = words * 2ull;
    rc = ps_board_arena_alloc(arena, bytes, &phys);
    if (rc != PS_BOARD_OK) { return rc; }

    dst = (uint16_t *)ps_board_arena_ptr(arena, phys);
    if (!dst) {
        /* 账本未绑定主机映射：只能给出物理地址，不做搬运。 */
        if (hdr) { *hdr = local; }
        if (codes_phys) { *codes_phys = phys; }
        if (codes_ptr)  { *codes_ptr = NULL; }
        return PS_BOARD_OK;
    }

    /* 第二遍：读进缓冲（同时再校验一次，两次口径一致）。 */
    rc = ps_sd_frame_read_file(path, &local, dst, (size_t)words, check, reason, reason_cap);
    if (rc != PS_SD_OK) { return PS_BOARD_ERR_PSF; }

    if (hdr) { *hdr = local; }
    if (codes_phys) { *codes_phys = phys; }
    if (codes_ptr)  { *codes_ptr = dst; }
    return PS_BOARD_OK;
}

void ps_board_fill_synth_meas(int32_t *dst, uint32_t words, int32_t seed)
{
    uint32_t i;
    if (!dst) { return; }
    for (i = 0u; i < words; ++i) {
        /* 确定、非全零、含正负，便于「读回 == 写入」这类比对不因偶然相等而通过。 */
        dst[i] = (int32_t)(((uint32_t)seed + i * 2654435761u) & 0x0003FFFFu) - 0x00020000;
    }
}

/* ---------------------------------------------------------------- 四核 */

int ps_board_four_core_case(ps_fix_pipeline_t *p, const ps_fix_sequence_t *seq,
                            uint32_t poll_limit, ps_fix_sequence_result_t *out,
                            ps_board_regseq_t *regseq, int *regseq_writes,
                            uint32_t if_version)
{
    return ps_board_four_core_case_cached(p, seq, poll_limit, out, regseq,
                                          regseq_writes, if_version, NULL);
}

int ps_board_four_core_case_cached(ps_fix_pipeline_t *p, const ps_fix_sequence_t *seq,
                                   uint32_t poll_limit, ps_fix_sequence_result_t *out,
                                   ps_board_regseq_t *regseq, int *regseq_writes,
                                   uint32_t if_version,
                                   const ps_board_cache_span_t *cache)
{
    ps_board_regseq_cfg_t cfg;
    ps_cache_guard_t guard;
    int wrote = -1;
    int rc;

    if (!p || !p->opened || !seq || !out) { return PS_BOARD_ERR_ARGS; }
    memset(&guard, 0, sizeof(guard));

    /* 1) 提交 PL 之前：把输入侧推下 cache（相干映射下是空操作）。
     *    失败**在启动任何核之前**返回 —— 半维护状态下起核比直接失败更危险。 */
    if (cache) {
        const int crc = ps_cache_guard_input(&guard, cache->ops, cache->coherent,
                                             cache->input_virt, cache->input_bytes);
        if (crc != PS_CACHE_OK) { return PS_BOARD_ERR_CACHE; }
    }

    /* 2) 寄存器清单：生成（供核对/打印）并回放到窗口。 */
    if (regseq) {
        memset(&cfg, 0, sizeof(cfg));
        cfg.proj = seq->proj;
        cfg.fwd  = seq->fwd;
        cfg.adj  = seq->adj;
        cfg.pdhg = seq->pdhg;
        cfg.include_version = 1;
        cfg.if_version = if_version;
        if (ps_board_regseq_build(regseq, &cfg) != 0) { return PS_BOARD_ERR_ARGS; }
        wrote = ps_board_regseq_apply_writes(regseq, p->win);
    }
    if (regseq_writes) { *regseq_writes = wrote; }

    /* 3) 四核串联（proj -> fwd -> adj -> pdhg）。 */
    rc = ps_fix_run_sequence(p, seq, poll_limit, out);

    /* 4) 读回结果之前：作废输出侧 cache。即使作业失败也要收尾（结果可能只写了
     *    一部分），但收尾失败不覆盖作业本身的错误码。 */
    if (cache) {
        const int crc = ps_cache_guard_output(&guard, cache->ops, cache->coherent,
                                              cache->output_virt, cache->output_bytes);
        if (crc != PS_CACHE_OK && rc == PS_FIX_OK) { rc = PS_BOARD_ERR_CACHE; }
    }

    return rc;
}
