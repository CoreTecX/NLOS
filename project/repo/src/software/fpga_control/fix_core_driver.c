/* ============================================================================
 * PenumbraSight · 板端 PS 侧「定点四核」调用层实现
 * src/software/fpga_control/fix_core_driver.c
 *
 * 见 fix_core_driver.h 的边界与隔离说明。要点复述：
 *   · 四核 = proj(0xA0) / lr_fwd(0xA2) / lr_adj(0xA4) / pdhg(0xA6)，各带一个
 *     s_axi_control_r（基址 +0x10000）装 m_axi 缓冲基址。
 *   · 本层不自己拆 64 位地址、不自己读 VLD：一行都走 board/fix_core 的原语
 *     （ps_fix_set_base / ps_fix_read_vld / ps_fix_read_vld64）。顺序约束因此只有
 *     一处实现，不存在「两份实现漂移」。
 *   · psm 包长/偏移全部来自 psm_format.h 的 ps_psm_* 派生函数，不写死。
 *
 * 真板路径（UIO）
 * ---------------------------------------------------------------------------
 *   ps_fix_window_open_uio() 已填实为标准 UIO：open("/dev/uioN", O_RDWR|O_SYNC)
 *   + 按 /sys/class/uio/uioN/maps/map0/size 取长度做 mmap(map0)。
 *   逻辑在 uio_backend.{h,c}（可注入、主机可测），系统调用在 uio_backend_posix.c
 *   （只在 Linux 构建里编译）。要接通还需要：
 *     1) 设备树里给 0xA0000000 起至少 512 KiB（八段）留一个 UIO 节点，且
 *        map0 的 addr/size 覆盖它；若要读 0xA00A0000 的 PL 控制页，reg 范围
 *        需开到 0xA00A0010 以上（否则版本读回为 UNMAPPED，board_top 已处理）；
 *     2) 内核打开 CONFIG_UIO 与 UIO 平台驱动（uio_pdrv_genirq 或
 *        uio_dmem_genirq）；节点 compatible 与启动参数 of_id 匹配；
 *     3) 运行用户对 /dev/uioN 有读写权限。
 *   前提与逐条检查命令见 sim/results/uio_entry_20261001.md。
 * ==========================================================================*/

#include "fix_core_driver.h"
#include "uio_backend.h"

#include <stdio.h>
#include <string.h>
#include <float.h>

/* ---------------------------------------------------------------- 窗口 */

int ps_fix_window_open_shadow(ps_fix_window_t *w, void *storage, uint32_t bytes)
{
    if (!w || !storage || bytes == 0u) { return PS_FIX_DRV_ERR_ARGS; }
    /* 影子窗口必须覆盖到最后一个核的 ctrl_r 段末。八段 = 512 KiB。 */
    if (bytes < 8u * 0x10000u) { return PS_FIX_DRV_ERR_ARGS; }

    memset(w, 0, sizeof(*w));
    memset(storage, 0, (size_t)bytes);
    w->window      = (volatile uint32_t *)storage;
    w->window_phys = (uint32_t)PS_FIX_CORE_PROJ_CTRL;   /* 0xA0000000，不写死 */
    w->window_bytes = bytes;
    w->kind        = PS_FIX_WIN_SHADOW;
    w->storage     = storage;
    return PS_FIX_OK;
}

int ps_fix_window_open_uio(ps_fix_window_t *w, const char *uio_dev,
                           char *err, size_t errcap)
{
    const ps_uio_env_t *env;
    ps_uio_map_t m;
    int rc;

    if (w) { memset(w, 0, sizeof(*w)); }
    if (w) { w->fd = -1; }
    if (err && errcap > 0u) { err[0] = '\0'; }
    if (!w) { return PS_FIX_DRV_ERR_ARGS; }

    /* 非 Linux 构建没有 POSIX 后端：明确返回「本平台不可用」，不做假成功。 */
    env = ps_uio_env_default();
    if (!env) {
        if (err && errcap > 0u) {
            snprintf(err, errcap,
                     "本构建没有 POSIX UIO 后端（非 Linux）；设备 %s 可在 Linux 板上打开",
                     uio_dev ? uio_dev : "(auto)");
        }
        return PS_FIX_DRV_ERR_UNAVAILABLE;
    }

    rc = ps_uio_open_map(&m, env, uio_dev, PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, errcap);
    if (rc != PS_UIO_OK) {
        return (rc == PS_UIO_ERR_ARGS) ? PS_FIX_DRV_ERR_ARGS : rc;
    }

    w->window       = (volatile uint32_t *)m.map;
    w->window_phys  = (uint32_t)m.map_phys;
    w->window_bytes = (uint32_t)m.map_bytes;   /* 四核窗口 < 4 GiB，截断安全 */
    w->kind         = PS_FIX_WIN_UIO;
    w->storage      = NULL;
    w->fd           = m.fd;
    return PS_FIX_OK;
}

void ps_fix_window_close(ps_fix_window_t *w)
{
    if (!w) { return; }
    if (w->kind == PS_FIX_WIN_UIO) {
        /* 把窗口字段还原成 ps_uio_map_t 再走 UIO 的 munmap + close。 */
        const ps_uio_env_t *env = ps_uio_env_default();
        if (env) {
            ps_uio_map_t m;
            memset(&m, 0, sizeof(m));
            m.map       = (void *)w->window;
            m.map_phys  = (uint64_t)w->window_phys;
            m.map_bytes = (uint64_t)w->window_bytes;
            m.fd        = w->fd;
            ps_uio_close_map(env, &m);
        }
    }
    memset(w, 0, sizeof(*w));
    w->fd   = -1;
    w->kind = PS_FIX_WIN_NONE;
}

/* ---------------------------------------------------------------- 四核顺序 */

static const int kCoreOrder[4] = {
    PS_FIX_CORE_ID_PROJ, PS_FIX_CORE_ID_FWD,
    PS_FIX_CORE_ID_ADJ,  PS_FIX_CORE_ID_PDHG,
};

const int *ps_fix_core_order(int *n_out)
{
    if (n_out) { *n_out = 4; }
    return kCoreOrder;
}

/* ---------------------------------------------------------------- 管线 */

int ps_fix_pipeline_open(ps_fix_pipeline_t *p, ps_fix_window_t *w)
{
    if (!p || !w || w->kind == PS_FIX_WIN_NONE || !w->window) {
        return PS_FIX_DRV_ERR_ARGS;
    }
    memset(p, 0, sizeof(*p));
    p->win    = w;
    p->shadow = (w->kind == PS_FIX_WIN_SHADOW) ? 1 : 0;

    /* 每个核两个 AXI-Lite 从接口，ctrl 与 ctrl_r 在同一窗口内的不同偏移。 */
    static const uint32_t ctrl_phys[4] = {
        (uint32_t)PS_FIX_CORE_PROJ_CTRL, (uint32_t)PS_FIX_CORE_LR_FWD_CTRL,
        (uint32_t)PS_FIX_CORE_LR_ADJ_CTRL, (uint32_t)PS_FIX_CORE_PDHG_CTRL,
    };
    static const uint32_t ctrlr_phys[4] = {
        (uint32_t)PS_FIX_CORE_PROJ_CTRL_R, (uint32_t)PS_FIX_CORE_LR_FWD_CTRL_R,
        (uint32_t)PS_FIX_CORE_LR_ADJ_CTRL_R, (uint32_t)PS_FIX_CORE_PDHG_CTRL_R,
    };
    int i;
    for (i = 0; i < 4; ++i) {
        int rc = ps_fix_core_open(&p->cores[i], ctrl_phys[i], ctrlr_phys[i],
                                  w->window, w->window_phys);
        if (rc != PS_FIX_OK) { return rc; }
    }
    p->opened = 1;
    return PS_FIX_OK;
}

/* ---------------------------------------------------------------- 追踪 */

static void drv_trace(ps_fix_pipeline_t *p, int core_id, const char *stage)
{
    if (p && p->trace) { p->trace(p->trace_ctx, core_id, stage); }
}

/* ---------------------------------------------------------------- 对齐 */

/* 每个 m_axi 缓冲区基址都必须按数组对齐（= PS_PSM_ARRAY_ALIGN）落位：
 * 跨 4 KiB 边界的 AXI 突发会被拆包，§5.3 的带宽假设随之作废。 */
static int drv_check_aligned(uint64_t addr)
{
    if (addr == 0ull) { return PS_FIX_DRV_ERR_ARGS; }
    if ((addr % (uint64_t)PS_PSM_ARRAY_ALIGN) != 0ull) { return PS_FIX_DRV_ERR_ALIGN; }
    return PS_FIX_OK;
}

/* ---------------------------------------------------------------- 等待 */

/* 起手与板端一致：写 ap_start。真板走 fix_core 的轮询；影子窗口没有 PL 去置
 * ap_done，这里立即置上以模拟「作业瞬时完成」，好让上层继续走到读结果那一步。
 * 影子路径的结论只覆盖寄存器协议，不覆盖时序。 */
static int drv_start_and_wait(ps_fix_pipeline_t *p, ps_fix_core_t *c,
                              uint32_t poll_limit)
{
    if (p->shadow) {
        volatile uint32_t *ctrl = c->ctrl + (PS_FIX_REG_CTRL >> 2);
        *ctrl = PS_FIX_CTRL_AP_START;
        *ctrl = PS_FIX_CTRL_AP_START | PS_FIX_CTRL_AP_DONE;
        return PS_FIX_OK;
    }
    return ps_fix_start_and_wait(c, poll_limit);
}

/* ---------------------------------------------------------------- 结果读取（影子路径） */
/* 真板路径由 ps_fix_*_run 内部完成同样的读取（同一批 fix_core 原语）；
 * 影子路径把它显式写出来，好让自检能核对字段拼接。 */

static void drv_read_proj(ps_fix_core_t *c, ps_fix_proj_result_t *r)
{
    uint32_t u = 0u;
    uint64_t q = 0ull;
    if (!r) { return; }
    ps_fix_read_vld(c, PROJ_BEST_ID,    &u); r->best_id    = u;
    ps_fix_read_vld(c, PROJ_SECOND_ID,  &u); r->second_id  = u;
    ps_fix_read_vld(c, PROJ_MARGIN_Q16, &u); r->margin_q16 = (int32_t)u;
    ps_fix_read_vld(c, PROJ_BOUNDARY,   &u); r->boundary   = u;
    ps_fix_read_vld(c, PROJ_LOW_CONF,   &u); r->low_conf   = u;
    ps_fix_read_vld(c, PROJ_OVERFLOW,   &u); r->overflow   = u;
    ps_fix_read_vld64(c, PROJ_BEST_SCORE,   &q); r->best_score   = q;
    ps_fix_read_vld64(c, PROJ_SECOND_SCORE, &q); r->second_score = q;
    ps_fix_read_vld(c, PROJ_SAT_EVENTS,     &u); r->sat_events     = u;
    ps_fix_read_vld(c, PROJ_MAC_OVERFLOW,   &u); r->mac_overflow   = u;
    ps_fix_read_vld(c, PROJ_OUT_OVERFLOW,   &u); r->out_overflow   = u;
    ps_fix_read_vld(c, PROJ_SCORE_OVERFLOW, &u); r->score_overflow = u;
    ps_fix_read_vld(c, PROJ_GAIN_CLAMPED,   &u); r->gain_clamped   = u;
    ps_fix_read_vld64(c, PROJ_MAX_ABS_ACC,  &q); r->max_abs_acc    = (int64_t)q;
}

static void drv_read_lr(ps_fix_core_t *c, uint32_t base_mac, uint32_t base_sat,
                        ps_fix_lr_result_t *r)
{
    uint32_t u = 0u;
    uint64_t q = 0ull;
    if (!r) { return; }
    ps_fix_read_vld64(c, base_mac, &q); r->mac_count = (int64_t)q;
    ps_fix_read_vld(c, base_sat + (LRF_MAC_OVERFLOW   - LRF_SAT_EVENTS), &u); r->mac_overflow   = u;
    ps_fix_read_vld(c, base_sat + (LRF_OUT_OVERFLOW   - LRF_SAT_EVENTS), &u); r->out_overflow   = u;
    ps_fix_read_vld(c, base_sat + (LRF_SCORE_OVERFLOW - LRF_SAT_EVENTS), &u); r->score_overflow = u;
    ps_fix_read_vld(c, base_sat + (LRF_GAIN_CLAMPED   - LRF_SAT_EVENTS), &u); r->gain_clamped   = u;
    ps_fix_read_vld64(c, base_sat + (LRF_MAX_ABS_ACC  - LRF_SAT_EVENTS), &q);
    r->max_abs_acc = (int64_t)q;
    ps_fix_read_vld(c, base_sat, &u); r->sat_events = u;
}

static void drv_read_pdhg(ps_fix_core_t *c, ps_fix_pdhg_result_t *r)
{
    uint32_t u = 0u;
    uint64_t q = 0ull;
    if (!r) { return; }
    ps_fix_read_vld(c, PDHG_SAT_EVENTS,     &u); r->sat_events     = u;
    ps_fix_read_vld(c, PDHG_MAC_OVERFLOW,   &u); r->mac_overflow   = u;
    ps_fix_read_vld(c, PDHG_OUT_OVERFLOW,   &u); r->out_overflow   = u;
    ps_fix_read_vld(c, PDHG_SCORE_OVERFLOW, &u); r->score_overflow = u;
    ps_fix_read_vld(c, PDHG_GAIN_CLAMPED,   &u); r->gain_clamped   = u;
    ps_fix_read_vld64(c, PDHG_MAX_ABS_ACC,  &q); r->max_abs_acc    = (int64_t)q;
}

/* ---------------------------------------------------------------- 单核执行 */

int ps_fix_run_projection(ps_fix_pipeline_t *p, const ps_fix_proj_job_t *job,
                          uint32_t poll_limit, ps_fix_proj_result_t *out)
{
    int rc;
    if (!p || !p->opened || !job || !out) { return PS_FIX_DRV_ERR_ARGS; }

    rc = drv_check_aligned(job->meas);
    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->u_codes);
    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->gain_q16);
    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->score_out);
    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->score_ch);
    if (rc != PS_FIX_OK) { return rc; }

    memset(out, 0, sizeof(*out));
    drv_trace(p, PS_FIX_CORE_ID_PROJ, "setup");
    rc = ps_fix_proj_setup(&p->cores[PS_FIX_CORE_ID_PROJ],
                           job->meas, job->u_codes, job->gain_q16,
                           job->score_out, job->score_ch,
                           job->P, job->K, job->M,
                           job->nx, job->ny, job->nz,
                           job->margin_thresh_q16);
    if (rc != PS_FIX_OK) { return rc; }

    if (!p->shadow) {
        drv_trace(p, PS_FIX_CORE_ID_PROJ, "run");
        return ps_fix_proj_run(&p->cores[PS_FIX_CORE_ID_PROJ], poll_limit, out);
    }
    drv_trace(p, PS_FIX_CORE_ID_PROJ, "start");
    rc = drv_start_and_wait(p, &p->cores[PS_FIX_CORE_ID_PROJ], poll_limit);
    if (rc != PS_FIX_OK) { return rc; }
    drv_trace(p, PS_FIX_CORE_ID_PROJ, "read");
    drv_read_proj(&p->cores[PS_FIX_CORE_ID_PROJ], out);
    return PS_FIX_OK;
}

int ps_fix_run_forward(ps_fix_pipeline_t *p, const ps_fix_lrf_job_t *job,
                       uint32_t poll_limit, ps_fix_lr_result_t *out)
{
    int rc;
    if (!p || !p->opened || !job || !out) { return PS_FIX_DRV_ERR_ARGS; }
    rc = drv_check_aligned(job->U);   if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->S);   if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->V);   if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->f);   if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->y);   if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->wk_z); if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->wk_zs); if (rc != PS_FIX_OK) { return rc; }

    memset(out, 0, sizeof(*out));
    drv_trace(p, PS_FIX_CORE_ID_FWD, "setup");
    rc = ps_fix_lr_fwd_setup(&p->cores[PS_FIX_CORE_ID_FWD],
                             job->U, job->S, job->V, job->f, job->y,
                             job->wk_z, job->wk_zs,
                             job->K, job->M, job->N, &job->gain);
    if (rc != PS_FIX_OK) { return rc; }

    if (!p->shadow) {
        drv_trace(p, PS_FIX_CORE_ID_FWD, "run");
        return ps_fix_lr_fwd_run(&p->cores[PS_FIX_CORE_ID_FWD], poll_limit, out);
    }
    drv_trace(p, PS_FIX_CORE_ID_FWD, "start");
    rc = drv_start_and_wait(p, &p->cores[PS_FIX_CORE_ID_FWD], poll_limit);
    if (rc != PS_FIX_OK) { return rc; }
    drv_trace(p, PS_FIX_CORE_ID_FWD, "read");
    drv_read_lr(&p->cores[PS_FIX_CORE_ID_FWD], LRF_MAC_COUNT, LRF_SAT_EVENTS, out);
    return PS_FIX_OK;
}

int ps_fix_run_adjoint(ps_fix_pipeline_t *p, const ps_fix_lra_job_t *job,
                       uint32_t poll_limit, ps_fix_lr_result_t *out)
{
    int rc;
    if (!p || !p->opened || !job || !out) { return PS_FIX_DRV_ERR_ARGS; }
    rc = drv_check_aligned(job->U);    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->S);    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->V);    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->r);    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->g);    if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->wk_w); if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->wk_ws); if (rc != PS_FIX_OK) { return rc; }

    memset(out, 0, sizeof(*out));
    drv_trace(p, PS_FIX_CORE_ID_ADJ, "setup");
    rc = ps_fix_lr_adj_setup(&p->cores[PS_FIX_CORE_ID_ADJ],
                             job->U, job->S, job->V, job->r, job->g,
                             job->wk_w, job->wk_ws,
                             job->K, job->M, job->N, &job->gain);
    if (rc != PS_FIX_OK) { return rc; }

    if (!p->shadow) {
        drv_trace(p, PS_FIX_CORE_ID_ADJ, "run");
        return ps_fix_lr_adj_run(&p->cores[PS_FIX_CORE_ID_ADJ], poll_limit, out);
    }
    drv_trace(p, PS_FIX_CORE_ID_ADJ, "start");
    rc = drv_start_and_wait(p, &p->cores[PS_FIX_CORE_ID_ADJ], poll_limit);
    if (rc != PS_FIX_OK) { return rc; }
    drv_trace(p, PS_FIX_CORE_ID_ADJ, "read");
    drv_read_lr(&p->cores[PS_FIX_CORE_ID_ADJ], LRA_MAC_COUNT, LRA_SAT_EVENTS, out);
    return PS_FIX_OK;
}

int ps_fix_run_pdhg(ps_fix_pipeline_t *p, const ps_fix_pdhg_job_t *job,
                    uint32_t poll_limit, ps_fix_pdhg_result_t *out)
{
    int rc;
    if (!p || !p->opened || !job || !out) { return PS_FIX_DRV_ERR_ARGS; }
    rc = drv_check_aligned(job->U);  if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->S);  if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->V);  if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->d3); if (rc != PS_FIX_OK) { return rc; }
    rc = drv_check_aligned(job->f3); if (rc != PS_FIX_OK) { return rc; }

    memset(out, 0, sizeof(*out));
    drv_trace(p, PS_FIX_CORE_ID_PDHG, "setup");
    rc = ps_fix_pdhg_setup(&p->cores[PS_FIX_CORE_ID_PDHG],
                           job->U, job->S, job->V, job->d3, job->f3,
                           &job->scratch, &job->out,
                           job->K, job->M, job->N, &job->gain, &job->params);
    if (rc != PS_FIX_OK) { return rc; }

    if (!p->shadow) {
        drv_trace(p, PS_FIX_CORE_ID_PDHG, "run");
        return ps_fix_pdhg_run(&p->cores[PS_FIX_CORE_ID_PDHG], poll_limit, out);
    }
    drv_trace(p, PS_FIX_CORE_ID_PDHG, "start");
    rc = drv_start_and_wait(p, &p->cores[PS_FIX_CORE_ID_PDHG], poll_limit);
    if (rc != PS_FIX_OK) { return rc; }
    drv_trace(p, PS_FIX_CORE_ID_PDHG, "read");
    drv_read_pdhg(&p->cores[PS_FIX_CORE_ID_PDHG], out);
    return PS_FIX_OK;
}

/* ---------------------------------------------------------------- 四核顺序 */

int ps_fix_run_sequence(ps_fix_pipeline_t *p, const ps_fix_sequence_t *seq,
                        uint32_t poll_limit, ps_fix_sequence_result_t *out)
{
    int rc;
    if (!p || !p->opened || !seq || !out) { return PS_FIX_DRV_ERR_ARGS; }
    if (!seq->proj && !seq->fwd && !seq->adj && !seq->pdhg) {
        return PS_FIX_DRV_ERR_ARGS;
    }
    memset(out, 0, sizeof(*out));
    out->aborted_at = -1;

    /* 顺序不可重排：proj -> fwd -> adj -> pdhg（同 ps_fix_core_order）。 */
    if (seq->proj) {
        rc = ps_fix_run_projection(p, seq->proj, poll_limit, &out->proj);
        if (rc != PS_FIX_OK) { out->aborted_at = PS_FIX_CORE_ID_PROJ; return rc; }
        out->stages_ok++;
    }
    if (seq->fwd) {
        rc = ps_fix_run_forward(p, seq->fwd, poll_limit, &out->fwd);
        if (rc != PS_FIX_OK) { out->aborted_at = PS_FIX_CORE_ID_FWD; return rc; }
        out->stages_ok++;
    }
    if (seq->adj) {
        rc = ps_fix_run_adjoint(p, seq->adj, poll_limit, &out->adj);
        if (rc != PS_FIX_OK) { out->aborted_at = PS_FIX_CORE_ID_ADJ; return rc; }
        out->stages_ok++;
    }
    if (seq->pdhg) {
        rc = ps_fix_run_pdhg(p, seq->pdhg, poll_limit, &out->pdhg);
        if (rc != PS_FIX_OK) { out->aborted_at = PS_FIX_CORE_ID_PDHG; return rc; }
        out->stages_ok++;
    }
    return PS_FIX_OK;
}

/* ---------------------------------------------------------------- 模型视图 */

int ps_fix_model_view(const ps_psm_header_t *h, uint64_t model_phys,
                      uint32_t index, ps_fix_model_view_t *out)
{
    uint64_t rec, m, n;
    if (!h || !out) { return PS_FIX_DRV_ERR_ARGS; }
    if (index >= (uint32_t)h->candidates) { return PS_FIX_DRV_ERR_ARGS; }

    m = ps_psm_meas_len(h);
    n = ps_psm_hidden_len(h);
    rec = ps_psm_candidate_offset(h, index);

    memset(out, 0, sizeof(*out));
    out->record_bytes = ps_psm_record_bytes_hdr(h);
    out->u_bytes      = (uint64_t)h->rank_max * m * 2ull;
    out->cand_phys    = model_phys + rec;
    out->U_phys       = model_phys + rec + ps_psm_u_offset(h->rank_max, m);
    out->S_phys       = model_phys + rec + ps_psm_s_offset(h->rank_max, m);
    out->V_phys       = model_phys + rec + ps_psm_v_offset(h->rank_max, m);
    out->mask_phys    = model_phys + rec + ps_psm_mask_offset(h->rank_max, m, n);
#ifdef PS_PSM_OFF_CAND_S_MAX
    /* s_max 是 2026-09-29 起入 psm 的字段（此前只在 manifest）。用格式头里的
     * 偏移宏取值；若该宏被改名/移除，则从视图里去掉而不硬编码一个数字。 */
    out->s_max_phys = model_phys + rec + (uint64_t)PS_PSM_OFF_CAND_S_MAX;
    out->has_s_max  = 1;
#else
    out->s_max_phys = 0ull;
    out->has_s_max  = 0;
#endif
    return PS_FIX_OK;
}

/* ---------------------------------------------------------------- S 幅度折算 */
/* 见 fix_core_driver.h 的「S 幅度折算」。这里刻意不依赖 libm：位模式判 2 的幂、
 * 一次乘法 + 偶数舍入算 Q16.16，板端与主机链路都不需要 -lm。 */

/* x 正好是 2 的幂时返回 1 并给出指数；否则 0。用 double 的位模式判，不调 frexp。 */
static int fold_pow2_exp(double x, int *exp_out)
{
    uint64_t bits;
    uint64_t biased, mant;
    if (!(x > 0.0)) { return 0; }
    memcpy(&bits, &x, sizeof(bits));
    biased = (bits >> 52) & 0x7FFull;
    mant   = bits & 0xFFFFFFFFFFFFFull;
    if (biased == 0ull || biased == 0x7FFull || mant != 0ull) { return 0; }
    *exp_out = (int)biased - 1023;
    return 1;
}

static double fold_pow2_value(int e)
{
    double v = 1.0;
    int i;
    int n = (e < 0) ? -e : e;
    if (n > 200) { n = 200; }              /* 本项目的指数在 [-30, 30]，此界只为防呆 */
    for (i = 0; i < n; ++i) { v *= (e < 0) ? 0.5 : 2.0; }
    return v;
}

/* A1 严格判据：scale 必须是**精确的 2 的幂**且指数落在 int8（[-128,127]）。
 * 与 ps_fixed.hpp 的 scale_to_exp2 同口径（都要求尾数恰好 1），额外把 int8 界
 * 也在这里挡住 —— manifest 的 exp_int8_ok 说的就是这一条。
 * 返回 1 并写 *exp_out（= 精确指数）；否则返回 0（调用方必须拒绝，不许近似）。 */
static int fold_scale_exp_strict(double scale, int32_t *exp_out)
{
    int e = 0;
    if (!fold_pow2_exp(scale, &e)) { return 0; }
    if (e < -128 || e > 127) { return 0; }
    *exp_out = (int32_t)e;
    return 1;
}

/* (m*2^16) 向最近取整、半值取偶，与 ps_fixed.hpp 的 round_half_even 同口径。 */
static int32_t fold_gain_q16(double m)
{
    double scaled = m * 65536.0;
    long long t;
    double frac;
    if (scaled >= 2147483647.0) { return 0x7FFFFFFF; }   /* 与核的 gain 安全区一致 */
    t    = (long long)scaled;                            /* m > 0，强转即向下取整 */
    frac = scaled - (double)t;
    if (frac > 0.5) { ++t; }
    else if (frac == 0.5 && (t & 1LL)) { ++t; }
    return (int32_t)t;
}

/* 折算出「乘数 m -> (gain_q16, exp)」的公共核心。exp 有效表示走幂次支路（精确）。 */
static void fold_magnitude(double m, int32_t *gain_q16, int32_t *exp_out,
                           float *multiplier, float *rel_err, int *exact)
{
    int e = 0;
    if (fold_pow2_exp(m, &e)) {
        *gain_q16   = 0;                 /* 幂次支路下核忽略 gain 字段 */
        *exp_out    = (int32_t)e;
        *multiplier = (float)fold_pow2_value(e);
        *rel_err    = 0.0f;
        *exact      = 1;
        return;
    }
    *gain_q16  = fold_gain_q16(m);
    *exp_out   = PS_FIX_GAIN_EXP_INVALID;
    *multiplier = (float)((double)(*gain_q16) / 65536.0);
    {
        double d = (double)(*multiplier) - m;
        if (d < 0.0) { d = -d; }
        *rel_err = (float)(d / m);
    }
    *exact = 0;
}

int ps_fix_fold_multiplier(double m, ps_fix_s_fold_t *out)
{
    if (!out) { return PS_FIX_DRV_ERR_ARGS; }
    memset(out, 0, sizeof(*out));
    if (!(m > 0.0) || m > DBL_MAX) { return PS_FIX_DRV_ERR_ARGS; }
    fold_magnitude(m, &out->s_gain_q16, &out->s_exp,
                   &out->multiplier, &out->rel_err, &out->exact);
    return PS_FIX_OK;
}

int ps_fix_fold_s_scale(float s_scale, float s_max, ps_fix_s_fold_t *out)
{
    if (!(s_scale > 0.0f) || !(s_max > 0.0f) || s_max > FLT_MAX) {
        if (out) { memset(out, 0, sizeof(*out)); }
        return PS_FIX_DRV_ERR_ARGS;
    }
    /* 乘数是 s_scale*s_max，不是单独一个：S 码字 = 归一化奇异值/ s_scale。 */
    return ps_fix_fold_multiplier((double)s_scale * (double)s_max, out);
}

int ps_fix_fold_candidate_s(const uint8_t *cand_base, ps_fix_s_fold_t *out)
{
    float s_scale, s_max;
    if (!cand_base || !out) { return PS_FIX_DRV_ERR_ARGS; }
    /* 偏移取自 psm_format.h 的候选头布局（+12 s_scale / +20 s_max），不写死数字。 */
    s_scale = ps_psm_read_f32(cand_base,
                              (uint64_t)offsetof(ps_psm_candidate_hdr_t, s_scale));
#ifdef PS_PSM_OFF_CAND_S_MAX
    s_max = ps_psm_read_f32(cand_base, (uint64_t)PS_PSM_OFF_CAND_S_MAX);
#else
    s_max = ps_psm_read_f32(cand_base,
                            (uint64_t)offsetof(ps_psm_candidate_hdr_t, s_max));
#endif
    return ps_fix_fold_s_scale(s_scale, s_max, out);
}

void ps_fix_gain_apply_s(ps_fix_gain_t *g, const ps_fix_s_fold_t *f)
{
    if (!g || !f) { return; }
    g->s_gain_q16 = f->s_gain_q16;
    g->s_exp      = f->s_exp;
    /* U/V 的六个字段由调用方负责，本函数不碰 —— 避免把「只改 S」的要求扩散成三处。 */
}

int ps_fix_check_candidate_gain_scales(const uint8_t *cand_base)
{
    float u_scale, v_scale;
    int32_t e = 0;
    if (!cand_base) { return PS_FIX_DRV_ERR_ARGS; }
    u_scale = ps_psm_read_f32(cand_base,
                              (uint64_t)offsetof(ps_psm_candidate_hdr_t, u_scale));
    v_scale = ps_psm_read_f32(cand_base,
                              (uint64_t)offsetof(ps_psm_candidate_hdr_t, v_scale));
    /* A1：二者都必须是精确的 2 的幂且指数在 int8 内，否则拒绝（不近似、不回退）。 */
    if (!fold_scale_exp_strict((double)u_scale, &e) ||
        !fold_scale_exp_strict((double)v_scale, &e)) {
        return PS_FIX_DRV_ERR_GAIN_EXP;
    }
    return PS_FIX_OK;
}

int ps_fix_gain_from_candidate(const uint8_t *cand_base, ps_fix_gain_t *out)
{
    float u_scale, s_scale, v_scale, s_max;
    int32_t u_exp = 0, v_exp = 0;
    if (!cand_base || !out) { return PS_FIX_DRV_ERR_ARGS; }

    u_scale = ps_psm_read_f32(cand_base,
                              (uint64_t)offsetof(ps_psm_candidate_hdr_t, u_scale));
    s_scale = ps_psm_read_f32(cand_base,
                              (uint64_t)offsetof(ps_psm_candidate_hdr_t, s_scale));
    v_scale = ps_psm_read_f32(cand_base,
                              (uint64_t)offsetof(ps_psm_candidate_hdr_t, v_scale));
#ifdef PS_PSM_OFF_CAND_S_MAX
    s_max = ps_psm_read_f32(cand_base, (uint64_t)PS_PSM_OFF_CAND_S_MAX);
#else
    s_max = ps_psm_read_f32(cand_base,
                            (uint64_t)offsetof(ps_psm_candidate_hdr_t, s_max));
#endif
    if (!(u_scale > 0.0f) || !(s_scale > 0.0f) || !(v_scale > 0.0f) || !(s_max > 0.0f) || s_max > FLT_MAX) {
        return PS_FIX_DRV_ERR_ARGS;
    }
    /* A1：U/V 走 exp 支路的前提是 scale 为**精确的 2 的幂**（build_model 的
     * quantize_symmetric_pow2 保证）。非 2 的幂在此**拒绝**，不再落到 Q16.16 ——
     * 旧的连续 scale（1e-6 量级）折成 Q16.16 会取整到 0，核输出全零且不报错。 */
    if (!fold_scale_exp_strict((double)u_scale, &u_exp) ||
        !fold_scale_exp_strict((double)v_scale, &v_exp)) {
        memset(out, 0, sizeof(*out));
        out->u_exp = PS_FIX_GAIN_EXP_INVALID;
        out->s_exp = PS_FIX_GAIN_EXP_INVALID;
        out->v_exp = PS_FIX_GAIN_EXP_INVALID;
        return PS_FIX_DRV_ERR_GAIN_EXP;
    }
    /* 幂次支路下核忽略 gain 字段（保持与旧行为逐位一致：0）。 */
    out->u_gain_q16 = 0; out->u_exp = u_exp;
    out->v_gain_q16 = 0; out->v_exp = v_exp;
    {
        /* S 的乘数是 s_scale*s_max（s_max 一般非 2 的幂）：Q16.16 支路是**合法**的，
         * 且对该乘数是整体缩放误差、不引入逐元素噪声，故这里不套用 A1 的严格判据。 */
        ps_fix_s_fold_t f;
        int rc = ps_fix_fold_s_scale(s_scale, s_max, &f);
        if (rc != PS_FIX_OK) { return rc; }
        ps_fix_gain_apply_s(out, &f);
    }
    return PS_FIX_OK;
}

/* ---------------------------------------------------------------- 帧量程接缝（meas_norm） */
/* 见 fix_core_driver.h 的「帧量程接缝」。本层只**调用** meas_norm，不重写它的口径
 * （权威 = src/model/penumbrasight_model/quantize.py 的 shared_exponent）。 */

int ps_fix_prepare_meas(const uint32_t *rtl24_beats, uint32_t rows, uint32_t cols,
                        uint32_t channels, int32_t *meas_cf,
                        ps_meas_norm_result_t *frame)
{
    uint64_t n;

    if (!rtl24_beats || !meas_cf || rows == 0u || cols == 0u || channels == 0u) {
        return PS_FIX_DRV_ERR_ARGS;
    }
    n = (uint64_t)rows * (uint64_t)cols * (uint64_t)channels;
    if (n == 0ull || n > 0xFFFFFFFFull) { return PS_FIX_DRV_ERR_ARGS; }

    /* 24 位打包拍 -> 通道优先 Q2.16。这一行就是「meas_norm 接进流程」的全部内容：
     * 符号扩展、共享指数、通道优先重排都在 meas_norm 里，本层不复制一份。 */
    if (ps_meas_norm_from_rtl24(rtl24_beats, rows, cols, channels, meas_cf, frame)
            != PS_MEAS_OK) {
        return PS_FIX_DRV_ERR_ARGS;
    }
    return PS_FIX_OK;
}

int ps_fix_meas_scale_exponent(const ps_meas_norm_result_t *frame)
{
    if (!frame) { return 0; }
    /* d_scale = 2^(exponent - frac_bits)；与 divider_exp = frac_bits - exponent 互逆。 */
    return frame->exponent - frame->frac_bits;
}

double ps_fix_meas_d_scale(const ps_meas_norm_result_t *frame)
{
    if (!frame) { return 0.0; }
    return fold_pow2_value(ps_fix_meas_scale_exponent(frame));
}

int ps_fix_fold_frame_scale(const ps_meas_norm_result_t *frame, float u_scale,
                            ps_fix_frame_scale_t *out)
{
    ps_fix_s_fold_t f;
    int e;

    if (!frame || !out || !(u_scale > 0.0f)) { return PS_FIX_DRV_ERR_ARGS; }
    e = ps_fix_meas_scale_exponent(frame);
    if (e < -200 || e > 200) { return PS_FIX_DRV_ERR_ARGS; }

    memset(&f, 0, sizeof(f));
    /* 总乘数 m = u_scale * d_scale；走与 S 幅度折算同一套「2 的幂支路 / Q16.16 支路」。 */
    fold_magnitude((double)u_scale * fold_pow2_value(e),
                   &f.s_gain_q16, &f.s_exp, &f.multiplier, &f.rel_err, &f.exact);

    out->multiplier  = (double)f.multiplier;
    out->gain_q16    = f.s_gain_q16;
    out->exp         = f.s_exp;
    out->exact       = f.exact;
    out->d_scale_exp = e;
    return PS_FIX_OK;
}

int ps_fix_proj_attach_meas(ps_fix_proj_job_t *job, uint64_t meas_phys,
                            float u_scale, const ps_meas_norm_result_t *frame,
                            ps_fix_frame_scale_t *scale_out)
{
    if (!job || meas_phys == 0ull) { return PS_FIX_DRV_ERR_ARGS; }
    job->meas = meas_phys;
    if (scale_out) {
        int rc = ps_fix_fold_frame_scale(frame, u_scale, scale_out);
        if (rc != PS_FIX_OK) { return rc; }
    }
    return PS_FIX_OK;
}
