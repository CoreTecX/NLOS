/* ============================================================================
 * PenumbraSight · 板端 PS 侧寄存器层实现
 * board/fix_core.c
 *
 * 见 board/fix_core.h 的边界说明：本层不碰 mmap、不分配缓冲区、不加载模型。
 * ==========================================================================*/

#include "fix_core.h"

/* 64 位结果寄存器后面跟的 VLD 偏移：数据占 o 与 o+4，VLD 在 o+8。
 * （ps_fix_regs.h 的 PS_FIX_VLD_OFF = 0x4 是给**32 位**结果用的，别混） */
#define PS_FIX_VLD64_OFF 0x008u

/* 偏移都是 4 字节对齐，所以按下标寻址；不要用 char* 做指针运算，
 * 那会把 volatile 限定丢掉。 */
static volatile uint32_t *at(volatile uint32_t *base, uint32_t off)
{
    return base + (off >> 2);
}

/* 一次寄存器写：先通知观测钩子（若装了），再真写。顺序与实际写一致，
 * 这样钩子记下的序列就是真实的写序（用于断言「先 HI 再 LO」）。 */
static void wr32(ps_fix_core_t *c, volatile uint32_t *base, uint32_t off, uint32_t v)
{
    if (c && c->write_obs) { c->write_obs(c->write_obs_ctx, off, v); }
    *at(base, off) = v;
}

int ps_fix_core_open(ps_fix_core_t *c, uint32_t ctrl_phys, uint32_t ctrl_r_phys,
                     volatile uint32_t *window, uint32_t window_phys)
{
    if (!c || !window) { return PS_FIX_ERR_ARG; }
    /* 两个基址都必须落在同一个映射窗口内，否则指针会算到窗口外面去。
     * 窗口取 0xA0000000 起 512 KiB 就能覆盖四个核的八段（见 regmap §2）。 */
    if (ctrl_phys < window_phys || ctrl_r_phys < window_phys) { return PS_FIX_ERR_ARG; }

    c->ctrl     = at(window, ctrl_phys - window_phys);
    c->ctrl_r   = at(window, ctrl_r_phys - window_phys);
    c->ctrl_phys     = ctrl_phys;
    c->ctrl_r_phys   = ctrl_r_phys;
    c->write_obs     = 0;   /* 默认关闭写序观测（主机自检自行装上） */
    c->write_obs_ctx = 0;
    c->clock_ns = 0; c->clock_ctx = 0; c->timeout_ns = 0;
    return PS_FIX_OK;
}

void ps_fix_set_base(ps_fix_core_t *c, uint32_t o, uint64_t addr)
{
    /* **先 HI 再 LO**：核在 ap_start 后开始取地址，若先落 LO，硬件可能先看到
     * 「新 LO + 旧 HI」这种半截地址，去访问一个根本不存在的缓冲区。
     * 先写 HI 能保证 LO 落笔时 HI 已是新值。
     * 与 board/ps_fix_regs.h 的 ps_fix_write_base64、baremetal/main.c 的 seq_base64、
     * board_regseq.c 的 push_base、fpga_control.c 的 ps_fpga_write_addr64 同一纪律。 */
    wr32(c, c->ctrl_r, o + PS_FIX_64_HI, (uint32_t)(addr >> 32));
    wr32(c, c->ctrl_r, o + PS_FIX_64_LO, (uint32_t)(addr & 0xFFFFFFFFu));
}

int ps_fix_start_and_wait(ps_fix_core_t *c, uint32_t poll_limit)
{
    volatile uint32_t *ctrl;
    uint64_t started;
    if (!c || !c->ctrl || !poll_limit) return PS_FIX_ERR_ARG;
    ctrl = at(c->ctrl, PS_FIX_REG_CTRL);
    started = c->clock_ns ? c->clock_ns(c->clock_ctx) : 0;

    /* 写 ap_start。CTRL 的 bit1(ap_done) 是「写 ap_start 时清零、作业完成时置位」，
     * 所以在同一次作业内轮询它是可靠的。 */
    *ctrl = PS_FIX_CTRL_AP_START;

    while (poll_limit--) {
        if (*ctrl & PS_FIX_CTRL_AP_DONE) { return PS_FIX_OK; }
        if (c->clock_ns && c->timeout_ns &&
            c->clock_ns(c->clock_ctx) - started >= c->timeout_ns) return PS_FIX_ERR_TIMEOUT;
    }
    return PS_FIX_ERR_TIMEOUT;
}

int ps_fix_read_vld(ps_fix_core_t *c, uint32_t o, uint32_t *out)
{
    /* **先读数据、再读 VLD**：VLD 是 ap_vld(COR)，读一下就清，
     * 反过来的话会先清掉标志再读数据，读到的有效性与数据不同步。 */
    uint32_t d = *at(c->ctrl, o);
    uint32_t v = *at(c->ctrl, o + PS_FIX_VLD_OFF);
    if (out) { *out = d; }
    return (v & 0x1u) ? 1 : 0;
}

int ps_fix_read_vld64(ps_fix_core_t *c, uint32_t o, uint64_t *out)
{
    uint32_t lo = *at(c->ctrl, o + PS_FIX_64_LO);
    uint32_t hi = *at(c->ctrl, o + PS_FIX_64_HI);
    uint32_t v  = *at(c->ctrl, o + PS_FIX_VLD64_OFF);
    if (out) { *out = ((uint64_t)hi << 32) | (uint64_t)lo; }
    return (v & 0x1u) ? 1 : 0;
}

/* ==========================================================================
 * 投影核
 * ========================================================================*/

int ps_fix_proj_setup(ps_fix_core_t *c,
                      uint64_t meas, uint64_t u_codes, uint64_t gain_q16,
                      uint64_t score_out, uint64_t score_ch,
                      uint32_t P, uint32_t K, uint32_t M,
                      uint32_t nx, uint32_t ny, uint32_t nz,
                      uint32_t margin_thresh_q16)
{
    if (!c) { return PS_FIX_ERR_ARG; }

    ps_fix_set_base(c, PROJ_BASE_MEAS,     meas);
    ps_fix_set_base(c, PROJ_BASE_U_CODES,  u_codes);
    ps_fix_set_base(c, PROJ_BASE_GAIN_Q16, gain_q16);
    ps_fix_set_base(c, PROJ_BASE_SCORE_OUT, score_out);
    ps_fix_set_base(c, PROJ_BASE_SCORE_CH,  score_ch);

    *at(c->ctrl, PROJ_P) = P;
    *at(c->ctrl, PROJ_K) = K;
    *at(c->ctrl, PROJ_M) = M;
    *at(c->ctrl, PROJ_NX) = nx;
    *at(c->ctrl, PROJ_NY) = ny;
    *at(c->ctrl, PROJ_NZ) = nz;
    *at(c->ctrl, PROJ_MARGIN_THRESH_Q16) = margin_thresh_q16;
    return PS_FIX_OK;
}

int ps_fix_proj_run(ps_fix_core_t *c, uint32_t poll_limit, ps_fix_proj_result_t *r)
{
    uint32_t u = 0;
    uint64_t q = 0;
    int rc;

    if (!c) { return PS_FIX_ERR_ARG; }
    rc = ps_fix_start_and_wait(c, poll_limit);
    if (rc != PS_FIX_OK || !r) { return rc; }

    /* 一次作业把整组结果都写出来，所以这里逐个读、不逐个判 VLD。
     * 需要「哪些字段真的写了」时，用 ps_fix_read_vld 的返回值自己判。 */
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
    return PS_FIX_OK;
}

/* ==========================================================================
 * 低秩前向 / 伴随
 * ========================================================================*/

/* 前向与伴随的 s_axi_control 布局**逐字节相同**，只差基址与缓冲区别名，
 * 所以标量部分抽成一个共用函数，省得一改改两处、改漏一处。 */
static void lr_setup_common(ps_fix_core_t *c, uint32_t K, uint32_t M, uint32_t N,
                            const ps_fix_gain_t *g)
{
    *at(c->ctrl, LRF_K) = K;
    *at(c->ctrl, LRF_M) = M;
    *at(c->ctrl, LRF_N) = N;
    if (!g) { return; }
    *at(c->ctrl, LRF_U_GAIN_Q16) = (uint32_t)g->u_gain_q16;
    *at(c->ctrl, LRF_S_GAIN_Q16) = (uint32_t)g->s_gain_q16;
    *at(c->ctrl, LRF_V_GAIN_Q16) = (uint32_t)g->v_gain_q16;
    *at(c->ctrl, LRF_U_EXP) = (uint32_t)g->u_exp;
    *at(c->ctrl, LRF_S_EXP) = (uint32_t)g->s_exp;
    *at(c->ctrl, LRF_V_EXP) = (uint32_t)g->v_exp;
}

static void lr_read_common(ps_fix_core_t *c, ps_fix_lr_result_t *r)
{
    uint32_t u = 0;
    uint64_t q = 0;
    ps_fix_read_vld64(c, LRF_MAC_COUNT,     &q); r->mac_count      = (int64_t)q;
    ps_fix_read_vld(c, LRF_SAT_EVENTS,      &u); r->sat_events     = u;
    ps_fix_read_vld(c, LRF_MAC_OVERFLOW,    &u); r->mac_overflow   = u;
    ps_fix_read_vld(c, LRF_OUT_OVERFLOW,    &u); r->out_overflow   = u;
    ps_fix_read_vld(c, LRF_SCORE_OVERFLOW,  &u); r->score_overflow = u;
    ps_fix_read_vld(c, LRF_GAIN_CLAMPED,    &u); r->gain_clamped   = u;
    ps_fix_read_vld64(c, LRF_MAX_ABS_ACC,   &q); r->max_abs_acc    = (int64_t)q;
}

int ps_fix_lr_fwd_setup(ps_fix_core_t *c,
                        uint64_t U, uint64_t S, uint64_t V,
                        uint64_t f, uint64_t y, uint64_t wk_z, uint64_t wk_zs,
                        uint32_t K, uint32_t M, uint32_t N,
                        const ps_fix_gain_t *g)
{
    if (!c) { return PS_FIX_ERR_ARG; }
    ps_fix_set_base(c, LRF_BASE_U,     U);
    ps_fix_set_base(c, LRF_BASE_S,     S);
    ps_fix_set_base(c, LRF_BASE_V,     V);
    ps_fix_set_base(c, LRF_BASE_F,     f);
    ps_fix_set_base(c, LRF_BASE_Y,     y);
    ps_fix_set_base(c, LRF_BASE_WK_Z,  wk_z);
    ps_fix_set_base(c, LRF_BASE_WK_ZS, wk_zs);
    lr_setup_common(c, K, M, N, g);
    return PS_FIX_OK;
}

int ps_fix_lr_fwd_run(ps_fix_core_t *c, uint32_t poll_limit, ps_fix_lr_result_t *r)
{
    int rc;
    if (!c) { return PS_FIX_ERR_ARG; }
    rc = ps_fix_start_and_wait(c, poll_limit);
    if (rc != PS_FIX_OK || !r) { return rc; }
    lr_read_common(c, r);
    return PS_FIX_OK;
}

int ps_fix_lr_adj_setup(ps_fix_core_t *c,
                        uint64_t U, uint64_t S, uint64_t V,
                        uint64_t r, uint64_t g, uint64_t wk_w, uint64_t wk_ws,
                        uint32_t K, uint32_t M, uint32_t N,
                        const ps_fix_gain_t *gain)
{
    if (!c) { return PS_FIX_ERR_ARG; }
    ps_fix_set_base(c, LRA_BASE_U,     U);
    ps_fix_set_base(c, LRA_BASE_S,     S);
    ps_fix_set_base(c, LRA_BASE_V,     V);
    ps_fix_set_base(c, LRA_BASE_R,     r);
    ps_fix_set_base(c, LRA_BASE_G,     g);
    ps_fix_set_base(c, LRA_BASE_WK_W,  wk_w);
    ps_fix_set_base(c, LRA_BASE_WK_WS, wk_ws);
    lr_setup_common(c, K, M, N, gain);
    return PS_FIX_OK;
}

int ps_fix_lr_adj_run(ps_fix_core_t *c, uint32_t poll_limit, ps_fix_lr_result_t *r)
{
    int rc;
    if (!c) { return PS_FIX_ERR_ARG; }
    rc = ps_fix_start_and_wait(c, poll_limit);
    if (rc != PS_FIX_OK || !r) { return rc; }
    lr_read_common(c, r);
    return PS_FIX_OK;
}

/* ==========================================================================
 * PDHG 重建
 * ========================================================================*/

int ps_fix_pdhg_setup(ps_fix_core_t *c,
                      uint64_t U, uint64_t S, uint64_t V,
                      uint64_t d3, uint64_t f3,
                      const ps_fix_pdhg_scratch_t *scratch,
                      const ps_fix_pdhg_outputs_t *out,
                      uint32_t K, uint32_t M, uint32_t N,
                      const ps_fix_gain_t *gain,
                      const ps_fix_pdhg_params_t *p)
{
    if (!c || !scratch || !out || !p) { return PS_FIX_ERR_ARG; }

    ps_fix_set_base(c, PDHG_BASE_U, U);
    ps_fix_set_base(c, PDHG_BASE_S, S);
    ps_fix_set_base(c, PDHG_BASE_V, V);
    ps_fix_set_base(c, PDHG_BASE_D3, d3);
    ps_fix_set_base(c, PDHG_BASE_F3, f3);

    ps_fix_set_base(c, PDHG_BASE_WK_F_PREV, scratch->wk_f_prev);
    ps_fix_set_base(c, PDHG_BASE_WK_FBAR,   scratch->wk_fbar);
    ps_fix_set_base(c, PDHG_BASE_WK_Q,      scratch->wk_q);
    ps_fix_set_base(c, PDHG_BASE_WK_GRAD,   scratch->wk_grad);
    ps_fix_set_base(c, PDHG_BASE_WK_DIVQ,   scratch->wk_divq);
    ps_fix_set_base(c, PDHG_BASE_WK_Y,      scratch->wk_y);
    ps_fix_set_base(c, PDHG_BASE_WK_R,      scratch->wk_r);
    ps_fix_set_base(c, PDHG_BASE_WK_BTR,    scratch->wk_btr);
    ps_fix_set_base(c, PDHG_BASE_WK_TMP,    scratch->wk_tmp);

    ps_fix_set_base(c, PDHG_BASE_OUT_N_USED,        out->out_n_used);
    ps_fix_set_base(c, PDHG_BASE_OUT_EARLY_STOPPED, out->out_early_stopped);
    ps_fix_set_base(c, PDHG_BASE_OUT_OVERFLOW_CH,   out->out_overflow_ch);
    ps_fix_set_base(c, PDHG_BASE_OUT_RHO_Q16,       out->out_rho_q16);
    ps_fix_set_base(c, PDHG_BASE_OUT_RES_SQ,        out->out_res_sq);
    ps_fix_set_base(c, PDHG_BASE_OUT_D_SQ,          out->out_d_sq);
    ps_fix_set_base(c, PDHG_BASE_OUT_MAC_COUNT,     out->out_mac_count);

    *at(c->ctrl, PDHG_K) = K;
    *at(c->ctrl, PDHG_M) = M;
    *at(c->ctrl, PDHG_N) = N;
    if (gain) {
        *at(c->ctrl, PDHG_U_GAIN_Q16) = (uint32_t)gain->u_gain_q16;
        *at(c->ctrl, PDHG_S_GAIN_Q16) = (uint32_t)gain->s_gain_q16;
        *at(c->ctrl, PDHG_V_GAIN_Q16) = (uint32_t)gain->v_gain_q16;
        *at(c->ctrl, PDHG_U_EXP) = (uint32_t)gain->u_exp;
        *at(c->ctrl, PDHG_S_EXP) = (uint32_t)gain->s_exp;
        *at(c->ctrl, PDHG_V_EXP) = (uint32_t)gain->v_exp;
    }
    *at(c->ctrl, PDHG_N_ROWS)         = p->n_rows;
    *at(c->ctrl, PDHG_N_COLS)         = p->n_cols;
    *at(c->ctrl, PDHG_N_ITER)         = p->n_iter;
    *at(c->ctrl, PDHG_MIN_ITERS)      = p->min_iters;
    *at(c->ctrl, PDHG_RESIDUAL_EVERY) = p->residual_every;
    *at(c->ctrl, PDHG_TV_MODE)        = p->tv_mode;
    *at(c->ctrl, PDHG_TAU_Q16)        = (uint32_t)p->tau_q16;
    *at(c->ctrl, PDHG_SIGMA_Q16)      = (uint32_t)p->sigma_q16;
    *at(c->ctrl, PDHG_LAMBDA_Q16)     = (uint32_t)p->lambda_q16;
    *at(c->ctrl, PDHG_THETA_Q16)      = (uint32_t)p->theta_q16;
    *at(c->ctrl, PDHG_RHO_STOP_Q8)    = (uint32_t)p->rho_stop_q8;
    return PS_FIX_OK;
}

int ps_fix_pdhg_run(ps_fix_core_t *c, uint32_t poll_limit, ps_fix_pdhg_result_t *r)
{
    uint32_t u = 0;
    uint64_t q = 0;
    int rc;

    if (!c) { return PS_FIX_ERR_ARG; }
    rc = ps_fix_start_and_wait(c, poll_limit);
    if (rc != PS_FIX_OK || !r) { return rc; }

    ps_fix_read_vld(c, PDHG_SAT_EVENTS,     &u); r->sat_events     = u;
    ps_fix_read_vld(c, PDHG_MAC_OVERFLOW,   &u); r->mac_overflow   = u;
    ps_fix_read_vld(c, PDHG_OUT_OVERFLOW,   &u); r->out_overflow   = u;
    ps_fix_read_vld(c, PDHG_SCORE_OVERFLOW, &u); r->score_overflow = u;
    ps_fix_read_vld(c, PDHG_GAIN_CLAMPED,   &u); r->gain_clamped   = u;
    ps_fix_read_vld64(c, PDHG_MAX_ABS_ACC,  &q); r->max_abs_acc    = (int64_t)q;
    return PS_FIX_OK;
}

void ps_fix_pdhg_collect_channels(const int32_t  *n_used,
                                  const int32_t  *early_stopped,
                                  const int32_t  *overflow_ch,
                                  const uint32_t *rho_q16,
                                  const uint64_t *res_sq,
                                  const uint64_t *d_sq,
                                  const uint64_t *mac_count,
                                  int n, ps_fix_pdhg_channels_t *dst)
{
    int i;
    if (!dst) { return; }
    if (n > 3) { n = 3; }
    if (n < 0) { n = 0; }

    for (i = 0; i < n; ++i) {
        dst->n_used[i]        = n_used        ? n_used[i]        : 0;
        dst->early_stopped[i] = early_stopped ? early_stopped[i] : 0;
        dst->overflow_ch[i]   = overflow_ch   ? overflow_ch[i]   : 0;
        dst->rho_q16[i]       = rho_q16       ? rho_q16[i]       : 0u;
        dst->res_sq[i]        = res_sq        ? res_sq[i]        : 0ull;
        dst->d_sq[i]          = d_sq          ? d_sq[i]          : 0ull;
        dst->mac_count[i]     = mac_count     ? mac_count[i]     : 0ull;
    }
    /* 未填满的通道清零，避免调用方读到栈上的垃圾 */
    for (; i < 3; ++i) {
        dst->n_used[i] = 0; dst->early_stopped[i] = 0; dst->overflow_ch[i] = 0;
        dst->rho_q16[i] = 0u; dst->res_sq[i] = 0ull; dst->d_sq[i] = 0ull;
        dst->mac_count[i] = 0ull;
    }
}
