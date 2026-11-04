/* ============================================================================
 * PenumbraSight · 板端顶层应用「寄存器初始化序列」实现
 * src/software/board_app/board_regseq.c
 *
 * 见 board_regseq.h 的边界与顺序约束。本文件只有一件要小心的事：每个偏移都从
 * board/ps_fix_regs.h 的宏来，**不写裸字面量** —— 那正是这张清单存在的意义，
 * 若这里抄错一个偏移，清单与代码会一起错，主机侧的交叉比对（test_board_top.c）
 * 就失去作用。
 * ==========================================================================*/

#include "board_regseq.h"

#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------- 段基址 */

uint64_t ps_board_core_ctrl_base(int core)
{
    switch (core) {
        case PS_FIX_CORE_ID_PROJ: return (uint64_t)PS_FIX_CORE_PROJ_CTRL;
        case PS_FIX_CORE_ID_FWD:  return (uint64_t)PS_FIX_CORE_LR_FWD_CTRL;
        case PS_FIX_CORE_ID_ADJ:  return (uint64_t)PS_FIX_CORE_LR_ADJ_CTRL;
        case PS_FIX_CORE_ID_PDHG: return (uint64_t)PS_FIX_CORE_PDHG_CTRL;
        default:                  return 0ull;   /* 0 不是合法段基址 */
    }
}

uint64_t ps_board_core_ctrl_r_base(int core)
{
    switch (core) {
        case PS_FIX_CORE_ID_PROJ: return (uint64_t)PS_FIX_CORE_PROJ_CTRL_R;
        case PS_FIX_CORE_ID_FWD:  return (uint64_t)PS_FIX_CORE_LR_FWD_CTRL_R;
        case PS_FIX_CORE_ID_ADJ:  return (uint64_t)PS_FIX_CORE_LR_ADJ_CTRL_R;
        case PS_FIX_CORE_ID_PDHG: return (uint64_t)PS_FIX_CORE_PDHG_CTRL_R;
        default:                  return 0ull;
    }
}

/* ---------------------------------------------------------------- 步骤压入 */

void ps_board_regseq_reset(ps_board_regseq_t *seq)
{
    if (!seq) { return; }
    seq->n = 0;
    seq->overflow = 0;
}

static void push(ps_board_regseq_t *s, uint32_t kind, int core, uint32_t space,
                 uint64_t addr, uint32_t offset, uint32_t value, const char *name)
{
    ps_board_regseq_step_t *e;
    if (s->n >= PS_BOARD_REGSEQ_MAX) { s->overflow = 1; return; }
    e = &s->step[s->n++];
    e->kind   = kind;
    e->core   = core;
    e->space  = space;
    e->addr   = addr;
    e->offset = offset;
    e->value  = value;
    e->name   = name;
}

/* 一个 64 位基址 = 先 HI(+0x4) 再 LO(+0x0) 两步。 */
static void push_base(ps_board_regseq_t *s, int core, uint32_t ctrlr_base,
                      uint32_t off, uint64_t a,
                      const char *name_hi, const char *name_lo)
{
    push(s, PS_REGSEQ_WRITE, core, PS_REGSEQ_SP_CTRL_R,
         (uint64_t)ctrlr_base + (uint64_t)off + (uint64_t)PS_FIX_64_HI,
         off + PS_FIX_64_HI, (uint32_t)(a >> 32), name_hi);
    push(s, PS_REGSEQ_WRITE, core, PS_REGSEQ_SP_CTRL_R,
         (uint64_t)ctrlr_base + (uint64_t)off + (uint64_t)PS_FIX_64_LO,
         off + PS_FIX_64_LO, (uint32_t)(a & 0xFFFFFFFFu), name_lo);
}

static void push_scalar(ps_board_regseq_t *s, int core, uint32_t ctrl_base,
                        uint32_t off, uint32_t v, const char *name)
{
    push(s, PS_REGSEQ_WRITE, core, PS_REGSEQ_SP_CTRL,
         (uint64_t)ctrl_base + (uint64_t)off, off, v, name);
}

/* ---------------------------------------------------------------- 各核 */

static void seq_proj(ps_board_regseq_t *s, const ps_fix_proj_job_t *j)
{
    const int      core = PS_FIX_CORE_ID_PROJ;
    const uint32_t cb   = (uint32_t)PS_FIX_CORE_PROJ_CTRL;
    const uint32_t cr   = (uint32_t)PS_FIX_CORE_PROJ_CTRL_R;

    push_base(s, core, cr, PROJ_BASE_MEAS,     j->meas,     "PROJ_BASE_MEAS.HI",     "PROJ_BASE_MEAS.LO");
    push_base(s, core, cr, PROJ_BASE_U_CODES,  j->u_codes,  "PROJ_BASE_U_CODES.HI",  "PROJ_BASE_U_CODES.LO");
    push_base(s, core, cr, PROJ_BASE_GAIN_Q16, j->gain_q16, "PROJ_BASE_GAIN_Q16.HI", "PROJ_BASE_GAIN_Q16.LO");
    push_base(s, core, cr, PROJ_BASE_SCORE_OUT, j->score_out, "PROJ_BASE_SCORE_OUT.HI", "PROJ_BASE_SCORE_OUT.LO");
    push_base(s, core, cr, PROJ_BASE_SCORE_CH,  j->score_ch,  "PROJ_BASE_SCORE_CH.HI",  "PROJ_BASE_SCORE_CH.LO");

    push_scalar(s, core, cb, PROJ_P,                j->P,                "PROJ_P");
    push_scalar(s, core, cb, PROJ_K,                j->K,                "PROJ_K");
    push_scalar(s, core, cb, PROJ_M,                j->M,                "PROJ_M");
    push_scalar(s, core, cb, PROJ_NX,               j->nx,               "PROJ_NX");
    push_scalar(s, core, cb, PROJ_NY,               j->ny,               "PROJ_NY");
    push_scalar(s, core, cb, PROJ_NZ,               j->nz,               "PROJ_NZ");
    push_scalar(s, core, cb, PROJ_MARGIN_THRESH_Q16, j->margin_thresh_q16, "PROJ_MARGIN_THRESH_Q16");
}

static void seq_fwd(ps_board_regseq_t *s, const ps_fix_lrf_job_t *j)
{
    const int      core = PS_FIX_CORE_ID_FWD;
    const uint32_t cb   = (uint32_t)PS_FIX_CORE_LR_FWD_CTRL;
    const uint32_t cr   = (uint32_t)PS_FIX_CORE_LR_FWD_CTRL_R;

    push_base(s, core, cr, LRF_BASE_U,     j->U,     "LRF_BASE_U.HI",     "LRF_BASE_U.LO");
    push_base(s, core, cr, LRF_BASE_S,     j->S,     "LRF_BASE_S.HI",     "LRF_BASE_S.LO");
    push_base(s, core, cr, LRF_BASE_V,     j->V,     "LRF_BASE_V.HI",     "LRF_BASE_V.LO");
    push_base(s, core, cr, LRF_BASE_F,     j->f,     "LRF_BASE_F.HI",     "LRF_BASE_F.LO");
    push_base(s, core, cr, LRF_BASE_Y,     j->y,     "LRF_BASE_Y.HI",     "LRF_BASE_Y.LO");
    push_base(s, core, cr, LRF_BASE_WK_Z,  j->wk_z,  "LRF_BASE_WK_Z.HI",  "LRF_BASE_WK_Z.LO");
    push_base(s, core, cr, LRF_BASE_WK_ZS, j->wk_zs, "LRF_BASE_WK_ZS.HI", "LRF_BASE_WK_ZS.LO");

    push_scalar(s, core, cb, LRF_K,          j->K,          "LRF_K");
    push_scalar(s, core, cb, LRF_M,          j->M,          "LRF_M");
    push_scalar(s, core, cb, LRF_N,          j->N,          "LRF_N");
    push_scalar(s, core, cb, LRF_U_GAIN_Q16, (uint32_t)j->gain.u_gain_q16, "LRF_U_GAIN_Q16");
    push_scalar(s, core, cb, LRF_S_GAIN_Q16, (uint32_t)j->gain.s_gain_q16, "LRF_S_GAIN_Q16");
    push_scalar(s, core, cb, LRF_V_GAIN_Q16, (uint32_t)j->gain.v_gain_q16, "LRF_V_GAIN_Q16");
    push_scalar(s, core, cb, LRF_U_EXP,      (uint32_t)j->gain.u_exp,      "LRF_U_EXP");
    push_scalar(s, core, cb, LRF_S_EXP,      (uint32_t)j->gain.s_exp,      "LRF_S_EXP");
    push_scalar(s, core, cb, LRF_V_EXP,      (uint32_t)j->gain.v_exp,      "LRF_V_EXP");
}

static void seq_adj(ps_board_regseq_t *s, const ps_fix_lra_job_t *j)
{
    const int      core = PS_FIX_CORE_ID_ADJ;
    const uint32_t cb   = (uint32_t)PS_FIX_CORE_LR_ADJ_CTRL;
    const uint32_t cr   = (uint32_t)PS_FIX_CORE_LR_ADJ_CTRL_R;

    /* 前向与伴随的标量偏移逐字节相同，缓冲区别名不同（见 regmap §4.3）。 */
    push_base(s, core, cr, LRA_BASE_U,     j->U,     "LRA_BASE_U.HI",     "LRA_BASE_U.LO");
    push_base(s, core, cr, LRA_BASE_S,     j->S,     "LRA_BASE_S.HI",     "LRA_BASE_S.LO");
    push_base(s, core, cr, LRA_BASE_V,     j->V,     "LRA_BASE_V.HI",     "LRA_BASE_V.LO");
    push_base(s, core, cr, LRA_BASE_R,     j->r,     "LRA_BASE_R.HI",     "LRA_BASE_R.LO");
    push_base(s, core, cr, LRA_BASE_G,     j->g,     "LRA_BASE_G.HI",     "LRA_BASE_G.LO");
    push_base(s, core, cr, LRA_BASE_WK_W,  j->wk_w,  "LRA_BASE_WK_W.HI",  "LRA_BASE_WK_W.LO");
    push_base(s, core, cr, LRA_BASE_WK_WS, j->wk_ws, "LRA_BASE_WK_WS.HI", "LRA_BASE_WK_WS.LO");

    push_scalar(s, core, cb, LRA_K,          j->K,          "LRA_K");
    push_scalar(s, core, cb, LRA_M,          j->M,          "LRA_M");
    push_scalar(s, core, cb, LRA_N,          j->N,          "LRA_N");
    push_scalar(s, core, cb, LRA_U_GAIN_Q16, (uint32_t)j->gain.u_gain_q16, "LRA_U_GAIN_Q16");
    push_scalar(s, core, cb, LRA_S_GAIN_Q16, (uint32_t)j->gain.s_gain_q16, "LRA_S_GAIN_Q16");
    push_scalar(s, core, cb, LRA_V_GAIN_Q16, (uint32_t)j->gain.v_gain_q16, "LRA_V_GAIN_Q16");
    push_scalar(s, core, cb, LRA_U_EXP,      (uint32_t)j->gain.u_exp,      "LRA_U_EXP");
    push_scalar(s, core, cb, LRA_S_EXP,      (uint32_t)j->gain.s_exp,      "LRA_S_EXP");
    push_scalar(s, core, cb, LRA_V_EXP,      (uint32_t)j->gain.v_exp,      "LRA_V_EXP");
}

static void seq_pdhg(ps_board_regseq_t *s, const ps_fix_pdhg_job_t *j)
{
    const int      core = PS_FIX_CORE_ID_PDHG;
    const uint32_t cb   = (uint32_t)PS_FIX_CORE_PDHG_CTRL;
    const uint32_t cr   = (uint32_t)PS_FIX_CORE_PDHG_CTRL_R;

    push_base(s, core, cr, PDHG_BASE_U,          j->U,          "PDHG_BASE_U.HI",          "PDHG_BASE_U.LO");
    push_base(s, core, cr, PDHG_BASE_S,          j->S,          "PDHG_BASE_S.HI",          "PDHG_BASE_S.LO");
    push_base(s, core, cr, PDHG_BASE_V,          j->V,          "PDHG_BASE_V.HI",          "PDHG_BASE_V.LO");
    push_base(s, core, cr, PDHG_BASE_D3,         j->d3,         "PDHG_BASE_D3.HI",         "PDHG_BASE_D3.LO");
    push_base(s, core, cr, PDHG_BASE_F3,         j->f3,         "PDHG_BASE_F3.HI",         "PDHG_BASE_F3.LO");
    push_base(s, core, cr, PDHG_BASE_WK_F_PREV,  j->scratch.wk_f_prev, "PDHG_BASE_WK_F_PREV.HI", "PDHG_BASE_WK_F_PREV.LO");
    push_base(s, core, cr, PDHG_BASE_WK_FBAR,    j->scratch.wk_fbar,   "PDHG_BASE_WK_FBAR.HI",   "PDHG_BASE_WK_FBAR.LO");
    push_base(s, core, cr, PDHG_BASE_WK_Q,       j->scratch.wk_q,      "PDHG_BASE_WK_Q.HI",      "PDHG_BASE_WK_Q.LO");
    push_base(s, core, cr, PDHG_BASE_WK_GRAD,    j->scratch.wk_grad,   "PDHG_BASE_WK_GRAD.HI",   "PDHG_BASE_WK_GRAD.LO");
    push_base(s, core, cr, PDHG_BASE_WK_DIVQ,    j->scratch.wk_divq,   "PDHG_BASE_WK_DIVQ.HI",   "PDHG_BASE_WK_DIVQ.LO");
    push_base(s, core, cr, PDHG_BASE_WK_Y,       j->scratch.wk_y,      "PDHG_BASE_WK_Y.HI",      "PDHG_BASE_WK_Y.LO");
    push_base(s, core, cr, PDHG_BASE_WK_R,       j->scratch.wk_r,      "PDHG_BASE_WK_R.HI",      "PDHG_BASE_WK_R.LO");
    push_base(s, core, cr, PDHG_BASE_WK_BTR,     j->scratch.wk_btr,    "PDHG_BASE_WK_BTR.HI",    "PDHG_BASE_WK_BTR.LO");
    push_base(s, core, cr, PDHG_BASE_WK_TMP,     j->scratch.wk_tmp,    "PDHG_BASE_WK_TMP.HI",    "PDHG_BASE_WK_TMP.LO");
    push_base(s, core, cr, PDHG_BASE_OUT_N_USED,        j->out.out_n_used,        "PDHG_BASE_OUT_N_USED.HI",        "PDHG_BASE_OUT_N_USED.LO");
    push_base(s, core, cr, PDHG_BASE_OUT_EARLY_STOPPED, j->out.out_early_stopped, "PDHG_BASE_OUT_EARLY_STOPPED.HI", "PDHG_BASE_OUT_EARLY_STOPPED.LO");
    push_base(s, core, cr, PDHG_BASE_OUT_OVERFLOW_CH,   j->out.out_overflow_ch,   "PDHG_BASE_OUT_OVERFLOW_CH.HI",   "PDHG_BASE_OUT_OVERFLOW_CH.LO");
    push_base(s, core, cr, PDHG_BASE_OUT_RHO_Q16,       j->out.out_rho_q16,       "PDHG_BASE_OUT_RHO_Q16.HI",       "PDHG_BASE_OUT_RHO_Q16.LO");
    push_base(s, core, cr, PDHG_BASE_OUT_RES_SQ,        j->out.out_res_sq,        "PDHG_BASE_OUT_RES_SQ.HI",        "PDHG_BASE_OUT_RES_SQ.LO");
    push_base(s, core, cr, PDHG_BASE_OUT_D_SQ,          j->out.out_d_sq,          "PDHG_BASE_OUT_D_SQ.HI",          "PDHG_BASE_OUT_D_SQ.LO");
    push_base(s, core, cr, PDHG_BASE_OUT_MAC_COUNT,     j->out.out_mac_count,     "PDHG_BASE_OUT_MAC_COUNT.HI",     "PDHG_BASE_OUT_MAC_COUNT.LO");

    push_scalar(s, core, cb, PDHG_K,          j->K,          "PDHG_K");
    push_scalar(s, core, cb, PDHG_M,          j->M,          "PDHG_M");
    push_scalar(s, core, cb, PDHG_N,          j->N,          "PDHG_N");
    push_scalar(s, core, cb, PDHG_U_GAIN_Q16, (uint32_t)j->gain.u_gain_q16, "PDHG_U_GAIN_Q16");
    push_scalar(s, core, cb, PDHG_S_GAIN_Q16, (uint32_t)j->gain.s_gain_q16, "PDHG_S_GAIN_Q16");
    push_scalar(s, core, cb, PDHG_V_GAIN_Q16, (uint32_t)j->gain.v_gain_q16, "PDHG_V_GAIN_Q16");
    push_scalar(s, core, cb, PDHG_U_EXP,      (uint32_t)j->gain.u_exp,      "PDHG_U_EXP");
    push_scalar(s, core, cb, PDHG_S_EXP,      (uint32_t)j->gain.s_exp,      "PDHG_S_EXP");
    push_scalar(s, core, cb, PDHG_V_EXP,      (uint32_t)j->gain.v_exp,      "PDHG_V_EXP");
    push_scalar(s, core, cb, PDHG_N_ROWS,         j->params.n_rows,         "PDHG_N_ROWS");
    push_scalar(s, core, cb, PDHG_N_COLS,         j->params.n_cols,         "PDHG_N_COLS");
    push_scalar(s, core, cb, PDHG_N_ITER,         j->params.n_iter,         "PDHG_N_ITER");
    push_scalar(s, core, cb, PDHG_MIN_ITERS,      j->params.min_iters,      "PDHG_MIN_ITERS");
    push_scalar(s, core, cb, PDHG_RESIDUAL_EVERY, j->params.residual_every, "PDHG_RESIDUAL_EVERY");
    push_scalar(s, core, cb, PDHG_TV_MODE,        j->params.tv_mode,        "PDHG_TV_MODE");
    push_scalar(s, core, cb, PDHG_TAU_Q16,        (uint32_t)j->params.tau_q16,     "PDHG_TAU_Q16");
    push_scalar(s, core, cb, PDHG_SIGMA_Q16,      (uint32_t)j->params.sigma_q16,   "PDHG_SIGMA_Q16");
    push_scalar(s, core, cb, PDHG_LAMBDA_Q16,     (uint32_t)j->params.lambda_q16,  "PDHG_LAMBDA_Q16");
    push_scalar(s, core, cb, PDHG_THETA_Q16,      (uint32_t)j->params.theta_q16,   "PDHG_THETA_Q16");
    push_scalar(s, core, cb, PDHG_RHO_STOP_Q8,    (uint32_t)j->params.rho_stop_q8, "PDHG_RHO_STOP_Q8");
}

/* 每个核的收尾：写 ap_start，再轮询 ap_done。 */
static void seq_start_poll(ps_board_regseq_t *s, int core)
{
    const uint32_t cb = (uint32_t)ps_board_core_ctrl_base(core);
    push(s, PS_REGSEQ_WRITE, core, PS_REGSEQ_SP_CTRL,
         (uint64_t)cb + PS_FIX_REG_CTRL, PS_FIX_REG_CTRL, PS_FIX_CTRL_AP_START,
         "CTRL.ap_start");
    push(s, PS_REGSEQ_POLL, core, PS_REGSEQ_SP_CTRL,
         (uint64_t)cb + PS_FIX_REG_CTRL, PS_FIX_REG_CTRL, PS_FIX_CTRL_AP_DONE,
         "CTRL.ap_done");
}

/* ---------------------------------------------------------------- 生成 */

int ps_board_regseq_build(ps_board_regseq_t *seq, const ps_board_regseq_cfg_t *cfg)
{
    if (!seq || !cfg) { return -1; }
    if (!cfg->proj && !cfg->fwd && !cfg->adj && !cfg->pdhg) { return -1; }

    ps_board_regseq_reset(seq);

    /* 第一步：接口版本读回。排在整段序列最前 —— 版本不符时后续偏移全部无意义。
     * 读的是 PL 控制页（ps_regs）的 VERSION 寄存器；同一页 0x000 处是 ID
     * （0x5053_0001），可先做存在性探测。该页在 2026-09-30 已生成的位流里还没有，
     * 因此在旧位流上这一步不可达（UNMAPPED），见 board_regseq.h 的说明。 */
    if (cfg->include_version) {
        push(seq, PS_REGSEQ_READ, -1, PS_REGSEQ_SP_ABS,
             (uint64_t)PS_BOARD_REGSEQ_IFVER_ADDR, 0u, cfg->if_version,
             "PL_CTRL.VERSION(==PS_FIX_IF_VERSION)");
    }

    if (cfg->proj) { seq_proj(seq, cfg->proj); seq_start_poll(seq, PS_FIX_CORE_ID_PROJ); }
    if (cfg->fwd)  { seq_fwd(seq, cfg->fwd);   seq_start_poll(seq, PS_FIX_CORE_ID_FWD); }
    if (cfg->adj)  { seq_adj(seq, cfg->adj);   seq_start_poll(seq, PS_FIX_CORE_ID_ADJ); }
    if (cfg->pdhg) { seq_pdhg(seq, cfg->pdhg); seq_start_poll(seq, PS_FIX_CORE_ID_PDHG); }
    return 0;
}

/* ---------------------------------------------------------------- 回放 */

int ps_board_regseq_apply_writes(const ps_board_regseq_t *seq, ps_fix_window_t *win)
{
    int i, wrote = 0;
    if (!seq || !win || !win->window) { return -1; }

    for (i = 0; i < seq->n; ++i) {
        const ps_board_regseq_step_t *e = &seq->step[i];
        uint64_t off_in_win;
        if (e->kind != PS_REGSEQ_WRITE) { continue; }
        /* 只回放落在映射窗口内的写；越界的跳过（调用方按条数核对，不静默）。 */
        if (e->addr < (uint64_t)win->window_phys) { continue; }
        off_in_win = e->addr - (uint64_t)win->window_phys;
        if (off_in_win + 4ull > (uint64_t)win->window_bytes) { continue; }
        win->window[(uint32_t)(off_in_win >> 2)] = e->value;
        ++wrote;
    }
    return wrote;
}

int ps_board_regseq_count_kind(const ps_board_regseq_t *seq, uint32_t kind)
{
    int i, n = 0;
    if (!seq) { return -1; }
    for (i = 0; i < seq->n; ++i) {
        if (seq->step[i].kind == kind) { ++n; }
    }
    return n;
}

/* ---------------------------------------------------------------- 打印 */

static const char *kind_name(uint32_t k)
{
    switch (k) {
        case PS_REGSEQ_WRITE: return "WRITE";
        case PS_REGSEQ_READ:  return "READ ";
        case PS_REGSEQ_POLL:  return "POLL ";
        case PS_REGSEQ_MARK:  return "MARK ";
        default:              return "?????";
    }
}

static const char *space_name(uint32_t s)
{
    switch (s) {
        case PS_REGSEQ_SP_CTRL:   return "ctrl  ";
        case PS_REGSEQ_SP_CTRL_R: return "ctrl_r";
        case PS_REGSEQ_SP_ABS:    return "abs   ";
        default:                  return "??????";
    }
}

static const char *core_name(int c)
{
    switch (c) {
        case PS_FIX_CORE_ID_PROJ: return "proj";
        case PS_FIX_CORE_ID_FWD:  return "fwd ";
        case PS_FIX_CORE_ID_ADJ:  return "adj ";
        case PS_FIX_CORE_ID_PDHG: return "pdhg";
        default:                  return "-";
    }
}

int ps_board_regseq_print(const ps_board_regseq_t *seq, char *out, size_t cap)
{
    size_t used = 0u;
    int i;

    if (!seq || !out || cap == 0u) { return -1; }

    /* appender：写入若干字符，保证 NUL 结尾且不越界。返回 1 表示还有空间。 */
#define APPEND_FMT(...)                                                     \
    do {                                                                    \
        if (used + 1u < cap) {                                              \
            int w = snprintf(out + used, cap - used, __VA_ARGS__);          \
            if (w > 0) {                                                    \
                if ((size_t)w >= cap - used) { used = cap - 1u; }           \
                else { used += (size_t)w; }                                 \
            }                                                               \
        }                                                                   \
    } while (0)

    APPEND_FMT("序号 动作  核   段     偏移   绝对地址     值           名称\n");
    for (i = 0; i < seq->n; ++i) {
        const ps_board_regseq_step_t *e = &seq->step[i];
        APPEND_FMT("%4d %s %s %s 0x%05X 0x%08lX  0x%08X  %s\n",
                   i + 1, kind_name(e->kind), core_name(e->core),
                   space_name(e->space), e->offset,
                   (unsigned long)e->addr, e->value, e->name ? e->name : "?");
    }
    if (seq->overflow) {
        APPEND_FMT("!! 清单被截断（步骤超过 %d 条）\n", (int)PS_BOARD_REGSEQ_MAX);
    }
    out[used] = '\0';
#undef APPEND_FMT
    return (int)used;
}
