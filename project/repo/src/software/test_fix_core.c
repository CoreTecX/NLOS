/* ============================================================================
 * PenumbraSight · 定点四核调用层的主机自检
 * src/software/test_fix_core.c
 *
 * 可立即运行（不依赖板卡、不依赖 Xilinx 头文件）：
 *     gcc -std=c11 -Wall -Wextra -I../../board -Iinclude \
 *         ../../board/fix_core.c fpga_control/fix_core_driver.c \
 *         test_fix_core.c -o bin/test_fix_core
 *     bin/test_fix_core
 * 退出码非零表示失败。`make -C src/software test-fix-core` 会跑它。
 *
 * 影子窗口验什么、不验什么
 * ---------------------------------------------------------------------------
 * 验（主机可判）：
 *   · 寄存器映射：四核八段的基址落点、m_axi 基址 HI/LO 两个半字、标量偏移；
 *   · VLD 拼接：32 位与 64 位结果的取值与 VLD 位；
 *   · 四核顺序：proj -> lr_fwd -> lr_adj -> pdhg，且失败即中止；
 *   · 模型视图：候选记录内 U/S/V/mask 的偏移全部由 psm_format.h 算出；
 *   · S 幅度折算：从候选头（psm_format.h 布局）读 s_scale(+12)/s_max(+20)，折成
 *     s_gain_q16/s_exp，并与「manifest 侧期望值」（由 manifest 的 s_scale/s_max 独立
 *     算出的标量）逐值比对；结果再写进 LRF/LRA/PDHG 的 S 寄存器并读回核对。
 *   · 参数门禁：未对齐基址 / 空指针 / 窗口越界各自返回对应错误码。
 *   · 写序（A3）：ps_fix_set_base 对每个 64 位基址必须**先写 HI 再写 LO**。
 *     影子内存只留最终值，看不出顺序，故用 ps_fix_core_t.write_obs 观测钩子
 *     记录真实写序再断言（见 test_base_write_order）。
 * 不验（等板卡）：
 *   · 真实 AXI 读写的时序与 ap_done 到达时间（影子把 done 直接置上）；
 *   · 「先写 HI 再写 LO」在真硬件上是否真的消除了半截地址窗口 —— 本测试只证明
 *     **软件发出的写序**是 [HI, LO]，硬件侧的效应只能板上测。
 * ==========================================================================*/

#include <stdio.h>
#include <string.h>

#include "fix_core_driver.h"

/* 覆盖 0xA0000000 起八段共 512 KiB。 */
#define WIN_BYTES (8u * 0x10000u)
static uint32_t g_win[WIN_BYTES / 4u];

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do {                                   \
        if (cond) { g_pass++; printf("  PASS  %s\n", (msg)); }  \
        else { g_fail++; printf("  FAIL  %s\n", (msg)); }       \
    } while (0)

static uint32_t rd(uint32_t phys_off)
{
    return g_win[phys_off >> 2];
}

/* 一组互不相同、且 4096 对齐的 DDR 物理地址（i 从 1 起）。 */
static uint64_t fake_addr(unsigned i)
{
    return 0x000100000000ull + (uint64_t)i * 0x1000ull;
}

/* ---------------------------------------------------------------- 追踪 */

static int g_trace[16];
static int g_ntrace = 0;

static void on_trace(void *ctx, int core_id, const char *stage)
{
    (void)ctx;
    if (strcmp(stage, "setup") == 0 && g_ntrace < 16) {
        g_trace[g_ntrace++] = core_id;
    }
}

/* ---------------------------------------------------------------- 写序观测 */
/* ps_fix_set_base 每次写寄存器都会回调它；记下 (off, val) 序列用于断言写序。 */
static uint32_t g_obs_off[64];
static uint32_t g_obs_val[64];
static int      g_obs_n = 0;

static void on_write_obs(void *ctx, uint32_t off, uint32_t val)
{
    (void)ctx;
    if (g_obs_n < 64) {
        g_obs_off[g_obs_n] = off;
        g_obs_val[g_obs_n] = val;
        ++g_obs_n;
    }
}

/* ---------------------------------------------------------------- 作业装配 */

static void fill_gain(ps_fix_gain_t *g)
{
    g->u_gain_q16 = 0x1A2B;
    g->s_gain_q16 = 0x2B3C;
    g->v_gain_q16 = 0x3C4D;
    g->u_exp = -17;
    g->s_exp = -14;
    g->v_exp = -16;
}

static ps_fix_proj_job_t make_proj_job(void)
{
    ps_fix_proj_job_t j;
    memset(&j, 0, sizeof(j));
    j.meas     = fake_addr(1);
    j.u_codes  = fake_addr(2);
    j.gain_q16 = fake_addr(3);
    j.score_out = fake_addr(4);
    j.score_ch  = fake_addr(5);
    j.P = 243; j.K = 8; j.M = 9120;
    j.nx = 9; j.ny = 9; j.nz = 3;
    j.margin_thresh_q16 = 0x8000u;
    return j;
}

static ps_fix_lrf_job_t make_fwd_job(void)
{
    ps_fix_lrf_job_t j;
    memset(&j, 0, sizeof(j));
    j.U = fake_addr(10); j.S = fake_addr(11); j.V = fake_addr(12);
    j.f = fake_addr(13); j.y = fake_addr(14);
    j.wk_z = fake_addr(15); j.wk_zs = fake_addr(16);
    j.K = 8; j.M = 9120; j.N = 432;
    fill_gain(&j.gain);
    return j;
}

static ps_fix_lra_job_t make_adj_job(void)
{
    ps_fix_lra_job_t j;
    memset(&j, 0, sizeof(j));
    j.U = fake_addr(10); j.S = fake_addr(11); j.V = fake_addr(12);
    j.r = fake_addr(20); j.g = fake_addr(21);
    j.wk_w = fake_addr(22); j.wk_ws = fake_addr(23);
    j.K = 8; j.M = 9120; j.N = 432;
    fill_gain(&j.gain);
    return j;
}

static ps_fix_pdhg_job_t make_pdhg_job(void)
{
    ps_fix_pdhg_job_t j;
    memset(&j, 0, sizeof(j));
    j.U = fake_addr(10); j.S = fake_addr(11); j.V = fake_addr(12);
    j.d3 = fake_addr(30); j.f3 = fake_addr(31);
    j.scratch.wk_f_prev = fake_addr(32);
    j.scratch.wk_fbar   = fake_addr(33);
    j.scratch.wk_q      = fake_addr(34);
    j.scratch.wk_grad   = fake_addr(35);
    j.scratch.wk_divq   = fake_addr(36);
    j.scratch.wk_y      = fake_addr(37);
    j.scratch.wk_r      = fake_addr(38);
    j.scratch.wk_btr    = fake_addr(39);
    j.scratch.wk_tmp    = fake_addr(40);
    j.out.out_n_used        = fake_addr(41);
    j.out.out_early_stopped = fake_addr(42);
    j.out.out_overflow_ch   = fake_addr(43);
    j.out.out_rho_q16       = fake_addr(44);
    j.out.out_res_sq        = fake_addr(45);
    j.out.out_d_sq          = fake_addr(46);
    j.out.out_mac_count     = fake_addr(47);
    j.K = 8; j.M = 9120; j.N = 432;
    fill_gain(&j.gain);
    j.params.n_rows = 18; j.params.n_cols = 24; j.params.n_iter = 1024;
    j.params.min_iters = 8; j.params.residual_every = 4; j.params.tv_mode = 0;
    j.params.tau_q16 = 18053; j.params.sigma_q16 = 18053;
    j.params.lambda_q16 = 66; j.params.theta_q16 = 65536;
    j.params.rho_stop_q8 = 0;
    return j;
}

/* ---------------------------------------------------------------- 用例 */

static void test_window_and_open(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    int rc;
    unsigned ctrlr_off;

    /* 窗口过小应被拒 */
    rc = ps_fix_window_open_shadow(&w, g_win, 0x1000u);
    CHECK(rc == PS_FIX_DRV_ERR_ARGS, "影子窗口过小被拒");

    rc = ps_fix_window_open_shadow(&w, g_win, WIN_BYTES);
    CHECK(rc == PS_FIX_OK, "影子窗口打开成功");
    CHECK(w.window_phys == (uint32_t)PS_FIX_CORE_PROJ_CTRL,
          "窗口物理基址 = PS_FIX_CORE_PROJ_CTRL");

    rc = ps_fix_pipeline_open(&p, &w);
    CHECK(rc == PS_FIX_OK && p.opened == 1 && p.shadow == 1,
          "四核管线打开（影子）");

    /* 每个核的 ctrl / ctrl_r 指针应落在窗口内正确偏移 */
    ctrlr_off = (uint32_t)(PS_FIX_CORE_LR_FWD_CTRL_R - w.window_phys);
    CHECK(p.cores[PS_FIX_CORE_ID_FWD].ctrl ==
          (volatile uint32_t *)g_win +
              (((uint32_t)PS_FIX_CORE_LR_FWD_CTRL - w.window_phys) >> 2),
          "lr_fwd.ctrl 指向 0xA0020000");
    CHECK(p.cores[PS_FIX_CORE_ID_FWD].ctrl_r ==
          (volatile uint32_t *)g_win + (ctrlr_off >> 2),
          "lr_fwd.ctrl_r 指向 0xA0030000");

    /* 基址落在窗口外时 ps_fix_core_open 应拒（这里 window_phys 高于 ctrl） */
    {
        ps_fix_core_t c;
        rc = ps_fix_core_open(&c, (uint32_t)PS_FIX_CORE_PROJ_CTRL,
                              (uint32_t)PS_FIX_CORE_PROJ_CTRL_R,
                              w.window, 0xB0000000u);
        CHECK(rc == PS_FIX_ERR_ARG, "窗口外基址被 ps_fix_core_open 拒");
    }

    /* UIO 后端已按标准 UIO 填实（见 uio_backend.c）。主机（非 Linux）构建没有
     * POSIX 后端，故这里返回 UNAVAILABLE；真板上的系统调用与设备树前提等板卡。 */
    {
        char err[200];
        rc = ps_fix_window_open_uio(&w, "/dev/uio0", err, sizeof(err));
        CHECK(rc == PS_FIX_DRV_ERR_UNAVAILABLE, "主机无 POSIX 后端 -> UNAVAILABLE（非留桩）");
    }
}

static void test_projection_regmap(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    ps_fix_proj_job_t job;
    ps_fix_proj_result_t res;
    int rc;

    ps_fix_window_open_shadow(&w, g_win, WIN_BYTES);
    ps_fix_pipeline_open(&p, &w);
    job = make_proj_job();
    rc = ps_fix_run_projection(&p, &job, 16u, &res);
    CHECK(rc == PS_FIX_OK, "proj 作业在影子窗口返回 OK");

    /* s_axi_control_r（0xA0010000）里的 m_axi 基址：HI/LO 两个半字都要对 */
    CHECK(rd(0x10000u + PROJ_BASE_MEAS + PS_FIX_64_LO) == (uint32_t)(job.meas & 0xFFFFFFFFu),
          "PROJ_BASE_MEAS.LO 正确");
    CHECK(rd(0x10000u + PROJ_BASE_MEAS + PS_FIX_64_HI) == (uint32_t)(job.meas >> 32),
          "PROJ_BASE_MEAS.HI 正确");
    CHECK(rd(0x10000u + PROJ_BASE_U_CODES + PS_FIX_64_LO) == (uint32_t)(job.u_codes & 0xFFFFFFFFu),
          "PROJ_BASE_U_CODES.LO 正确");
    CHECK(rd(0x10000u + PROJ_BASE_U_CODES + PS_FIX_64_HI) == (uint32_t)(job.u_codes >> 32),
          "PROJ_BASE_U_CODES.HI 正确");
    CHECK(rd(0x10000u + PROJ_BASE_SCORE_CH + PS_FIX_64_HI) == (uint32_t)(job.score_ch >> 32),
          "PROJ_BASE_SCORE_CH.HI 正确");

    /* s_axi_control（0xA0000000）里的标量 */
    CHECK(rd(PROJ_P) == job.P, "PROJ_P 标量正确");
    CHECK(rd(PROJ_K) == job.K, "PROJ_K 标量正确");
    CHECK(rd(PROJ_M) == job.M, "PROJ_M 标量正确");
    CHECK(rd(PROJ_NX) == job.nx && rd(PROJ_NZ) == job.nz, "PROJ_NX/NZ 标量正确");
    CHECK(rd(PROJ_MARGIN_THRESH_Q16) == job.margin_thresh_q16,
          "PROJ_MARGIN_THRESH_Q16 标量正确");

    /* 起手写 ap_start，影子置 ap_done；二字都在 CTRL 里 */
    CHECK((rd(PS_FIX_REG_CTRL) & PS_FIX_CTRL_AP_START) != 0u, "CTRL.ap_start 已写");
    CHECK((rd(PS_FIX_REG_CTRL) & PS_FIX_CTRL_AP_DONE) != 0u, "CTRL.ap_done 已到（影子替身）");
}

/* A3：64 位基址写序必须是 [HI, LO]。
 * 影子内存只保留最终值，看不出顺序；故装一个写序观测钩子，取 ps_fix_set_base
 * 真实发出的写序来断言。5 个地址的 HI 都非零，保证「反了」一定能被值判据抓到。 */
static void test_base_write_order(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    ps_fix_core_t *c;
    static const uint64_t addr[5] = {
        0x0000123456789ABCull, 0x0000123456789ABDull, 0x0000123456789ABEull,
        0x0000123456789ABFull, 0x0000123456789AC0ull};
    static const uint32_t offs[5] = {
        PROJ_BASE_MEAS, PROJ_BASE_U_CODES, PROJ_BASE_GAIN_Q16,
        PROJ_BASE_SCORE_OUT, PROJ_BASE_SCORE_CH};
    int i, rc, ok_off = 1, ok_val = 1;

    printf("\n---- A3. 64 位基址写序（先 HI 后 LO）----\n");

    ps_fix_window_open_shadow(&w, g_win, WIN_BYTES);
    ps_fix_pipeline_open(&p, &w);
    c = &p.cores[PS_FIX_CORE_ID_PROJ];

    /* 生产路径必须默认关闭（否则每次基址写都要多一次回调） */
    CHECK(c->write_obs == NULL, "ps_fix_core_open 后写序观测默认关闭");

    c->write_obs = on_write_obs;
    c->write_obs_ctx = NULL;
    g_obs_n = 0;

    rc = ps_fix_proj_setup(c, addr[0], addr[1], addr[2], addr[3], addr[4],
                           243u, 8u, 9120u, 9u, 9u, 3u, 0x8000u);
    CHECK(rc == PS_FIX_OK, "proj_setup 返回 OK");
    CHECK(g_obs_n == 10, "观测到 10 次基址写 = 5 个 64 位对");

    for (i = 0; i < 5; ++i) {
        if (g_obs_off[2 * i]     != offs[i] + PS_FIX_64_HI) ok_off = 0;
        if (g_obs_off[2 * i + 1] != offs[i] + PS_FIX_64_LO) ok_off = 0;
        if (g_obs_val[2 * i]     != (uint32_t)(addr[i] >> 32)) ok_val = 0;
        if (g_obs_val[2 * i + 1] != (uint32_t)(addr[i] & 0xFFFFFFFFu)) ok_val = 0;
    }
    CHECK(ok_off, "5 个基址逐个写序 = [HI(+4), LO(+0)]（HI 在前、LO 紧邻其后）");
    CHECK(ok_val, "写序里的值 = [addr>>32, addr&0xFFFFFFFF]（不是反向）");
}

static void test_result_vld_readback(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    ps_fix_proj_job_t job;
    ps_fix_proj_result_t res;
    int rc;

    ps_fix_window_open_shadow(&w, g_win, WIN_BYTES);
    ps_fix_pipeline_open(&p, &w);

    /* 预置结果寄存器（模拟 PL 写完），核对 VLD 位与数值拼接 */
    g_win[PROJ_BEST_ID / 4u] = 121u;
    g_win[(PROJ_BEST_ID + PS_FIX_VLD_OFF) / 4u] = 1u;
    g_win[PROJ_SECOND_ID / 4u] = 99u;
    g_win[(PROJ_SECOND_ID + PS_FIX_VLD_OFF) / 4u] = 1u;
    g_win[PROJ_MARGIN_Q16 / 4u] = 0xFFFF8000u;   /* int32 -32768 */
    g_win[PROJ_BEST_SCORE / 4u] = 0x11111111u;
    g_win[(PROJ_BEST_SCORE + PS_FIX_64_HI) / 4u] = 0x22222222u;
    g_win[(PROJ_BEST_SCORE + 0x8u) / 4u] = 1u;   /* 64 位结果 VLD 在 +0x8 */
    g_win[PROJ_MAX_ABS_ACC / 4u] = 0xDEADBEEFu;
    g_win[(PROJ_MAX_ABS_ACC + PS_FIX_64_HI) / 4u] = 0x00000001u;
    g_win[(PROJ_MAX_ABS_ACC + 0x8u) / 4u] = 1u;

    job = make_proj_job();
    rc = ps_fix_run_projection(&p, &job, 16u, &res);
    CHECK(rc == PS_FIX_OK, "proj 作业读回返回 OK");
    CHECK(res.best_id == 121u && res.second_id == 99u, "best_id/second_id 读回正确");
    CHECK(res.margin_q16 == (int32_t)0xFFFF8000, "margin_q16 读回为负值");
    CHECK(res.best_score == 0x2222222211111111ull, "best_score 64 位拼接正确");
    CHECK(res.max_abs_acc == (int64_t)0x00000001DEADBEEFull, "max_abs_acc 64 位拼接正确");
}

static void test_sequence_order(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    ps_fix_proj_job_t jp;
    ps_fix_lrf_job_t jf;
    ps_fix_lra_job_t ja;
    ps_fix_pdhg_job_t jd;
    ps_fix_sequence_t seq;
    ps_fix_sequence_result_t res;
    int rc;
    int n = 0;
    const int *order = ps_fix_core_order(&n);

    CHECK(n == 4 && order[0] == PS_FIX_CORE_ID_PROJ && order[3] == PS_FIX_CORE_ID_PDHG,
          "四核顺序 = proj/fwd/adj/pdhg");

    ps_fix_window_open_shadow(&w, g_win, WIN_BYTES);
    ps_fix_pipeline_open(&p, &w);
    g_ntrace = 0;
    p.trace = on_trace;

    jp = make_proj_job();
    jf = make_fwd_job();
    ja = make_adj_job();
    jd = make_pdhg_job();
    memset(&seq, 0, sizeof(seq));
    seq.proj = &jp; seq.fwd = &jf; seq.adj = &ja; seq.pdhg = &jd;

    rc = ps_fix_run_sequence(&p, &seq, 16u, &res);
    CHECK(rc == PS_FIX_OK && res.stages_ok == 4 && res.aborted_at == -1,
          "四核串联 4/4 成功");
    CHECK(g_ntrace == 4 &&
          g_trace[0] == PS_FIX_CORE_ID_PROJ && g_trace[1] == PS_FIX_CORE_ID_FWD &&
          g_trace[2] == PS_FIX_CORE_ID_ADJ  && g_trace[3] == PS_FIX_CORE_ID_PDHG,
          "阶段追踪顺序 = proj/fwd/adj/pdhg");

    /* 三个低秩/重建核的标量落点（LRF 与 LRA 布局相同，只查一份） */
    CHECK(rd(0x20000u + LRF_K) == jf.K && rd(0x20000u + LRF_M) == jf.M,
          "lr_fwd K/M 标量正确");
    CHECK(rd(0x20000u + LRF_U_EXP) == (uint32_t)jf.gain.u_exp,
          "lr_fwd u_exp 标量正确");
    CHECK(rd(0x40000u + LRA_N) == ja.N, "lr_adj N 标量正确");
    CHECK(rd(0x60000u + PDHG_N_ITER) == jd.params.n_iter &&
          rd(0x60000u + PDHG_LAMBDA_Q16) == (uint32_t)jd.params.lambda_q16,
          "pdhg n_iter/lambda 标量正确");
    CHECK(rd(0x60000u + PS_FIX_REG_CTRL) ==
          (PS_FIX_CTRL_AP_START | PS_FIX_CTRL_AP_DONE),
          "pdhg CTRL = ap_start|ap_done（影子）");

    /* 缺一个核的合法串联：只跑 fwd+adj，stages_ok 应为 2 */
    memset(&seq, 0, sizeof(seq));
    seq.fwd = &jf; seq.adj = &ja;
    g_ntrace = 0;
    rc = ps_fix_run_sequence(&p, &seq, 16u, &res);
    CHECK(rc == PS_FIX_OK && res.stages_ok == 2, "部分串联（fwd+adj）2/2 成功");

    /* 全空串联应报参数错 */
    memset(&seq, 0, sizeof(seq));
    rc = ps_fix_run_sequence(&p, &seq, 16u, &res);
    CHECK(rc == PS_FIX_DRV_ERR_ARGS, "空串联被拒");
}

static void test_gates(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    ps_fix_proj_job_t job;
    ps_fix_proj_result_t res;
    int rc;

    ps_fix_window_open_shadow(&w, g_win, WIN_BYTES);
    ps_fix_pipeline_open(&p, &w);

    job = make_proj_job();
    job.u_codes = job.u_codes + 1ull;     /* 破坏 4096 对齐 */
    rc = ps_fix_run_projection(&p, &job, 16u, &res);
    CHECK(rc == PS_FIX_DRV_ERR_ALIGN, "未对齐 m_axi 基址返回 ERR_ALIGN");

    job = make_proj_job();
    job.meas = 0ull;
    rc = ps_fix_run_projection(&p, &job, 16u, &res);
    CHECK(rc == PS_FIX_DRV_ERR_ARGS, "空缓冲基址返回 ERR_ARGS");

    rc = ps_fix_run_projection(&p, &job, 16u, NULL);
    CHECK(rc == PS_FIX_DRV_ERR_ARGS, "空结果指针返回 ERR_ARGS");
}

static void test_model_view(void)
{
    ps_psm_header_t h;
    ps_fix_model_view_t v;
    int rc;

    memset(&h, 0, sizeof(h));
    h.magic[0] = (char)PS_PSM_MAGIC0;
    h.magic[1] = (char)PS_PSM_MAGIC1;
    h.magic[2] = (char)PS_PSM_MAGIC2;
    h.magic[3] = (char)PS_PSM_MAGIC3;
    h.version = (uint16_t)PS_PSM_VERSION;
    h.header_bytes = (uint16_t)PS_PSM_HEADER_BYTES;
    h.meas_h = 95; h.meas_w = 96; h.hidden_h = 18; h.hidden_w = 24;
    h.candidates = 243; h.rank_max = 64; h.rank_default = 64; h.loc_rank = 8;

    rc = ps_fix_model_view(&h, 0x8000000000ull, 121u, &v);
    CHECK(rc == PS_FIX_OK, "模型视图计算成功");
    CHECK(v.record_bytes == 1236992ull, "单候选记录 = 1,236,992 B（来自 psm_format.h）");
    CHECK(v.cand_phys == 0x8000000000ull + 64ull + 121ull * 1236992ull,
          "候选 121 记录基址 = 头部 + 索引 * 记录字节数");
    CHECK(v.U_phys - v.cand_phys == (uint64_t)ps_psm_u_offset(h.rank_max, 9120ull),
          "U 偏移取自 ps_psm_u_offset（非硬编码）");
    CHECK(v.S_phys - v.cand_phys == (uint64_t)ps_psm_s_offset(h.rank_max, 9120ull),
          "S 偏移取自 ps_psm_s_offset");
    CHECK(v.V_phys - v.cand_phys == (uint64_t)ps_psm_v_offset(h.rank_max, 9120ull),
          "V 偏移取自 ps_psm_v_offset");
    CHECK(v.mask_phys - v.cand_phys ==
          (uint64_t)ps_psm_mask_offset(h.rank_max, 9120ull, 432ull),
          "mask 偏移取自 ps_psm_mask_offset");
    CHECK(v.u_bytes == 64ull * 9120ull * 2ull, "U 数组字节数 = rank_max * M * 2");

#ifdef PS_PSM_OFF_CAND_S_MAX
    /* 格式含 s_max（2026-09-29 起）：视图应给出它的偏移，且不写死数字 */
    CHECK(v.has_s_max == 1 &&
          v.s_max_phys - v.cand_phys == (uint64_t)PS_PSM_OFF_CAND_S_MAX,
          "s_max 字段偏移取自 psm_format.h");
#else
    CHECK(v.has_s_max == 0, "格式不含 s_max 时视图如实置 has_s_max = 0");
#endif

    /* 换 rank_max，偏移/字节数应随之变（证明不是写死的） */
    h.rank_max = 96; h.rank_default = 96;
    rc = ps_fix_model_view(&h, 0x8000000000ull, 0u, &v);
    CHECK(rc == PS_FIX_OK && v.u_bytes == 96ull * 9120ull * 2ull,
          "rank_max 改 96 后 U 字节数随之改变");

    /* 候选越界应被拒 */
    h.candidates = 3;
    rc = ps_fix_model_view(&h, 0x8000000000ull, 5u, &v);
    CHECK(rc == PS_FIX_DRV_ERR_ARGS, "候选索引越界被拒");
}

/* ---------------------------------------------------------------- S 幅度折算 */

static void put_f32(uint8_t *base, uint32_t off, float v)
{
    memcpy(base + off, &v, sizeof(v));
}

/* 独立于驱动实现的一份偶数舍入，用来算「manifest 侧期望值」。 */
static long long t_round_half_even(double x)
{
    long long t = (long long)x;                 /* 本用例 x > 0，强转即向下取整 */
    double f = x - (double)t;
    if (f > 0.5) { ++t; }
    else if (f == 0.5 && (t & 1LL)) { ++t; }
    return t;
}

static void test_s_max_fold(void)
{
    /* B 档四个候选的 manifest 值：s_scale 恒为 2^-14，s_max 逐候选不同。
     * 期望标量由 manifest 侧独立算出：gain = round_half_even(s_scale*s_max*2^16)
     * （s_scale = 2^-14 时等于 round_half_even(4*s_max)），非 2 的幂 -> exp 无效。 */
    static const struct {
        const char *name;
        float       s_scale;
        float       s_max;
        int32_t     exp_gain;
        int32_t     exp_exp;
        int         exact;
    } cases[] = {
        {"B 候选 0",     6.103515625e-05f, 66.08143873125681f, 264, PS_FIX_GAIN_EXP_INVALID, 0},
        {"B 候选 60",    6.103515625e-05f, 68.46549453358988f, 274, PS_FIX_GAIN_EXP_INVALID, 0},
        {"B 候选 121",   6.103515625e-05f, 71.88351073470406f, 288, PS_FIX_GAIN_EXP_INVALID, 0},
        {"B 候选 242",   6.103515625e-05f, 79.23782163955153f, 317, PS_FIX_GAIN_EXP_INVALID, 0},
        {"2 的幂 m=1",   6.103515625e-05f, 16384.0f,             0, 0,  1},
        {"2 的幂 m=2^-4",6.103515625e-05f, 1024.0f,              0, -4, 1},
        {"2 的幂 m=2^12",1.0f,             4096.0f,              0, 12, 1},
    };
    const unsigned n = (unsigned)(sizeof(cases) / sizeof(cases[0]));
    unsigned i;

    CHECK((uint32_t)offsetof(ps_psm_candidate_hdr_t, u_scale) == 8u &&
          (uint32_t)offsetof(ps_psm_candidate_hdr_t, s_scale) == 12u &&
          (uint32_t)offsetof(ps_psm_candidate_hdr_t, v_scale) == 16u &&
          (uint32_t)PS_PSM_OFF_CAND_S_MAX == 20u,
          "候选头偏移 u/s/v/s_max = 8/12/16/20（psm_format.h）");

    for (i = 0; i < n; ++i) {
        uint8_t rec[64];
        ps_fix_s_fold_t f;
        long long want_gain;
        int rc;

        memset(rec, 0, sizeof(rec));
        put_f32(rec, 8,  7.62939453125e-06f);   /* u_scale = 2^-17 */
        put_f32(rec, 12, cases[i].s_scale);
        put_f32(rec, 16, 1.52587890625e-05f);   /* v_scale = 2^-16 */
        put_f32(rec, 20, cases[i].s_max);

        /* 从候选头（psm_format.h 布局）读 s_scale/s_max 后折算 */
        rc = ps_fix_fold_candidate_s(rec, &f);
        if (rc != PS_FIX_OK) {
            CHECK(0, cases[i].name);
            continue;
        }
        /* manifest 侧期望值：独立算一遍 */
        want_gain = t_round_half_even((double)cases[i].s_scale *
                                      (double)cases[i].s_max * 65536.0);
        {
            char msg[96];
            snprintf(msg, sizeof(msg), "%s: s_gain_q16/s_exp 与 manifest 期望一致",
                     cases[i].name);
            if (cases[i].exact) {
                /* 幂次支路下 gain 字段被核忽略，折算按约定填 0；判 exp 与 exact。 */
                CHECK(f.s_gain_q16 == 0 && f.s_exp == cases[i].exp_exp &&
                      f.exact == 1, msg);
            } else {
                CHECK(f.s_gain_q16 == cases[i].exp_gain &&
                      f.s_exp == cases[i].exp_exp &&
                      (long long)f.s_gain_q16 == want_gain && f.exact == 0, msg);
            }
        }
        if (cases[i].exact) {
            char msg[96];
            double m = (double)cases[i].s_scale * (double)cases[i].s_max;
            snprintf(msg, sizeof(msg), "%s: 2 的幂支路 rel_err = 0 且乘数正确",
                     cases[i].name);
            CHECK(f.rel_err == 0.0f && (double)f.multiplier == m, msg);
        } else {
            char msg[96];
            double m = (double)cases[i].s_scale * (double)cases[i].s_max;
            double bound = 0.5 / (m * 65536.0);           /* 半 LSB 相对上界 */
            snprintf(msg, sizeof(msg), "%s: Q16.16 支路 rel_err=%.2e <= 半 LSB %.2e",
                     cases[i].name, (double)f.rel_err, bound);
            CHECK(f.rel_err > 0.0f && (double)f.rel_err <= bound + 1e-9, msg);
        }
    }

    /* 帧量程比 c = s_f/s_d = 1/32（B 档重建量程，见 X20）：乘数 = s_scale*s_max/32，
     * 走 Q16.16 支路，gain = round_half_even(4*s_max/32) = round(s_max/8)。 */
    {
        double m = 6.103515625e-05 * 71.88351073470406 / 32.0;
        ps_fix_s_fold_t f;
        CHECK(ps_fix_fold_multiplier(m, &f) == PS_FIX_OK && f.s_gain_q16 == 9 &&
              f.s_exp == PS_FIX_GAIN_EXP_INVALID && f.exact == 0,
              "帧量程 c=1/32：乘数 = s_scale*s_max/32 -> s_gain_q16=9, s_exp=INVALID");
    }

    /* 入参门禁 */
    {
        ps_fix_s_fold_t f;
        CHECK(ps_fix_fold_s_scale(0.0f, 71.0f, &f) == PS_FIX_DRV_ERR_ARGS &&
              ps_fix_fold_s_scale(2.0f, 0.0f, &f) == PS_FIX_DRV_ERR_ARGS &&
              ps_fix_fold_candidate_s(NULL, &f) == PS_FIX_DRV_ERR_ARGS &&
              ps_fix_fold_multiplier(0.0, &f) == PS_FIX_DRV_ERR_ARGS,
              "非正 s_scale/s_max/乘数 与空指针被拒");
    }
}

/* A1：增益入口不得静默退回。
 * 判据：幂次模型正常通过，且输出与改动前逐位相同；非 2 的幂 / 过小（denormal）
 * 的 u_scale 或 v_scale 被拒且给出明确错误码 PS_FIX_DRV_ERR_GAIN_EXP。 */
static void test_gain_entry_reject(void)
{
    uint8_t rec[64];
    ps_fix_gain_t g;
    int rc;

    printf("\n---- A1. 增益入口：非 2 的幂/过小 scale 必须拒绝 ----\n");

    /* 合法：B 档候选 121 的真实尺度（u=2^-17 s=2^-14 v=2^-16，s_max=71.8835）。 */
    memset(rec, 0, sizeof(rec));
    put_f32(rec, 8,  7.62939453125e-06f);   /* u_scale = 2^-17 */
    put_f32(rec, 12, 6.103515625e-05f);     /* s_scale = 2^-14 */
    put_f32(rec, 16, 1.52587890625e-05f);   /* v_scale = 2^-16 */
    put_f32(rec, 20, 71.88351073470406f);   /* manifest s_max */

    CHECK(ps_fix_check_candidate_gain_scales(rec) == PS_FIX_OK,
          "幂次模型的 u/v scale 通过入口校验");
    memset(&g, 0xA5, sizeof(g));
    rc = ps_fix_gain_from_candidate(rec, &g);
    CHECK(rc == PS_FIX_OK && g.u_exp == -17 && g.v_exp == -16 &&
          g.u_gain_q16 == 0 && g.v_gain_q16 == 0 &&
          g.s_gain_q16 == 288 && g.s_exp == PS_FIX_GAIN_EXP_INVALID,
          "合法路径输出逐位不变（u_exp=-17 v_exp=-16 s=288/INVALID）");

    /* 非 2 的幂：u_scale = 3.0e-05（旧的连续 scale 的典型量级）。 */
    memset(rec, 0, sizeof(rec));
    put_f32(rec, 8,  3.0e-05f);
    put_f32(rec, 12, 6.103515625e-05f);
    put_f32(rec, 16, 1.52587890625e-05f);
    put_f32(rec, 20, 71.88351073470406f);
    CHECK(ps_fix_check_candidate_gain_scales(rec) == PS_FIX_DRV_ERR_GAIN_EXP,
          "非 2 的幂 u_scale 被入口拒绝（明确错误码）");
    memset(&g, 0xA5, sizeof(g));
    rc = ps_fix_gain_from_candidate(rec, &g);
    CHECK(rc == PS_FIX_DRV_ERR_GAIN_EXP &&
          g.u_exp == PS_FIX_GAIN_EXP_INVALID &&
          g.v_exp == PS_FIX_GAIN_EXP_INVALID &&
          g.s_exp == PS_FIX_GAIN_EXP_INVALID,
          "拒绝后不留半填状态（三个 exp 都置 INVALID）");

    /* 非 2 的幂：v_scale。 */
    memset(rec, 0, sizeof(rec));
    put_f32(rec, 8,  7.62939453125e-06f);
    put_f32(rec, 12, 6.103515625e-05f);
    put_f32(rec, 16, 3.0e-05f);
    put_f32(rec, 20, 71.88351073470406f);
    CHECK(ps_fix_gain_from_candidate(rec, &g) == PS_FIX_DRV_ERR_GAIN_EXP,
          "非 2 的幂 v_scale 被入口拒绝");

    /* 过小（denormal，已无精确幂次表示）：1e-45 是最小的 float 量级。 */
    memset(rec, 0, sizeof(rec));
    put_f32(rec, 8,  1.0e-45f);
    put_f32(rec, 12, 6.103515625e-05f);
    put_f32(rec, 16, 1.52587890625e-05f);
    put_f32(rec, 20, 71.88351073470406f);
    CHECK(ps_fix_check_candidate_gain_scales(rec) == PS_FIX_DRV_ERR_GAIN_EXP,
          "过小的 u_scale（denormal）被入口拒绝");

    /* 合法：u/v 为 2 的幂但 S 乘数（s_scale*s_max）非 2 的幂 —— S 的 Q16.16 支路
     * 是设计内的合法路径，不得被 A1 的严格判据误伤。 */
    memset(rec, 0, sizeof(rec));
    put_f32(rec, 8,  7.62939453125e-06f);
    put_f32(rec, 12, 6.103515625e-05f);
    put_f32(rec, 16, 1.52587890625e-05f);
    put_f32(rec, 20, 71.88351073470406f);
    CHECK(ps_fix_gain_from_candidate(rec, &g) == PS_FIX_OK &&
          g.s_exp == PS_FIX_GAIN_EXP_INVALID,
          "S 乘数非 2 的幂仍走 Q16.16（不被 A1 误伤）");

    /* 空指针门禁 */
    CHECK(ps_fix_check_candidate_gain_scales(NULL) == PS_FIX_DRV_ERR_ARGS &&
          ps_fix_gain_from_candidate(NULL, &g) == PS_FIX_DRV_ERR_ARGS,
          "空候选头被拒");
}

static void test_s_fold_register_write(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    ps_fix_lrf_job_t jf;
    ps_fix_lra_job_t ja;
    ps_fix_pdhg_job_t jd;
    ps_fix_lr_result_t rl;
    ps_fix_pdhg_result_t rp;
    ps_fix_s_fold_t f;
    ps_fix_gain_t g;
    uint8_t rec[64];
    int rc;

    memset(rec, 0, sizeof(rec));
    put_f32(rec, 8,  7.62939453125e-06f);   /* 2^-17 */
    put_f32(rec, 12, 6.103515625e-05f);     /* 2^-14 */
    put_f32(rec, 16, 1.52587890625e-05f);   /* 2^-16 */
    put_f32(rec, 20, 71.88351073470406f);   /* B 档候选 121 的 manifest s_max */

    rc = ps_fix_fold_candidate_s(rec, &f);
    CHECK(rc == PS_FIX_OK && f.s_gain_q16 == 288 && f.s_exp == PS_FIX_GAIN_EXP_INVALID,
          "候选 121 候选头 -> s_gain_q16=288, s_exp=INVALID");

    memset(&g, 0, sizeof(g));
    rc = ps_fix_gain_from_candidate(rec, &g);
    CHECK(rc == PS_FIX_OK && g.u_exp == -17 && g.v_exp == -16 &&
          g.s_gain_q16 == 288 && g.s_exp == PS_FIX_GAIN_EXP_INVALID,
          "整组增益：u_exp=-17 v_exp=-16 s=Q16.16 支路(288)");

    ps_fix_window_open_shadow(&w, g_win, WIN_BYTES);
    ps_fix_pipeline_open(&p, &w);

    jf = make_fwd_job();  jf.gain = g;
    ja = make_adj_job();  ja.gain = g;
    jd = make_pdhg_job(); jd.gain = g;

    rc = ps_fix_run_forward(&p, &jf, 16u, &rl);
    CHECK(rc == PS_FIX_OK, "lr_fwd（带折算增益）作业返回 OK");
    CHECK(rd(0x20000u + LRF_S_GAIN_Q16) == (uint32_t)f.s_gain_q16 &&
          rd(0x20000u + LRF_S_EXP) == (uint32_t)f.s_exp,
          "lr_fwd S 两件写进 0x038/0x050");
    CHECK(rd(0x20000u + LRF_U_EXP) == (uint32_t)(-17) &&
          rd(0x20000u + LRF_V_EXP) == (uint32_t)(-16),
          "本折算不动 U/V（u_exp=-17 v_exp=-16 保持）");

    rc = ps_fix_run_adjoint(&p, &ja, 16u, &rl);
    CHECK(rc == PS_FIX_OK &&
          rd(0x40000u + LRA_S_GAIN_Q16) == (uint32_t)f.s_gain_q16 &&
          rd(0x40000u + LRA_S_EXP) == (uint32_t)f.s_exp,
          "lr_adj S 两件写进 0x038/0x050");

    rc = ps_fix_run_pdhg(&p, &jd, 16u, &rp);
    CHECK(rc == PS_FIX_OK &&
          rd(0x60000u + PDHG_S_GAIN_Q16) == (uint32_t)f.s_gain_q16 &&
          rd(0x60000u + PDHG_S_EXP) == (uint32_t)f.s_exp,
          "pdhg S 两件写进 0x038/0x050");
}

/* 若有真实的 model_b.psm，则直接读候选 121 的候选头（+12/+20）再折算，作为
 * 「从 psm 读」这一层的端到端证据。文件不在（.psm 不入库）时明确跳过，不伪造。 */
static void test_s_max_fold_real_psm(void)
{
    static const char *paths[] = {
        "../../sim/models/model_b.psm",
        "sim/models/model_b.psm",
        "../sim/models/model_b.psm",
    };
    FILE *fp = NULL;
    const char *used = NULL;
    unsigned i;
    ps_psm_header_t h;
    long long off;
    uint8_t hdr[PS_PSM_HEADER_BYTES];
    uint8_t rec[24];
    ps_fix_s_fold_t f;
    float s_max;

    for (i = 0; i < sizeof(paths) / sizeof(paths[0]); ++i) {
        fp = fopen(paths[i], "rb");
        if (fp) { used = paths[i]; break; }
    }
    if (!fp) {
        printf("  NOTE  未找到 model_b.psm（.psm 不入库），跳过真实文件读取\n");
        return;
    }
    if (fread(hdr, 1, PS_PSM_HEADER_BYTES, fp) != PS_PSM_HEADER_BYTES) {
        printf("  NOTE  %s 头部读不满，跳过\n", used);
        fclose(fp);
        return;
    }
    memcpy(&h, hdr, sizeof(h));
    CHECK(h.candidates > 121u, "真实 psm 候选数 > 121");
    off = (long long)PS_PSM_HEADER_BYTES + 121ll * (long long)ps_psm_record_bytes_hdr(&h);
    if (fseek(fp, (long)off, SEEK_SET) != 0 || fread(rec, 1, sizeof(rec), fp) != sizeof(rec)) {
        printf("  NOTE  %s 候选 121 记录头读不到，跳过\n", used);
        fclose(fp);
        return;
    }
    fclose(fp);

    CHECK(ps_fix_fold_candidate_s(rec, &f) == PS_FIX_OK,
          "真实 model_b.psm 候选 121 候选头折算成功");
    s_max = 0.0f;
    memcpy(&s_max, rec + PS_PSM_OFF_CAND_S_MAX, sizeof(s_max));
    {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "psm 读 s_max=%.6f -> s_gain_q16=%d s_exp=0x%08X（manifest 期望 288）",
                 (double)s_max, (int)f.s_gain_q16, (unsigned)f.s_exp);
        CHECK(f.s_gain_q16 == 288 && f.s_exp == PS_FIX_GAIN_EXP_INVALID &&
              s_max > 71.88f && s_max < 71.89f, msg);
    }
}

static uint64_t fake_clock(void *ctx) {uint64_t *v=(uint64_t *)ctx;*v+=10;return *v;}
static void test_monotonic_timeout(void) {
    uint32_t words[8]={0}; ps_fix_core_t core; uint64_t tick=0;
    memset(&core,0,sizeof(core));core.ctrl=words;
    core.clock_ns=fake_clock;core.clock_ctx=&tick;core.timeout_ns=30;
    CHECK(ps_fix_start_and_wait(&core,UINT32_MAX)==PS_FIX_ERR_TIMEOUT && tick==40,
          "wall time deadline overrides huge poll count");
    CHECK(ps_fix_start_and_wait(NULL,1)==PS_FIX_ERR_ARG,"null core rejected");
}

int main(void)
{
    printf("==== fix_core 调用层主机自检（影子窗口，无板卡）====\n");
    test_monotonic_timeout();
    test_window_and_open();
    test_projection_regmap();
    test_base_write_order();
    test_result_vld_readback();
    test_sequence_order();
    test_gates();
    test_model_view();
    test_s_max_fold();
    test_gain_entry_reject();
    test_s_fold_register_write();
    test_s_max_fold_real_psm();
    printf("------------------------------------------------------------\n");
    printf("通过 %d ，失败 %d\n", g_pass, g_fail);
    if (g_fail != 0) {
        printf("结果：FAIL\n");
        return 1;
    }
    printf("结果：PASS（仅覆盖寄存器协议；上板时序等板卡）\n");
    return 0;
}
