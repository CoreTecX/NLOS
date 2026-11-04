/* ============================================================================
 * PenumbraSight · 板端顶层应用的主机自检
 * src/software/test_board_top.c
 *
 * 可立即运行（不依赖板卡、不依赖 Xilinx 头文件）：
 *     make -C src/software test-board-top
 * 或在仓库根：make test-board-top
 * 退出码非零表示失败。
 *
 * 覆盖（主机可判）
 * ---------------------------------------------------------------------------
 *   A. DDR 账本：4096 对齐、顺序分配不重叠、耗尽返回 NOSPACE、主机指针换算；
 *   B. 缓冲布局：各项对齐后之和 = total、u_codes 随候选数变、模型体走
 *      psm_format.h 的记录字节数（不写死）；
 *   C. 寄存器初始化清单：顺序约束（基址 HI 先于 LO、标量在基址后、ap_start 收尾）、
 *      首步为版本读回、逐条地址/偏移/值与 ps_fix_regs.h 的宏一致、打印不越界；
 *   D. 清单 ↔ 驱动 的交叉比对：清单回放后的寄存器内容，与 fix_core_driver 在
 *      影子窗口实际写入的内容**逐字相同**（唯一差异是 CTRL 的 ap_done 影子替身）；
 *   E. 接口版本：读回一致 / 读回不符 / 地址不在窗口内（未实现）三种结果；
 *   F. 最小用例：单核写基址 -> 写标量 -> ap_start -> ap_done -> 读回；
 *   G. 四核串联：清单回放 + ps_fix_run_sequence 4/4；
 *   H. 帧缓冲：定标端点、饱和、通道序、灰屏、入参门禁；
 *   I. .psf 装载：入库 fixture 读进 DDR 账本并校验（crc32/sum/sumsq/min/max）。
 *
 * 不覆盖（等板卡）
 * ---------------------------------------------------------------------------
 *   真实 AXI 读写与时序、ap_done 到达时间、DMA 搬运、SD 驱动、DP 扫描输出、
 *   只读版本寄存器的板上读回（该寄存器当前不存在）。
 * ==========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_top.h"
#include "board_regseq.h"
#include "board_fb.h"

/* 窗口开到 1 MiB：覆盖到预留的版本寄存器（0xA00A0000）。 */
#define WIN_BYTES (1024u * 1024u)
#define WIN_WORDS (WIN_BYTES / 4u)
static uint32_t g_winA[WIN_WORDS];
static uint32_t g_winB[WIN_WORDS];

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do {                                   \
        if (cond) { g_pass++; printf("  PASS  %s\n", (msg)); }  \
        else { g_fail++; printf("  FAIL  %s\n", (msg)); }       \
    } while (0)

static uint64_t fake_addr(unsigned i)
{
    return 0x000100000000ull + (uint64_t)i * 0x1000ull;
}

static void fill_gain(ps_fix_gain_t *g)
{
    g->u_gain_q16 = 0x1A2B; g->s_gain_q16 = 288; g->v_gain_q16 = 0x3C4D;
    g->u_exp = -17; g->s_exp = PS_FIX_GAIN_EXP_INVALID; g->v_exp = -16;
}

static ps_fix_proj_job_t make_proj_job(void)
{
    ps_fix_proj_job_t j;
    memset(&j, 0, sizeof(j));
    j.meas = fake_addr(1); j.u_codes = fake_addr(2); j.gain_q16 = fake_addr(3);
    j.score_out = fake_addr(4); j.score_ch = fake_addr(5);
    j.P = 243; j.K = 8; j.M = 9120; j.nx = 9; j.ny = 9; j.nz = 3;
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
    j.K = 8; j.M = 9120; j.N = 432; fill_gain(&j.gain);
    return j;
}
static ps_fix_lra_job_t make_adj_job(void)
{
    ps_fix_lra_job_t j;
    memset(&j, 0, sizeof(j));
    j.U = fake_addr(10); j.S = fake_addr(11); j.V = fake_addr(12);
    j.r = fake_addr(20); j.g = fake_addr(21);
    j.wk_w = fake_addr(22); j.wk_ws = fake_addr(23);
    j.K = 8; j.M = 9120; j.N = 432; fill_gain(&j.gain);
    return j;
}
static ps_fix_pdhg_job_t make_pdhg_job(void)
{
    ps_fix_pdhg_job_t j;
    memset(&j, 0, sizeof(j));
    j.U = fake_addr(10); j.S = fake_addr(11); j.V = fake_addr(12);
    j.d3 = fake_addr(30); j.f3 = fake_addr(31);
    j.scratch.wk_f_prev = fake_addr(32); j.scratch.wk_fbar = fake_addr(33);
    j.scratch.wk_q = fake_addr(34); j.scratch.wk_grad = fake_addr(35);
    j.scratch.wk_divq = fake_addr(36); j.scratch.wk_y = fake_addr(37);
    j.scratch.wk_r = fake_addr(38); j.scratch.wk_btr = fake_addr(39);
    j.scratch.wk_tmp = fake_addr(40);
    j.out.out_n_used = fake_addr(41); j.out.out_early_stopped = fake_addr(42);
    j.out.out_overflow_ch = fake_addr(43); j.out.out_rho_q16 = fake_addr(44);
    j.out.out_res_sq = fake_addr(45); j.out.out_d_sq = fake_addr(46);
    j.out.out_mac_count = fake_addr(47);
    j.K = 8; j.M = 9120; j.N = 432; fill_gain(&j.gain);
    j.params.n_rows = 18; j.params.n_cols = 24; j.params.n_iter = 1024;
    j.params.min_iters = 8; j.params.residual_every = 4; j.params.tv_mode = 0;
    j.params.tau_q16 = 18053; j.params.sigma_q16 = 18053;
    j.params.lambda_q16 = 66; j.params.theta_q16 = 65536; j.params.rho_stop_q8 = 0;
    return j;
}

/* ---------------------------------------------------------------- A. 账本 */

static void test_arena(void)
{
    static uint8_t ddr[1u * 1024u * 1024u];
    ps_board_arena_t a;
    uint64_t p1 = 0ull, p2 = 0ull, p3 = 0ull;
    void *h1;

    printf("\n---- A. DDR 账本 ----\n");
    CHECK(ps_board_arena_init(&a, 0x000100000001ull, sizeof(ddr), ddr) == PS_BOARD_ERR_ALIGN,
          "未对齐基址被拒");
    CHECK(ps_board_arena_init(&a, 0x000100000000ull, 0ull, ddr) == PS_BOARD_ERR_ARGS,
          "size=0 被拒");

    CHECK(ps_board_arena_init(&a, 0x000100000000ull, sizeof(ddr), ddr) == PS_BOARD_OK,
          "账本初始化成功");
    CHECK(ps_board_arena_used(&a) == 0ull, "初始已用为 0");

    CHECK(ps_board_arena_alloc(&a, 1000u, &p1) == PS_BOARD_OK &&
          (p1 % PS_BOARD_ALIGN) == 0ull, "首次分配 4096 对齐");
    CHECK(ps_board_arena_alloc(&a, 5000u, &p2) == PS_BOARD_OK && p2 > p1 &&
          (p2 - p1) == ps_board_align_up(1000u, PS_BOARD_ALIGN), "第二次分配紧随其后且不重叠");
    CHECK(ps_board_arena_alloc(&a, 100u, &p3) == PS_BOARD_OK && (p3 % PS_BOARD_ALIGN) == 0ull,
          "第三次分配仍对齐");
    CHECK(ps_board_arena_used(&a) == ps_board_align_up(1000u, PS_BOARD_ALIGN) +
                                      ps_board_align_up(5000u, PS_BOARD_ALIGN) +
                                      ps_board_align_up(100u, PS_BOARD_ALIGN),
          "已用累计 = 各次对齐后之和");

    h1 = ps_board_arena_ptr(&a, p1);
    CHECK(h1 == (void *)(ddr + (size_t)(p1 - a.base)), "主机指针 = host + (phys - base)");
    CHECK(ps_board_arena_ptr(&a, a.base + a.size) == NULL, "越出 used 的地址取指针返回 NULL");

    {
        uint64_t big = 0ull;
        CHECK(ps_board_arena_alloc(&a, sizeof(ddr), &big) == PS_BOARD_ERR_NOSPACE,
              "装不下返回 NOSPACE");
    }
}

/* ---------------------------------------------------------------- B. 布局 */

static void test_layout(void)
{
    ps_board_layout_cfg_t cfg;
    ps_board_layout_t L, L2;

    printf("\n---- B. 缓冲布局 ----\n");
    ps_board_layout_default_b(&cfg);
    CHECK(cfg.meas_h == 95u && cfg.meas_w == 96u && cfg.candidates == 243u &&
          cfg.loc_rank == 8u && cfg.rank_max == 64u && cfg.frame_side == 1920u,
          "B 档默认几何（1920 帧 / 95x96 / 243 候选 / K8 / R64）");

    CHECK(ps_board_layout_compute(&cfg, &L) == PS_BOARD_OK, "布局计算成功");
    CHECK((L.total % PS_BOARD_ALIGN) == 0ull, "总量是 4096 的整数倍");
    {
        uint64_t meas_len = 95ull * 96ull;
        uint64_t want_u = ps_board_align_up(243ull * 8ull * meas_len * 2ull, PS_BOARD_ALIGN);
        uint64_t sum = L.raw_frame + L.rtl_beats + L.meas + L.u_codes + L.gain_q16 +
                       L.score_out + L.score_ch + L.model + L.fb;
        CHECK(L.u_codes == want_u, "u_codes 字节数 = align(候选*K*M*2)");
        CHECK(L.total == sum, "total = 各项对齐后之和");
        CHECK(L.raw_frame == ps_board_align_up(1920ull * 1920ull * 2ull, PS_BOARD_ALIGN),
              "raw_frame = align(1920*1920*2)");
    }
    /* 改候选数 -> u_codes 与 total 随之变（证明不是写死） */
    cfg.candidates = 121u;
    CHECK(ps_board_layout_compute(&cfg, &L2) == PS_BOARD_OK && L2.u_codes < L.u_codes &&
          L2.total < L.total, "候选数变小 -> u_codes 与 total 变小");

    cfg.meas_h = 0u;
    CHECK(ps_board_layout_compute(&cfg, &L) == PS_BOARD_ERR_ARGS, "零尺寸被拒");
}

/* ---------------------------------------------------------------- C. 清单 */

/* 清单里 addr 处的步骤序号；找不到返回 -1。 */
static int find_addr(const ps_board_regseq_t *s, uint64_t addr)
{
    int i;
    for (i = 0; i < s->n; ++i) { if (s->step[i].addr == addr) { return i; } }
    return -1;
}

static void test_regseq_build_and_order(void)
{
    ps_fix_proj_job_t jp = make_proj_job();
    ps_fix_lrf_job_t  jf = make_fwd_job();
    ps_fix_lra_job_t  ja = make_adj_job();
    ps_fix_pdhg_job_t jd = make_pdhg_job();
    ps_board_regseq_cfg_t cfg;
    ps_board_regseq_t s;
    int i;

    printf("\n---- C. 寄存器初始化清单 ----\n");
    memset(&cfg, 0, sizeof(cfg));
    CHECK(ps_board_regseq_build(&s, &cfg) == -1, "全空配置被拒");

    cfg.proj = &jp; cfg.fwd = &jf; cfg.adj = &ja; cfg.pdhg = &jd;
    cfg.include_version = 1; cfg.if_version = 1u;
    CHECK(ps_board_regseq_build(&s, &cfg) == 0 && s.overflow == 0 && s.n > 100,
          "四核清单生成，未截断");

    /* 第 1 步：版本读回 */
    CHECK(s.step[0].kind == PS_REGSEQ_READ &&
          s.step[0].addr == (uint64_t)PS_BOARD_REGSEQ_IFVER_ADDR &&
          s.step[0].value == 1u, "第 1 步 = PS_FIX_IF_VERSION 读回（期望 1）");

    /* 顺序约束：每个 64 位基址 HI 步紧邻其后是 LO 步，且 LO 地址 = HI 地址 - 4 */
    {
        int ok = 1;
        int n_hi = 0;
        for (i = 0; i < s.n - 1; ++i) {
            const char *nm = s.step[i].name ? s.step[i].name : "";
            size_t len = strlen(nm);
            if (len >= 3 && strcmp(nm + len - 3, ".HI") == 0) {
                ++n_hi;
                if (!(s.step[i + 1].addr == s.step[i].addr - 4ull)) { ok = 0; break; }
                if (s.step[i].space != PS_REGSEQ_SP_CTRL_R) { ok = 0; break; }
            }
        }
        CHECK(ok && n_hi == 40, "40 个 64 位基址：HI 步在前、LO 步紧随其后");
    }

    /* 每核收尾：ap_start 写 + ap_done 轮询 */
    CHECK(s.step[s.n - 1].kind == PS_REGSEQ_POLL &&
          s.step[s.n - 1].value == PS_FIX_CTRL_AP_DONE &&
          s.step[s.n - 2].kind == PS_REGSEQ_WRITE &&
          s.step[s.n - 2].value == PS_FIX_CTRL_AP_START,
          "末两步 = CTRL.ap_start 写 + ap_done 轮询");

    /* 逐条地址/偏移与 ps_fix_regs.h 的宏一致 */
    {
        int i_hi = find_addr(&s, (uint64_t)PS_FIX_CORE_PROJ_CTRL_R + PROJ_BASE_MEAS + PS_FIX_64_HI);
        int i_lo = find_addr(&s, (uint64_t)PS_FIX_CORE_PROJ_CTRL_R + PROJ_BASE_MEAS + PS_FIX_64_LO);
        int i_p  = find_addr(&s, (uint64_t)PS_FIX_CORE_PROJ_CTRL + PROJ_P);
        int i_mc = find_addr(&s, (uint64_t)PS_FIX_CORE_PDHG_CTRL_R + PDHG_BASE_OUT_MAC_COUNT + PS_FIX_64_LO);
        CHECK(i_hi >= 0 && i_lo >= 0 && i_p >= 0 && i_mc >= 0, "关键步骤在清单里");
        CHECK(i_hi < 0 || s.step[i_hi].value == (uint32_t)(jp.meas >> 32),
              "PROJ_BASE_MEAS.HI 值与作业一致");
        CHECK(i_lo < 0 || s.step[i_lo].value == (uint32_t)(jp.meas & 0xFFFFFFFFu),
              "PROJ_BASE_MEAS.LO 值与作业一致");
        CHECK(i_p < 0 || s.step[i_p].value == jp.P, "PROJ_P 写在 0xA0000000 + 0x018");
        CHECK(i_mc < 0 || s.step[i_mc].value == (uint32_t)(jd.out.out_mac_count & 0xFFFFFFFFu),
              "PDHG out_mac_count 基址写进 ctrl_r");
    }

    /* 打印不越界、含关键名字 */
    {
        static char buf[16384];
        int n = ps_board_regseq_print(&s, buf, sizeof(buf));
        CHECK(n > 0 && (size_t)n < sizeof(buf) &&
              strstr(buf, "PS_FIX_IF_VERSION") != NULL &&
              strstr(buf, "PROJ_BASE_MEAS.HI") != NULL &&
              strstr(buf, "CTRL.ap_start") != NULL &&
              strstr(buf, "pdhg") != NULL,
              "清单打印含版本读回、基址、ap_start、核名");
        CHECK(ps_board_regseq_print(&s, buf, 16u) == 15, "打印缓冲区过小只写 15 字节");
        CHECK(ps_board_regseq_count_kind(&s, PS_REGSEQ_WRITE) ==
              ps_board_regseq_count_kind(&s, PS_REGSEQ_WRITE) &&
              ps_board_regseq_count_kind(&s, PS_REGSEQ_POLL) == 4,
              "轮询步 4 条（每核一条）");
    }
}

/* ---------------------------------------------------------------- D. 交叉比对 */

/* 比较两块窗口，返回不同字数；first 记第一处不同的字节偏移（可为 NULL）。 */
static int count_diffs(const uint32_t *a, const uint32_t *b, int words, int *first)
{
    int i, n = 0;
    if (first) { *first = -1; }
    for (i = 0; i < words; ++i) {
        if (a[i] != b[i]) { if (n == 0 && first) { *first = i * 4; } ++n; }
    }
    return n;
}

static void test_regseq_matches_driver(void)
{
    ps_fix_window_t wA, wB;
    ps_fix_pipeline_t p;
    ps_fix_proj_job_t jp = make_proj_job();
    ps_fix_proj_result_t rp;
    ps_board_regseq_cfg_t cfg;
    ps_board_regseq_t s;
    int first = -1, nd;

    printf("\n---- D. 清单 ↔ 驱动 交叉比对 ----\n");

    /* 单核（proj）：驱动在影子窗口写入的内容，与清单回放的内容逐字相同，
     * 唯一差异是 CTRL 的 ap_done 影子替身（驱动置 start|done，清单只写 start）。 */
    CHECK(ps_fix_window_open_shadow(&wA, g_winA, WIN_BYTES) == PS_FIX_OK &&
          ps_fix_pipeline_open(&p, &wA) == PS_FIX_OK, "影子窗口 A + 管线打开");
    CHECK(ps_fix_run_projection(&p, &jp, 16u, &rp) == PS_FIX_OK, "驱动跑 proj（影子）");

    CHECK(ps_fix_window_open_shadow(&wB, g_winB, WIN_BYTES) == PS_FIX_OK, "影子窗口 B 打开");
    memset(&cfg, 0, sizeof(cfg));
    cfg.proj = &jp; cfg.include_version = 0;
    CHECK(ps_board_regseq_build(&s, &cfg) == 0, "proj 清单生成");
    CHECK(ps_board_regseq_apply_writes(&s, &wB) ==
          ps_board_regseq_count_kind(&s, PS_REGSEQ_WRITE), "清单回放：写入条数与 WRITE 步一致");
    CHECK(ps_board_min_case(&wB, &jp, 1, 16u, NULL) == PS_BOARD_ERR_ARGS,
          "min_case 结果指针为空被拒（入参门禁）");

    /* 重新回放（min_case 覆盖了 CTRL），再比对 */
    (void)ps_fix_window_open_shadow(&wB, g_winB, WIN_BYTES);
    (void)ps_board_regseq_apply_writes(&s, &wB);

    nd = count_diffs(g_winA, g_winB, (int)WIN_WORDS, &first);
    CHECK(nd == 1 && first == (int)((PS_FIX_CORE_PROJ_CTRL - PS_FIX_CORE_PROJ_CTRL) * 4),
          "单核：唯一差异落在 proj CTRL（ap_done 影子替身）");
    CHECK(g_winA[0] == (PS_FIX_CTRL_AP_START | PS_FIX_CTRL_AP_DONE) &&
          g_winB[0] == PS_FIX_CTRL_AP_START,
          "差异内容：驱动 = start|done，清单 = start");
}

static void test_regseq_matches_driver_four(void)
{
    ps_fix_window_t wA, wB;
    ps_fix_pipeline_t p;
    ps_fix_proj_job_t jp = make_proj_job();
    ps_fix_lrf_job_t  jf = make_fwd_job();
    ps_fix_lra_job_t  ja = make_adj_job();
    ps_fix_pdhg_job_t jd = make_pdhg_job();
    ps_fix_sequence_t seq;
    ps_fix_sequence_result_t res;
    ps_board_regseq_cfg_t cfg;
    ps_board_regseq_t s;
    int nd, i;
    static const uint32_t ctrl_off[4] = {
        (uint32_t)(PS_FIX_CORE_PROJ_CTRL - PS_FIX_CORE_PROJ_CTRL),
        (uint32_t)(PS_FIX_CORE_LR_FWD_CTRL - PS_FIX_CORE_PROJ_CTRL),
        (uint32_t)(PS_FIX_CORE_LR_ADJ_CTRL - PS_FIX_CORE_PROJ_CTRL),
        (uint32_t)(PS_FIX_CORE_PDHG_CTRL - PS_FIX_CORE_PROJ_CTRL),
    };

    printf("\n---- D'. 四核清单 ↔ 驱动 交叉比对 ----\n");
    CHECK(ps_fix_window_open_shadow(&wA, g_winA, WIN_BYTES) == PS_FIX_OK &&
          ps_fix_pipeline_open(&p, &wA) == PS_FIX_OK, "影子窗口 A + 管线（四核）");
    memset(&seq, 0, sizeof(seq));
    seq.proj = &jp; seq.fwd = &jf; seq.adj = &ja; seq.pdhg = &jd;
    CHECK(ps_fix_run_sequence(&p, &seq, 16u, &res) == PS_FIX_OK &&
          res.stages_ok == 4, "驱动四核串联 4/4（影子）");

    CHECK(ps_fix_window_open_shadow(&wB, g_winB, WIN_BYTES) == PS_FIX_OK, "影子窗口 B 打开");
    memset(&cfg, 0, sizeof(cfg));
    cfg.proj = &jp; cfg.fwd = &jf; cfg.adj = &ja; cfg.pdhg = &jd;
    cfg.include_version = 0;
    CHECK(ps_board_regseq_build(&s, &cfg) == 0, "四核清单生成");
    (void)ps_board_regseq_apply_writes(&s, &wB);

    /* 期望差异恰为 4 个核 CTRL 字 */
    nd = count_diffs(g_winA, g_winB, (int)WIN_WORDS, NULL);
    CHECK(nd == 4, "四核：恰好 4 处不同（各核 CTRL）");
    for (i = 0; i < 4; ++i) {
        uint32_t idx = ctrl_off[i] / 4u;
        CHECK(g_winA[idx] == (PS_FIX_CTRL_AP_START | PS_FIX_CTRL_AP_DONE) &&
              g_winB[idx] == PS_FIX_CTRL_AP_START,
              "第 i 核 CTRL：驱动 start|done / 清单 start");
    }
}

/* ---------------------------------------------------------------- E. 版本 */

static void test_if_version(void)
{
    ps_fix_window_t w;
    char reason[256];
    int rc;

    printf("\n---- E. PS_FIX_IF_VERSION ----\n");
    CHECK(ps_board_if_version_compile_check() == 1,
          "编译期：PS_FIX_IF_VERSION == PS_BOARD_EXPECT_IF_VERSION");

    CHECK(ps_fix_window_open_shadow(&w, g_winA, WIN_BYTES) == PS_FIX_OK, "大窗口打开");
    CHECK(ps_board_if_version_read(&w, &(uint32_t){0u}) == PS_BOARD_OK,
          "PL 控制页 VERSION 地址在 1 MiB 窗口内可读");

    /* 存在性探测：ID 寄存器 */
    {
        uint32_t id = 0u;
        (void)ps_board_write_reg(&w, (uint64_t)PS_BOARD_REGSEQ_PL_ID_ADDR, PS_BOARD_PL_ID_EXPECT);
        CHECK(ps_board_pl_id_read(&w, &id) == PS_BOARD_OK && id == PS_BOARD_PL_ID_EXPECT,
              "PL 控制页 ID 读回 = 0x50530001");
    }

    /* 读回 == 期望：OK */
    (void)ps_board_write_reg(&w, (uint64_t)PS_BOARD_REGSEQ_IFVER_ADDR, 1u);
    rc = ps_board_if_version_verify(&w, 1u, reason, sizeof(reason));
    CHECK(rc == PS_BOARD_OK, "读回值 == 期望 -> OK");

    /* 读回 != 期望：MISMATCH */
    (void)ps_board_write_reg(&w, (uint64_t)PS_BOARD_REGSEQ_IFVER_ADDR, 2u);
    rc = ps_board_if_version_verify(&w, 1u, reason, sizeof(reason));
    CHECK(rc == PS_BOARD_IFVER_MISMATCH, "读回值 != 期望 -> MISMATCH");

    /* 地址不在窗口内（2026-09-30 已生成位流的真实情形：没有 PL 控制页）-> UNMAPPED */
    {
        ps_fix_window_t small;
        CHECK(ps_fix_window_open_shadow(&small, g_winB, 8u * 0x10000u) == PS_FIX_OK,
              "最小窗口（8 段）打开");
        CHECK(ps_board_window_covers(&small, (uint64_t)PS_BOARD_REGSEQ_IFVER_ADDR) == 0,
              "PL 控制页地址不在 8 段窗口内");
        rc = ps_board_if_version_verify(&small, 1u, reason, sizeof(reason));
        CHECK(rc == PS_BOARD_IFVER_UNMAPPED, "读回不可达 -> UNMAPPED（该位流无此控制页）");
        CHECK(reason[0] != '\0', "UNMAPPED 给出可读原因");
        CHECK(ps_board_pl_id_read(&small, &(uint32_t){0u}) == PS_BOARD_IFVER_UNMAPPED,
              "ID 探测同样返回 UNMAPPED");
    }
}

/* ---------------------------------------------------------------- F. 最小用例 */

static void test_min_case(void)
{
    ps_fix_window_t w;
    ps_fix_proj_job_t job = make_proj_job();
    ps_board_min_report_t rep;
    int rc;

    printf("\n---- F. 最小用例（proj 单核）----\n");
    CHECK(ps_fix_window_open_shadow(&w, g_winA, WIN_BYTES) == PS_FIX_OK, "影子窗口打开");
    rc = ps_board_min_case(&w, &job, 1, 16u, &rep);
    CHECK(rc == PS_BOARD_OK && rep.core == PS_FIX_CORE_ID_PROJ, "最小用例返回 OK");
    CHECK(rep.writes_applied == 18, "写入 18 条（5 基址 x2 + 7 标量 + 1 ap_start）");
    CHECK(rep.base_readback_ok == 1, "5 个基址 HI/LO 读回一致");
    CHECK(rep.scalar_readback_ok == 1, "7 个标量读回一致");
    CHECK(rep.done_seen == 1 && (rep.ctrl_after & PS_FIX_CTRL_AP_DONE) != 0u,
          "ap_done 已到（影子替身）");

    /* 只有窗口、没写过的核：确认写入确实落在该核段，而不是别的段 */
    CHECK(ps_board_read_reg(&w, (uint64_t)PS_FIX_CORE_LR_FWD_CTRL + LRF_K) == 0u,
          "未启动的核段保持为 0");
    CHECK(ps_board_read_reg(&w, (uint64_t)PS_FIX_CORE_PROJ_CTRL + PROJ_M) == job.M,
          "PROJ_M 落在 proj 段 0xA0000000");
}

/* ---------------------------------------------------------------- G. 四核 */

static void test_four_core(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    ps_fix_proj_job_t jp = make_proj_job();
    ps_fix_lrf_job_t  jf = make_fwd_job();
    ps_fix_lra_job_t  ja = make_adj_job();
    ps_fix_pdhg_job_t jd = make_pdhg_job();
    ps_fix_sequence_t seq;
    ps_fix_sequence_result_t res;
    ps_board_regseq_t regseq;
    int wrote = -1, rc;

    printf("\n---- G. 四核串联 ----\n");
    CHECK(ps_fix_window_open_shadow(&w, g_winA, WIN_BYTES) == PS_FIX_OK &&
          ps_fix_pipeline_open(&p, &w) == PS_FIX_OK, "影子窗口 + 管线打开");
    memset(&seq, 0, sizeof(seq));
    seq.proj = &jp; seq.fwd = &jf; seq.adj = &ja; seq.pdhg = &jd;

    rc = ps_board_four_core_case(&p, &seq, 16u, &res, &regseq, &wrote, 1u);
    CHECK(rc == PS_FIX_OK && res.stages_ok == 4 && res.aborted_at == -1, "四核串联 4/4");
    CHECK(wrote == ps_board_regseq_count_kind(&regseq, PS_REGSEQ_WRITE) &&
          regseq.step[0].kind == PS_REGSEQ_READ, "四核清单回放，且首步为版本读回");
    CHECK(res.proj.best_id == 0u && res.fwd.mac_count == 0,
          "影子窗口未预置结果 -> 读回为 0（不伪造）");

    CHECK(ps_board_four_core_case(NULL, &seq, 16u, &res, NULL, NULL, 1u) == PS_BOARD_ERR_ARGS,
          "空管线被拒");
}

/* ---------------------------------------------------------------- G2. cache 维护 */

/* 记录型 cache 后端：只记 (start, bytes)，不碰内存（主机没有真 cache 指令）。 */
static uintptr_t g_fl_start[8], g_inv_start[8];
static size_t    g_fl_bytes[8],  g_inv_bytes[8];
static int       g_n_fl = 0, g_n_inv = 0;

static void rec_flush(void *ctx, void *virt, size_t bytes)
{
    (void)ctx;
    if (g_n_fl < 8) { g_fl_start[g_n_fl] = (uintptr_t)virt; g_fl_bytes[g_n_fl] = bytes; ++g_n_fl; }
}
static void rec_invalidate(void *ctx, void *virt, size_t bytes)
{
    (void)ctx;
    if (g_n_inv < 8) { g_inv_start[g_n_inv] = (uintptr_t)virt; g_inv_bytes[g_n_inv] = bytes; ++g_n_inv; }
}
static const ps_cache_ops_t g_rec_ops    = { "record", 64u, rec_flush, rec_invalidate };
static const ps_cache_ops_t g_badline_ops = { "badline", 0u, rec_flush, rec_invalidate };

static void reset_rec(void) { g_n_fl = 0; g_n_inv = 0; }

/* A6：验证「冲刷/失效成对出现」与「区间对齐到 cache line」。
 * 主机跑不了 Xil_DCache*，故用记录型后端断言调用序列与区间；真指令属板端。 */
static void test_cache_guard(void)
{
    ps_fix_window_t w;
    ps_fix_pipeline_t p;
    ps_fix_proj_job_t jp = make_proj_job();
    ps_fix_sequence_t seq;
    ps_fix_sequence_result_t res;
    ps_board_cache_span_t span;
    ps_cache_guard_t g;
    /* 故意不对齐的起点：验证守卫会把区间扩到 cache line 边界。 */
    static uint8_t inbuf[256 + 64]  __attribute__((aligned(64)));
    static uint8_t outbuf[256 + 64] __attribute__((aligned(64)));
    void       *in_virt   = (void *)(inbuf + 3);
    void       *out_virt  = (void *)(outbuf + 5);
    const size_t in_bytes = 100u, out_bytes = 130u;
    uintptr_t   s = 0u;
    size_t      l = 0u;
    int rc;

    printf("\n---- G2. PL 访问 DDR 的 cache 维护（成对守卫 + 行对齐）----\n");

    /* (1) 区间对齐：起点向下、终点向上扩到整行。 */
    rc = ps_cache_align_range(in_virt, in_bytes, 64u, &s, &l);
    CHECK(rc == PS_CACHE_OK && s == (uintptr_t)inbuf && l == 128u,
          "对齐：start 向下取整到行首、len 向上取整（3+100 -> 128）");
    CHECK(ps_cache_align_range(in_virt, in_bytes, 0u, &s, &l) == PS_CACHE_ERR_ALIGN,
          "line=0（给不出对齐区间）被拒");
    CHECK(ps_cache_align_range(NULL, 4u, 64u, &s, &l) == PS_CACHE_ERR_ARGS &&
          ps_cache_align_range(in_virt, 0u, 64u, &s, &l) == PS_CACHE_ERR_ARGS,
          "空指针 / 0 字节被拒");

    /* (2) 配对与顺序：output 先于 input、重复 input 都判 STATE。 */
    memset(&g, 0, sizeof(g));
    CHECK(ps_cache_guard_output(&g, &g_rec_ops, PS_CACHE_NONCOHERENT, out_virt, out_bytes)
              == PS_CACHE_ERR_STATE,
          "未 flush 就 invalidate（输出先于输入）被拒");
    reset_rec();
    memset(&g, 0, sizeof(g));
    CHECK(ps_cache_guard_input(&g, &g_rec_ops, PS_CACHE_NONCOHERENT, in_virt, in_bytes)
              == PS_CACHE_OK &&
          ps_cache_guard_input(&g, &g_rec_ops, PS_CACHE_NONCOHERENT, in_virt, in_bytes)
              == PS_CACHE_ERR_STATE,
          "重复 input 被拒");
    CHECK(g_n_fl == 1, "第一次 input 恰好 flush 一次");
    CHECK(ps_cache_guard_output(&g, &g_rec_ops, PS_CACHE_NONCOHERENT, out_virt, out_bytes)
              == PS_CACHE_OK && g_n_inv == 1,
          "output 恰好 invalidate 一次");
    CHECK(g_fl_start[0] == (uintptr_t)inbuf && g_fl_bytes[0] == 128u &&
          g_inv_start[0] == (uintptr_t)outbuf && g_inv_bytes[0] == 192u,
          "flush/invalidate 都落在行边界（输入 128、输出 192）");

    /* (3) 非相干但缺维护后端 / line 非法：拒绝。 */
    memset(&g, 0, sizeof(g));
    CHECK(ps_cache_guard_input(&g, NULL, PS_CACHE_NONCOHERENT, in_virt, in_bytes)
              == PS_CACHE_ERR_NO_OPS,
          "非相干映射缺维护后端被拒（NO_OPS：Linux 用户态补不了）");
    memset(&g, 0, sizeof(g));
    CHECK(ps_cache_guard_input(&g, &g_badline_ops, PS_CACHE_NONCOHERENT, in_virt, in_bytes)
              == PS_CACHE_ERR_ALIGN,
          "line_bytes=0 的后端被拒（ALIGN）");

    /* (4) 真实调用点：起核前 flush 输入、读回前 invalidate 输出。 */
    CHECK(ps_fix_window_open_shadow(&w, g_winA, WIN_BYTES) == PS_FIX_OK &&
          ps_fix_pipeline_open(&p, &w) == PS_FIX_OK, "影子窗口 + 管线打开");
    memset(&seq, 0, sizeof(seq));
    seq.proj = &jp;
    memset(&span, 0, sizeof(span));
    span.ops = &g_rec_ops; span.coherent = PS_CACHE_NONCOHERENT;
    span.input_virt = in_virt;  span.input_bytes = in_bytes;
    span.output_virt = out_virt; span.output_bytes = out_bytes;
    reset_rec();
    rc = ps_board_four_core_case_cached(&p, &seq, 16u, &res, NULL, NULL, 1u, &span);
    CHECK(rc == PS_FIX_OK && res.stages_ok == 1, "带 cache 维护的四核串联（仅 proj）成功");
    CHECK(g_n_fl == 1 && g_n_inv == 1 &&
          g_fl_start[0] == (uintptr_t)inbuf && g_inv_start[0] == (uintptr_t)outbuf,
          "调用点：flush(输入) 与 invalidate(输出) 各一次、区间正确");

    /* (5) 相干（非缓存）映射：零 cache 动作，路径 b。 */
    {
        ps_board_cache_span_t cspan = span;
        cspan.coherent = PS_CACHE_COHERENT;
        cspan.ops = NULL;
        reset_rec();
        rc = ps_board_four_core_case_cached(&p, &seq, 16u, &res, NULL, NULL, 1u, &cspan);
        CHECK(rc == PS_FIX_OK && g_n_fl == 0 && g_n_inv == 0,
              "相干（非缓存）映射：零 cache 动作");
    }

    /* (6) 非相干却缺 ops：起核之前拒绝（stages_ok 保持 0）。 */
    {
        ps_board_cache_span_t bspan = span;
        bspan.ops = NULL;
        reset_rec();
        memset(&res, 0, sizeof(res));
        rc = ps_board_four_core_case_cached(&p, &seq, 16u, &res, NULL, NULL, 1u, &bspan);
        CHECK(rc == PS_BOARD_ERR_CACHE && res.stages_ok == 0,
              "非相干缺 ops：起核前拒绝（PS_BOARD_ERR_CACHE，0 个核启动）");
    }
}

/* ---------------------------------------------------------------- H. 帧缓冲 */

static void test_fb(void)
{
    int32_t mono[4] = { -224, 224, 0, 1000 };
    uint32_t fb[4];
    int32_t r[4] = { 224, -224, 0, 0 };
    int32_t g[4] = { -224, 224, 0, 0 };
    int32_t b[4] = { -224, 0, 0, 0 };
    uint32_t rgb[4];

    printf("\n---- H. 帧缓冲格式转换 ----\n");
    CHECK(ps_board_fb_pack_mono_xrgb(mono, 1u, 4u, 4u, fb, -224, 224) == PS_BOARD_FB_OK,
          "单通道打包成功");
    CHECK((fb[0] & 0xFFu) == 0u, "lo -> 0");
    CHECK((fb[1] & 0xFFu) == 255u, "hi -> 255");
    CHECK((fb[2] & 0xFFu) == 127u, "中点 -> 127");
    CHECK((fb[3] & 0xFFu) == 255u, "超上界饱和到 255");
    CHECK(fb[1] == ps_board_fb_pack_xrgb(255u, 255u, 255u), "灰度三通道相等");

    CHECK(ps_board_fb_pack_rgb_xrgb(r, g, b, 1u, 4u, 4u, rgb, -224, 224) == PS_BOARD_FB_OK,
          "三通道打包成功");
    CHECK((rgb[0] >> 16 & 0xFFu) == 255u && (rgb[0] >> 8 & 0xFFu) == 0u &&
          (rgb[0] & 0xFFu) == 0u, "通道序 R/G/B 正确");
    CHECK(ps_board_fb_pack_rgb_xrgb(NULL, g, NULL, 1u, 4u, 4u, rgb, -224, 224) == PS_BOARD_FB_OK &&
          (rgb[0] >> 16 & 0xFFu) == 0u && (rgb[0] >> 8 & 0xFFu) == 0u,
          "空通道按 0 处理");

    {
        uint32_t gray[8];
        ps_board_fb_fill_gray(gray, 8u);
        CHECK(gray[0] == 0x00808080u && gray[7] == 0x00808080u, "灰屏整屏同色 0x808080");
    }

    CHECK(ps_board_fb_pack_mono_xrgb(NULL, 1u, 4u, 4u, fb, -224, 224) == PS_BOARD_FB_ERR_ARGS,
          "空 plane 被拒");
    CHECK(ps_board_fb_pack_mono_xrgb(mono, 1u, 4u, 2u, fb, -224, 224) == PS_BOARD_FB_ERR_ARGS,
          "stride < cols 被拒");
    CHECK(ps_board_fb_pack_mono_xrgb(mono, 1u, 4u, 4u, fb, 224, 224) == PS_BOARD_FB_ERR_RANGE,
          "hi <= lo 被拒");
}

/* ---------------------------------------------------------------- I. .psf */

static int try_paths(const char *arg, char *out, size_t cap)
{
    static const char *cands[] = {
        "../../tests/vectors/sd_frame/tiny8x8_f000.psf",
        "tests/vectors/sd_frame/tiny8x8_f000.psf",
        "../tests/vectors/sd_frame/tiny8x8_f000.psf",
    };
    size_t i;
    if (arg && arg[0] != '\0') {
        FILE *f = fopen(arg, "rb");
        if (f) { fclose(f); snprintf(out, cap, "%s", arg); return 1; }
    }
    for (i = 0; i < sizeof(cands) / sizeof(cands[0]); ++i) {
        FILE *f = fopen(cands[i], "rb");
        if (f) { fclose(f); snprintf(out, cap, "%s", cands[i]); return 1; }
    }
    return 0;
}

static void test_psf_load(const char *arg)
{
    static uint8_t ddr[4u * 1024u * 1024u];
    ps_board_arena_t arena;
    ps_sd_frame_header_t hdr;
    ps_sd_frame_check_t check;
    uint64_t phys = 0ull;
    uint16_t *codes = NULL;
    char path[512];
    char reason[256];
    int rc;

    printf("\n---- I. .psf 装载 ----\n");
    if (!try_paths(arg, path, sizeof(path))) {
        printf("  NOTE  未找到 .psf fixture（tests/vectors/sd_frame/tiny8x8_f000.psf），跳过\n");
        return;
    }
    printf("  fixture = %s\n", path);

    CHECK(ps_board_arena_init(&arena, 0x000100000000ull, sizeof(ddr), ddr) == PS_BOARD_OK,
          "影子 DDR 账本初始化");
    rc = ps_board_load_psf_file(path, &arena, &hdr, &phys, &codes, &check,
                                reason, sizeof(reason));
    CHECK(rc == PS_BOARD_OK, "读入并校验成功");
    CHECK(hdr.width == 8u && hdr.height == 8u && hdr.container_bits == 16u &&
          hdr.payload_bytes == 128u, "几何 8x8 / 16 bit 容器 / payload 128 B");
    CHECK(check.words == 64u && check.crc32 == hdr.payload_crc32 &&
          check.sum == hdr.payload_sum && check.min == hdr.payload_min &&
          check.max == hdr.payload_max, "crc32/sum/min/max 与头自洽");
    CHECK(codes != NULL && (phys % PS_BOARD_ALIGN) == 0ull, "码字落位 4096 对齐且可访问");

    /* 负例：不存在的文件 */
    rc = ps_board_load_psf_file("no_such_file.psf", &arena, &hdr, &phys, &codes, &check,
                                reason, sizeof(reason));
    CHECK(rc == PS_BOARD_ERR_PSF, "不存在的文件返回 ERR_PSF");
}

/* ---------------------------------------------------------------- 门禁 */

static void test_gates(void)
{
    ps_board_arena_t a;
    uint64_t p;
    static uint8_t ddr[4096];

    printf("\n---- J. 入参门禁 ----\n");
    CHECK(ps_board_arena_init(NULL, 0x1000ull, 4096ull, NULL) == PS_BOARD_ERR_ARGS,
          "空账本被拒");
    CHECK(ps_board_arena_alloc(NULL, 4096ull, &p) == PS_BOARD_ERR_ARGS, "空账本分配被拒");
    (void)ps_board_arena_init(&a, 0x000100000000ull, sizeof(ddr), ddr);
    CHECK(ps_board_arena_alloc(&a, 0ull, &p) == PS_BOARD_ERR_ARGS, "分配 0 字节被拒");
    CHECK(ps_board_arena_alloc(&a, 4096ull, NULL) == PS_BOARD_ERR_ARGS, "空出参被拒");
    CHECK(ps_board_load_psf_file(NULL, &a, NULL, NULL, NULL, NULL, NULL, 0) == PS_BOARD_ERR_ARGS,
          "空路径被拒");
    CHECK(ps_board_regseq_apply_writes(NULL, NULL) == -1, "空序列/空窗口回放被拒");
    CHECK(ps_board_if_version_verify(NULL, 1u, NULL, 0) == PS_BOARD_IFVER_UNMAPPED,
          "空窗口版本校验返回 UNMAPPED（不崩）");
}

/* ---------------------------------------------------------------- 主入口 */

int main(int argc, char **argv)
{
    const char *psf = (argc > 1) ? argv[1] : NULL;

    printf("==== 板端顶层应用主机自检（影子窗口，无板卡）====\n");
    test_arena();
    test_layout();
    test_regseq_build_and_order();
    test_regseq_matches_driver();
    test_regseq_matches_driver_four();
    test_if_version();
    test_min_case();
    test_four_core();
    test_cache_guard();
    test_fb();
    test_psf_load(psf);
    test_gates();

    printf("\n------------------------------------------------------------\n");
    printf("通过 %d ，失败 %d\n", g_pass, g_fail);
    if (g_fail != 0) {
        printf("结果：FAIL\n");
        return 1;
    }
    printf("结果：PASS（覆盖账本/布局/清单/版本/最小用例/四核/帧缓冲/.psf；"
           "真实时序与 DP 输出等板卡）\n");
    return 0;
}
