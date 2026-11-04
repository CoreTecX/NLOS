/* ============================================================================
 * PenumbraSight · 板端顶层应用（PetaLinux 用户态入口）
 * src/software/board_app/board_main.c
 *
 * 这是老师那条路线的可执行体：
 *     SD 卡 .psf -> DDR -> PL 预处理链（DMA）-> 四核 -> 结果回 DDR -> 帧缓冲 -> DP
 * 本文件只写「进程入口 + 环境适配（open/mmap/DP）」这一层；编排、寄存器清单、
 * 最小用例、四核、帧缓冲格式都在 board_top/board_regseq/board_fb 里，且那些
 * 部分已在主机侧逐条测过（见 test_board_top.c）。
 *
 * 为什么整份 main 被 PS_BOARD_APP_MAIN 包住
 * ---------------------------------------------------------------------------
 * 它要用 <fcntl.h>/<unistd.h>/<sys/mman.h>，主机（mingw）没有这些头。而
 * src/software/makefile 会自动把 board_app 目录下的 .c 编进模块库（供 host_selftest
 * 链接），若不加这层保护，主机构建会因缺头文件而红。故：
 *   · 主机构建：不定义 PS_BOARD_APP_MAIN，本文件只出类型声明（含 board_top.h）；
 *   · 板端/目标构建：-DPS_BOARD_APP_MAIN，出真正的 main。
 * 目标构建命令见 docs/board_top_app.md §5（makefile 的 board-app 目标）。
 *
 * 未验证声明
 * ---------------------------------------------------------------------------
 *   本文件的 --shadow 分支在主机上可跑（用来演练整条编排）；**非** shadow 分支
 *   依赖 /dev/uioN + mmap + DP 帧缓冲，**未在板上跑过**。UIO 入口已按标准 UIO
 *   填实（见 fpga_control/uio_backend.{h,c}），但真设备树、真 mmap/close 与
 *   map0 的 addr/size 取值属板端实测，见 sim/results/uio_entry_20261001.md。
 * ==========================================================================*/

#include "board_top.h"
#include "board_regseq.h"
#include "board_fb.h"
#include "board_pmem.h"

#include <stdio.h>
#include <string.h>

#if defined(PS_BOARD_APP_MAIN)

#include <stdlib.h>

/* 影子窗口要覆盖到预留的版本寄存器（0xA00A0000），故开到 1 MiB：
 * 0xA00A0000 - 0xA0000000 = 0xA0000 = 640 KiB < 1 MiB。 */
#define SHADOW_WIN_BYTES (1024u * 1024u)
static uint32_t g_win[SHADOW_WIN_BYTES / 4u];

/* 影子 DDR：host 端只演练编排，不必按 B 档 300 MB 模型实配，32 MiB 足够跑
 * .psf 装载与合成缓冲；真实尺寸由 --ddr 参数在板上给。 */
#define SHADOW_DDR_BYTES (32ull * 1024ull * 1024ull)

static uint64_t fake_addr(unsigned i)
{
    return 0x000100000000ull + (uint64_t)i * 0x1000ull;
}

static void fill_gain(ps_fix_gain_t *g)
{
    g->u_gain_q16 = 0x1A2B; g->s_gain_q16 = 0x288;
    g->v_gain_q16 = 0x3C4D;
    g->u_exp = -17; g->s_exp = PS_FIX_GAIN_EXP_INVALID; g->v_exp = -16;
}

static ps_fix_proj_job_t make_proj_job(void)
{
    ps_fix_proj_job_t j;
    memset(&j, 0, sizeof(j));
    j.meas = fake_addr(1); j.u_codes = fake_addr(2); j.gain_q16 = fake_addr(3);
    j.score_out = fake_addr(4); j.score_ch = fake_addr(5);
    j.P = 243; j.K = 8; j.M = 9120;
    j.nx = 9; j.ny = 9; j.nz = 3; j.margin_thresh_q16 = 0x8000u;
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

static void usage(const char *argv0)
{
    printf("用法：%s [选项]\n", argv0);
    printf("  --shadow            影子窗口演练整条编排（主机可跑；不含任何硬件/性能结论）\n");
    printf("  --uio <dev>         真板 /dev/uioN（或 auto：按 /sys/class/uio/uioN/name 扫描）\n");
    printf("  --print-regseq      只打印四核寄存器初始化清单（顺序/地址/期望值）后退出\n");
    printf("  --ifver             只做 PS_FIX_IF_VERSION 校验后退出\n");
    printf("  --min-case          只跑一个核的最小用例（写基址->写标量->ap_start->ap_done->读回）\n");
    printf("  --psf <file>        读一帧 .psf 进 DDR（校验 crc32/sum/sumsq/min/max）\n");
    printf("  --fb-ppm <file>     合成一张结果图并按 XRGB8888 打包，导出为 PPM 供肉眼核对\n");
    printf("  --process-model <psm> --geometry-hash <hex> --frame <psf> [--frame ...]\n");
    printf("    Linux: real RAW/DMA/projection/reconstruction; 1/4/8 frames, reserved udmabuf\n");
    printf("    --output <prefix> --iterations <8..4096> --lambda-kernel <0..1>\n");
    printf("    --model-layout <legacy_f|canonical_c> --snr-q16 <0..65535> --fbdev </dev/fb0>\n");
    printf("  --help              本帮助\n");
}

static int cmd_print_regseq(void)
{
    ps_fix_proj_job_t jp = make_proj_job();
    ps_fix_lrf_job_t  jf = make_fwd_job();
    ps_fix_lra_job_t  ja = make_adj_job();
    ps_fix_pdhg_job_t jd = make_pdhg_job();
    ps_board_regseq_cfg_t cfg;
    ps_board_regseq_t seq;
    static char buf[16384];
    int n;

    memset(&cfg, 0, sizeof(cfg));
    cfg.proj = &jp; cfg.fwd = &jf; cfg.adj = &ja; cfg.pdhg = &jd;
    cfg.include_version = 1;
    cfg.if_version = PS_BOARD_EXPECT_IF_VERSION;
    if (ps_board_regseq_build(&seq, &cfg) != 0) { return 1; }
    n = ps_board_regseq_print(&seq, buf, sizeof(buf));
    if (n < 0) { return 1; }
    printf("==== 四核寄存器初始化清单（%d 步，%s）====\n", seq.n,
           seq.overflow ? "已截断" : "完整");
    printf("%s", buf);
    return 0;
}

static int cmd_ifver(ps_fix_window_t *win)
{
    char reason[256];
    uint32_t id = 0u;
    int id_rc = ps_board_pl_id_read(win, &id);
    int rc = ps_board_if_version_verify(win, PS_BOARD_EXPECT_IF_VERSION,
                                        reason, sizeof(reason));
    printf("---- PS_FIX_IF_VERSION 校验 ----\n");
    printf("  编译期：PS_FIX_IF_VERSION=%d，期望=%d -> %s\n",
           (int)PS_FIX_IF_VERSION, (int)PS_BOARD_EXPECT_IF_VERSION,
           ps_board_if_version_compile_check() ? "一致" : "不一致");
    if (id_rc == PS_BOARD_OK) {
        printf("  链路探测：PL 控制页 ID=0x%08X（期望 0x%08X）-> %s\n",
               id, (unsigned)PS_BOARD_PL_ID_EXPECT,
               (id == PS_BOARD_PL_ID_EXPECT) ? "命中" : "不符");
    } else {
        printf("  链路探测：PL 控制页不可达 (rc=%d)\n", id_rc);
    }
    if (rc == PS_BOARD_OK) {
        printf("  读回：一致\n");
    } else if (rc == PS_BOARD_IFVER_UNMAPPED) {
        printf("  读回：不可达（%s）\n", reason);
    } else {
        printf("  读回：不一致（%s）\n", reason);
    }
    printf("  提示：PL 控制页（0xA00A0000）是 2026-09-30 之后才进块设计脚本的；"
           "在旧位流上读回为 UNMAPPED，需重新出位流（见文档 §4）\n");
    return (rc == PS_BOARD_OK || rc == PS_BOARD_IFVER_UNMAPPED) ? 0 : 1;
}

static int cmd_min_case(ps_fix_window_t *win, int shadow)
{
    ps_fix_proj_job_t job = make_proj_job();
    ps_board_min_report_t rep;
    int rc = ps_board_min_case(win, &job, shadow, 1000000u, &rep);
    printf("---- 最小用例（proj 单核）----\n");
    printf("  写入寄存器条数 : %d\n", rep.writes_applied);
    printf("  基址读回一致   : %s\n", rep.base_readback_ok ? "是" : "否");
    printf("  标量读回一致   : %s\n", rep.scalar_readback_ok ? "是" : "否");
    printf("  ap_done        : %s（CTRL=0x%08X）\n",
           rep.done_seen ? "已到" : "未到", rep.ctrl_after);
    if (rc != PS_BOARD_OK) { printf("  结果：失败（rc=%d）\n", rc); return 1; }
    printf("  结果：PASS\n");
    return 0;
}

static int cmd_four_core(ps_fix_window_t *win, int shadow)
{
    ps_fix_pipeline_t p;
    ps_fix_sequence_t seq;
    ps_fix_sequence_result_t res;
    ps_board_regseq_t regseq;
    ps_fix_proj_job_t jp = make_proj_job();
    ps_fix_lrf_job_t  jf = make_fwd_job();
    ps_fix_lra_job_t  ja = make_adj_job();
    ps_fix_pdhg_job_t jd = make_pdhg_job();
    int wrote = -1, rc;

    (void)shadow;
    if (ps_fix_pipeline_open(&p, win) != PS_FIX_OK) { return 1; }
    memset(&seq, 0, sizeof(seq));
    seq.proj = &jp; seq.fwd = &jf; seq.adj = &ja; seq.pdhg = &jd;

    rc = ps_board_four_core_case(&p, &seq, 1000000u, &res, &regseq, &wrote,
                                 PS_BOARD_EXPECT_IF_VERSION);
    printf("---- 四核串联 ----\n");
    printf("  清单回放写入   : %d 条\n", wrote);
    printf("  成功核数       : %d/4（首个失败核=%d）\n", res.stages_ok, res.aborted_at);
    printf("  proj best_id   : %u\n", res.proj.best_id);
    if (rc != PS_FIX_OK) { printf("  结果：失败（rc=%d）\n", rc); return 1; }
    printf("  结果：PASS（影子：仅寄存器协议，非时序/非性能）\n");
    return 0;
}

static int cmd_load_psf(ps_fix_window_t *win, const char *path)
{
    (void)win;
    /* DDR 缓冲：**优先 ps_pmem_alloc**（udmabuf/dma-heap/devmem，板端 mmap）。
     * 这些后端由内核按 DMA 属性映射，是 PL 可安全访问的非缓存/相干内存，于是
     * ps_cache 守卫可以置 coherent=1、不产生任何 cache 动作。
     * 只有在没有 ps_pmem 后端的**主机影子**下才退回 malloc —— malloc 出来的是
     * 可缓存普通内存，**不能**交给真 PL（用户态也无法 flush 它）。因此板上若
     * ps_pmem 不可用，这里直接失败而不是偷偷 malloc。 */
    unsigned char *ddr = NULL;
    ps_board_arena_t arena;
    ps_pmem_block_t blk;
    const ps_pmem_env_t *pmem_env = ps_pmem_env_default();
    ps_sd_frame_header_t hdr;
    ps_sd_frame_check_t check;
    uint64_t phys = 0ull;
    uint16_t *codes = NULL;
    char reason[256];
    int using_pmem = 0;
    int coherent = 0;
    int rc;

    memset(&blk, 0, sizeof(blk));
    blk.fd = -1;
    reason[0] = '\0';

    printf("---- .psf 装载 ----\n");
    if (pmem_env) {
        const int prc = ps_pmem_alloc(pmem_env, SHADOW_DDR_BYTES, PS_PMEM_F_REQUIRE_PHYS,
                                      &blk, reason, sizeof(reason));
        if (prc != PS_PMEM_OK) {
            printf("  DDR 分配失败（rc=%d）：%s\n", prc, reason);
            return 1;
        }
        ddr = (unsigned char *)blk.virt;
        using_pmem = 1;
        /* udmabuf / dma-heap / 已预留的 /dev/mem 都是 PL 相干内存。 */
        coherent = (blk.kind == PS_PMEM_KIND_UDMABUF ||
                    blk.kind == PS_PMEM_KIND_DMAHEAP ||
                    blk.kind == PS_PMEM_KIND_DEVMEM) ? PS_CACHE_COHERENT
                                                     : PS_CACHE_NONCOHERENT;
        if (ps_board_arena_init(&arena, blk.phys, blk.bytes, ddr) != PS_BOARD_OK) {
            ps_pmem_free(pmem_env, &blk);
            return 1;
        }
        printf("  DDR            : %s（kind=%d，phys=0x%llX），相干=%s\n",
               blk.dev_path, blk.kind, (unsigned long long)blk.phys,
               (coherent == PS_CACHE_COHERENT) ? "是（非缓存，无需 cache 维护）" : "否");
        printf("  cache 路径     : %s\n",
               (coherent == PS_CACHE_COHERENT)
                   ? "非缓存映射（路径 b）；ps_cache 守卫 coherent=1，零动作"
                   : "非相干：必须在提交前 flush、读回前 invalidate（路径 a），"
                     "且需注入维护后端（Linux 用户态无法补做）");
    } else {
        ddr = (unsigned char *)malloc((size_t)SHADOW_DDR_BYTES);
        if (!ddr) { return 1; }
        if (ps_board_arena_init(&arena, 0x000100000000ull, SHADOW_DDR_BYTES, ddr) != PS_BOARD_OK) {
            free(ddr); return 1;
        }
        printf("  DDR            : 主机影子（malloc；无 PL，cache 不相干项不适用）\n");
    }

    rc = ps_board_load_psf_file(path, &arena, &hdr, &phys, &codes, &check,
                                reason, sizeof(reason));
    if (rc != PS_BOARD_OK) {
        printf("  失败（rc=%d）：%s\n", rc, reason);
        if (using_pmem) { ps_pmem_free(pmem_env, &blk); } else { free(ddr); }
        return 1;
    }
    printf("  文件           : %s\n", path);
    printf("  几何           : %ux%u，valid_bits=%u，packing=%u，bayer=%u\n",
           hdr.width, hdr.height, hdr.valid_bits, hdr.value_packing, hdr.bayer_phase);
    printf("  payload        : %u B，crc32=0x%08X（重算=0x%08X），sum=%llu\n",
           hdr.payload_bytes, hdr.payload_crc32, check.crc32,
           (unsigned long long)check.sum);
    printf("  落位           : 物理 0x%llX，码字 %u 个\n",
           (unsigned long long)phys, hdr.width * hdr.height);
    if (codes) {
        printf("  首 4 个码字    : %u %u %u %u\n", codes[0], codes[1], codes[2], codes[3]);
    }
    printf("  结果：PASS（读到的字节与头自洽；未上板）\n");
    if (using_pmem) { ps_pmem_free(pmem_env, &blk); } else { free(ddr); }
    return 0;
}

static int cmd_fb_ppm(const char *path)
{
    /* 合成一张 96x96 的灰度结果图（模拟 f3 的一帧），按 XRGB8888 打包再导 PPM。
     * 目的只是让「定标与打包」这一步在真机上也能被肉眼看一次。 */
    enum { W = 96, H = 96 };
    static int32_t plane[W * H];
    static uint32_t fb[W * H];
    FILE *f;
    int y, x;

    for (y = 0; y < H; ++y) {
        for (x = 0; x < W; ++x) {
            /* 对角线渐变 + 四角方块，方便一眼看出方向与定标对不对。 */
            plane[y * W + x] = (x * 2 - W) + (y * 4 - 2 * H);
        }
    }
    if (ps_board_fb_pack_mono_xrgb(plane, H, W, W, fb, -224, 224) != PS_BOARD_FB_OK) {
        return 1;
    }
    f = fopen(path, "wb");
    if (!f) { printf("  无法写 %s\n", path); return 1; }
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (y = 0; y < H; ++y) {
        for (x = 0; x < W; ++x) {
            uint32_t v = fb[y * W + x];
            unsigned char rgb[3];
            rgb[0] = (unsigned char)((v >> 16) & 0xFFu);
            rgb[1] = (unsigned char)((v >> 8) & 0xFFu);
            rgb[2] = (unsigned char)(v & 0xFFu);
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
    printf("---- 帧缓冲导出 ----\n  已写 %s（%dx%d P6）\n", path, W, H);
    return 0;
}

#if defined(__linux__)
int ps_frame_command(int argc, char **argv);
#endif

int main(int argc, char **argv)
{
#if defined(__linux__)
    int frame_rc = ps_frame_command(argc, argv);
    if (frame_rc != -2) return frame_rc;
#endif
    ps_fix_window_t win;
    const char *uio = NULL;
    const char *psf = NULL;
    const char *ppm = NULL;
    int shadow = 1;      /* 默认影子：主机可直接演练 */
    int do_regseq = 0, do_ifver = 0, do_min = 0;
    int i, rc;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--shadow") == 0) { shadow = 1; }
        else if (strcmp(argv[i], "--uio") == 0 && i + 1 < argc) { uio = argv[++i]; shadow = 0; }
        else if (strcmp(argv[i], "--print-regseq") == 0) { do_regseq = 1; }
        else if (strcmp(argv[i], "--ifver") == 0) { do_ifver = 1; }
        else if (strcmp(argv[i], "--min-case") == 0) { do_min = 1; }
        else if (strcmp(argv[i], "--psf") == 0 && i + 1 < argc) { psf = argv[++i]; }
        else if (strcmp(argv[i], "--fb-ppm") == 0 && i + 1 < argc) { ppm = argv[++i]; }
        else if (strcmp(argv[i], "--help") == 0) { usage(argv[0]); return 0; }
        else { printf("未知参数：%s\n", argv[i]); usage(argv[0]); return 2; }
    }

    printf("==== PenumbraSight 板端顶层应用 ====\n");

    if (do_regseq) { return cmd_print_regseq(); }

    /* 打开映射窗口。 */
    if (shadow) {
        rc = ps_fix_window_open_shadow(&win, g_win, SHADOW_WIN_BYTES);
        if (rc != PS_FIX_OK) { printf("影子窗口打开失败 rc=%d\n", rc); return 1; }
        printf("窗口：影子（%u KiB）—— 只覆盖寄存器协议与数值口径，非硬件结论\n",
               SHADOW_WIN_BYTES / 1024u);
    } else {
        char err[256];
        rc = ps_fix_window_open_uio(&win, uio ? uio : "/dev/uio0", err, sizeof(err));
        if (rc != PS_FIX_OK) {
            printf("窗口：真板 %s 打开失败（rc=%d）：%s\n",
                   uio ? uio : "auto", rc, err);
            printf("按返回码定位：-6 无匹配设备/-7 权限/-8 mmap/-9 map0 覆盖不足；"
                   "见 sim/results/uio_entry_20261001.md 的前提清单。\n");
            return 3;
        }
    }

    if (!shadow && (do_min || (!psf && !ppm && !do_ifver))) {
        fprintf(stderr, "Test jobs contain synthetic addresses. Use --process-model for real frames, or the baremetal minimal test.\n");
        ps_fix_window_close(&win);
        return 2;
    }
    if (do_ifver) { int v = cmd_ifver(&win); ps_fix_window_close(&win); return v; }
    if (do_min)   { return cmd_min_case(&win, shadow); }
    if (psf) { int v=cmd_load_psf(&win,psf); ps_fix_window_close(&win); return v; }
    if (ppm)      { return cmd_fb_ppm(ppm); }

    /* 默认：版本校验 -> 最小用例 -> 四核。 */
    (void)cmd_ifver(&win);
    rc = cmd_min_case(&win, shadow);
    if (rc != 0) { return rc; }
    return cmd_four_core(&win, shadow);
}

#else /* !PS_BOARD_APP_MAIN */

/* 主机构建：本翻译单元只提供类型可见性，不产生 main。
 * 版本一致性在此做一次编译期断言，改坏任一侧会当场暴露。 */
int ps_board_main_host_tu_marker(void);
int ps_board_main_host_tu_marker(void) { return (int)PS_BOARD_EXPECT_IF_VERSION; }

#endif /* PS_BOARD_APP_MAIN */
