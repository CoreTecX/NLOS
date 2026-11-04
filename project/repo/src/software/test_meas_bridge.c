/* ============================================================================
 * PenumbraSight · 「RTL 24 位打包拍 -> meas_norm -> 定点投影核入参」接缝自检
 * src/software/test_meas_bridge.c
 *
 * 可立即运行（不依赖板卡、不依赖 Xilinx 头文件）：
 *     gcc -std=c11 -Wall -Wextra -Imeas_norm -Ifpga_control -Iinclude -I../../board \
 *         meas_norm/meas_norm.c fpga_control/fix_core_driver.c ../../board/fix_core.c \
 *         test_meas_bridge.c -o bin/test_meas_bridge
 *     bin/test_meas_bridge [ps_tmp/meas_norm]
 * 退出码非零表示失败。`make -C src/software test-meas-bridge` 会先跑 Python 生成
 * 向量，再跑本程序。
 *
 * 为什么要有这个测试
 * ---------------------------------------------------------------------------
 * `sim/results/meas_norm_rtl_to_fixcore_20260929.md` §七 第 5 条记：
 * 「与真实 gain_q16 折算的端到端复算」未做，且 `fix_core_driver.c` **没有**调用
 * `meas_norm`（`job->meas` 一直是调用方自己填的）。2026-09-30 补上这条接线后，
 * 本测试把它钉住：
 *
 *   A. 24 位打包拍 -> meas 缓冲
 *      `ps_fix_prepare_meas()`（driver 侧入口）与 `ps_meas_norm_from_rtl24()`（测量层）
 *      在同一输入上**逐位相同**；若 Python 生成的向量在（`ps_tmp/meas_norm/t2_seam.*`），
 *      再与 `.exp.bin`（quantize.shared_exponent 的权威期望）逐位相同。
 *   B. 帧量程折算
 *      `ps_fix_fold_frame_scale()` 把本帧共享指数折成「码字 -> 绝对测量」乘数：
 *      t2_seam（e=24）下 d_scale = 2^8，u_scale = 2^-17 -> 总乘数 = 2^-9（精确）。
 *   C. 定点核入参
 *      `ps_fix_proj_attach_meas()` 把 meas 缓冲地址填进 `ps_fix_proj_job_t.meas`，
 *      影子窗口跑 proj 作业返回 OK，且 `PROJ_BASE_MEAS` 的 LO/HI 两个半字与地址一致。
 *
 * 不验什么
 * ---------------------------------------------------------------------------
 * 板上 DMA 把 `m_diff_tdata[23:0]` 放进 32 位字的**实际落位**（本测试沿用
 * meas_norm 的「低 24 bit、signed」假设）；真实 AXI 搬运与时序。
 * ==========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fix_core_driver.h"   /* 内含 meas_norm.h / psm_format.h / fix_core.h */

/* 影子窗口：覆盖 0xA0000000 起八段共 512 KiB。 */
#define WIN_BYTES (8u * 0x10000u)
static uint32_t g_win[WIN_BYTES / 4u];

/* 4096 对齐的 meas 缓冲（= B 档差分帧 95x96x3）。 */
#define ROWS 95u
#define COLS 96u
#define CHS  3u
#define NSAMP (ROWS * COLS * CHS)
static _Alignas(4096) int32_t g_meas[NSAMP];

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

/* 与 test_fix_core.c 同一套互不相同、4096 对齐的假地址。 */
static uint64_t fake_addr(unsigned i)
{
    return 0x000100000000ull + (uint64_t)i * 0x1000ull;
}

/* ---------------------------------------------------------------- 激励 */

/* 把 signed 24 位差分码字打成 RTL 输出拍：低 24 bit 有效，高位保留 0。 */
static uint32_t pack24(int32_t v)
{
    return (uint32_t)v & PS_MEAS_RTL_TAG_MASK;
}

/* 确定性合成帧：三通道内容**非对称**，好让「通道优先重排」可判（否则透传也过）。
 * i = (r*cols + c)*channels + ch；值随 r/c/ch 变化，并混入负值。 */
static void fill_synth_beats(uint32_t *beats)
{
    unsigned r, c, ch;
    for (r = 0; r < ROWS; ++r) {
        for (c = 0; c < COLS; ++c) {
            for (ch = 0; ch < CHS; ++ch) {
                int32_t v = (int32_t)((r * 37u + c * 101u + ch * 997u) % 200003u)
                            - 100001;
                beats[(r * COLS + c) * CHS + ch] = pack24(v);
            }
        }
    }
}

/* ---------------------------------------------------------------- 向量读取 */

static void *read_blob(const char *dir, const char *name, size_t elem,
                       size_t count, int *ok)
{
    char path[1024];
    FILE *f;
    void *buf;
    size_t got;

    *ok = 0;
    if (snprintf(path, sizeof(path), "%s/%s", dir, name) >= (int)sizeof(path)) {
        return NULL;
    }
    f = fopen(path, "rb");
    if (f == NULL) { return NULL; }
    buf = malloc(elem * count);
    if (buf == NULL) { fclose(f); return NULL; }
    got = fread(buf, elem, count, f);
    fclose(f);
    if (got != count) { free(buf); return NULL; }
    *ok = 1;
    return buf;
}

/* ---------------------------------------------------------------- A. 24 位拍 -> meas */

static void test_prepare_meas(const char *dir)
{
    static uint32_t beats[NSAMP];
    int32_t ref[NSAMP];
    ps_meas_norm_result_t f_syn, f_ref;
    int rcS, rcR;
    unsigned i;
    int all_same = 1;

    printf("\n---- A. 24 位打包拍 -> meas 缓冲（合成帧 %ux%ux%u）----\n", ROWS, COLS, CHS);
    fill_synth_beats(beats);

    memset(g_meas, 0, sizeof(g_meas));
    rcS = ps_fix_prepare_meas(beats, ROWS, COLS, CHS, g_meas, &f_syn);
    CHECK(rcS == PS_FIX_OK, "ps_fix_prepare_meas 返回 OK（driver 入口）");

    rcR = ps_meas_norm_from_rtl24(beats, ROWS, COLS, CHS, ref, &f_ref);
    CHECK(rcR == PS_MEAS_OK, "ps_meas_norm_from_rtl24 返回 OK（测量层参照）");

    for (i = 0; i < NSAMP; ++i) {
        if (g_meas[i] != ref[i]) { all_same = 0; break; }
    }
    CHECK(all_same, "driver 入口与测量层输出逐位相同（逐 27360 个码字）");
    CHECK(f_syn.exponent == f_ref.exponent && f_syn.raw_absmax == f_ref.raw_absmax &&
          f_syn.code_absmax == f_ref.code_absmax && f_syn.sat_events == f_ref.sat_events,
          "帧级标量（exponent/raw_absmax/code_absmax/sat_events）一致");

    /* 布局：样本 i 属于流序第 (r*cols+c) 块的第 ch 通道，应落到 meas[ch*M + m] */
    {
        int layout_ok = 1;
        for (i = 0; i < NSAMP; ++i) {
            size_t m = i / CHS;
            size_t c = i % CHS;
            int32_t code = ps_meas_quantize_code(ps_meas_sext24(beats[i]),
                                                 f_syn.exponent, NULL);
            if (g_meas[c * (ROWS * COLS) + m] != code) { layout_ok = 0; break; }
        }
        CHECK(layout_ok, "输出满足通道优先块 meas[ch*M + m]（m = r*cols + c）");
    }
    /* 通道内容非对称：通道优先块不应等于原样透传 */
    {
        int interleaved_equal = 1;
        for (i = 0; i < NSAMP; ++i) {
            int32_t code = ps_meas_quantize_code(ps_meas_sext24(beats[i]),
                                                 f_syn.exponent, NULL);
            if (g_meas[i] != code) { interleaved_equal = 0; break; }
        }
        CHECK(!interleaved_equal, "通道优先块 != 交织序（重排确实生效）");
    }
    CHECK(PS_MEAS_NORM_MANTISSA_BITS == 18 && PS_MEAS_NORM_FRAC_BITS == 16,
          "meas_norm 静态格式 = Q2.16 signed 18 bit");
    CHECK(f_syn.divisor_exp == PS_MEAS_NORM_FRAC_BITS - f_syn.exponent,
          "divisor_exp = frac_bits - exponent");

    /* 真实帧（T2 接缝）的向量：由 tools/gen_meas_norm_vectors.py 用
     * quantize.shared_exponent 生成，是权威期望。文件不在则明确跳过。 */
    {
        int ok_w = 0, ok_e = 0;
        uint32_t *w = (uint32_t *)read_blob(dir, "t2_seam.w24.bin", 4, NSAMP, &ok_w);
        int32_t *e = (int32_t *)read_blob(dir, "t2_seam.exp.bin", 4, NSAMP, &ok_e);
        if (ok_w && ok_e) {
            ps_meas_norm_result_t ft;
            int same_py = 1;
            memset(g_meas, 0, sizeof(g_meas));
            rcS = ps_fix_prepare_meas(w, ROWS, COLS, CHS, g_meas, &ft);
            CHECK(rcS == PS_FIX_OK, "T2 真实帧：ps_fix_prepare_meas 返回 OK");
            for (i = 0; i < NSAMP; ++i) {
                if (g_meas[i] != e[i]) { same_py = 0; break; }
            }
            CHECK(same_py, "T2 真实帧：与 Python 期望逐位相同（quantize.shared_exponent）");
            CHECK(ft.exponent == 24 && ft.raw_absmax == 6214796 && ft.code_absmax == 24277,
                  "T2 真实帧：exponent=24 / raw_absmax=6214796 / code_absmax=24277");

            /* B. 帧量程折算：d_scale = 2^(24-16) = 2^8 */
            {
                ps_fix_frame_scale_t sc;
                double u_scale = 7.62939453125e-06;   /* 2^-17，B 档 psm 候选头 +8 */
                int rc = ps_fix_fold_frame_scale(&ft, (float)u_scale, &sc);
                CHECK(rc == PS_FIX_OK, "T2 帧量程折算返回 OK");
                CHECK(ps_fix_meas_scale_exponent(&ft) == 8 &&
                      ps_fix_meas_d_scale(&ft) == 256.0,
                      "d_scale = 2^(24-16) = 256");
                CHECK(sc.d_scale_exp == 8 && sc.exact == 1 && sc.exp == -9 &&
                      sc.multiplier == 0.001953125,
                      "u_scale(2^-17) x d_scale(2^8) = 2^-9（幂次支路，精确）");
            }

            /* C. 定点核入参：把 meas 缓冲地址绑进投影作业并在影子窗口跑通 */
            {
                ps_fix_window_t win;
                ps_fix_pipeline_t pip;
                ps_fix_proj_job_t job;
                ps_fix_proj_result_t res;
                ps_fix_frame_scale_t sc;
                /* g_meas 是 _Alignas(4096) 的静态缓冲，可当 4096 对齐的「物理」地址用 */
                uint64_t meas_phys = (uint64_t)(uintptr_t)g_meas;
                int rcx;

                CHECK((uintptr_t)g_meas % 4096u == 0u, "meas 缓冲 4096 对齐（m_axi 约束）");

                memset(&job, 0, sizeof(job));
                job.meas     = fake_addr(1);
                job.u_codes  = fake_addr(2);
                job.gain_q16 = fake_addr(3);
                job.score_out = fake_addr(4);
                job.score_ch  = fake_addr(5);
                job.P = 243; job.K = 8; job.M = 9120;
                job.nx = 9; job.ny = 9; job.nz = 3;
                job.margin_thresh_q16 = 0x8000u;

                memset(&sc, 0, sizeof(sc));
                rcx = ps_fix_proj_attach_meas(&job, meas_phys, 7.62939453125e-06f,
                                              &ft, &sc);
                CHECK(rcx == PS_FIX_OK && job.meas == meas_phys,
                      "attach_meas 把 meas 缓冲地址绑进投影作业");
                CHECK(sc.exp == -9, "attach_meas 同时回报帧量程折算（exp = -9）");

                rcx = ps_fix_window_open_shadow(&win, g_win, WIN_BYTES);
                rcx |= ps_fix_pipeline_open(&pip, &win);
                CHECK(rcx == PS_FIX_OK, "影子窗口 + 四核管线打开");
                rcx = ps_fix_run_projection(&pip, &job, 16u, &res);
                CHECK(rcx == PS_FIX_OK, "带 meas_norm 输出的 proj 作业在影子窗口返回 OK");
                CHECK(rd(0x10000u + PROJ_BASE_MEAS + PS_FIX_64_LO) ==
                      (uint32_t)(meas_phys & 0xFFFFFFFFu) &&
                      rd(0x10000u + PROJ_BASE_MEAS + PS_FIX_64_HI) ==
                      (uint32_t)(meas_phys >> 32),
                      "PROJ_BASE_MEAS 的 LO/HI 两半字 = meas 缓冲地址");
            }
        } else {
            printf("  NOTE  未找到 %s/t2_seam.{w24,exp}.bin（先跑 make test-meas-norm），"
                   "跳过真实帧与帧量程折算\n", dir);
        }
        free(w); free(e);
    }
}

/* ---------------------------------------------------------------- 全零帧的量程 */

static void test_zero_frame_scale(void)
{
    static uint32_t beats[NSAMP];
    ps_meas_norm_result_t f;
    ps_fix_frame_scale_t sc;
    int rc;

    printf("\n---- A'/B'. 全零帧：exponent=0 -> d_scale = 2^-16 ----\n");
    memset(beats, 0, sizeof(beats));
    memset(g_meas, 0xFF, sizeof(g_meas));      /* 先污染，确认会被写 */
    rc = ps_fix_prepare_meas(beats, ROWS, COLS, CHS, g_meas, &f);
    CHECK(rc == PS_FIX_OK, "全零帧 prepare_meas 返回 OK");
    CHECK(f.raw_absmax == 0 && f.exponent == 0 && f.code_absmax == 0,
          "全零帧：absmax=0 -> exponent=0、码字全 0");
    CHECK(ps_fix_meas_scale_exponent(&f) == -16, "全零帧 d_scale_exp = -16");

    rc = ps_fix_fold_frame_scale(&f, 7.62939453125e-06f, &sc);   /* u_scale = 2^-17 */
    CHECK(rc == PS_FIX_OK && sc.exact == 1 && sc.exp == -33 &&
          sc.multiplier == 1.1641532182693481e-10,                /* 2^-17 * 2^-16 = 2^-33 */
          "全零帧：u_scale(2^-17) x d_scale(2^-16) = 2^-33（幂次支路）");

    /* 非 2 的幂的 u_scale -> 走 Q16.16 支路 */
    rc = ps_fix_fold_frame_scale(&f, 3.0e-05f, &sc);
    CHECK(rc == PS_FIX_OK && sc.exact == 0 && sc.exp == PS_FIX_GAIN_EXP_INVALID,
          "非 2 的幂 u_scale 落到 Q16.16 支路（exp = INVALID）");
}

/* ---------------------------------------------------------------- 门禁 */

static void test_gates(void)
{
    ps_fix_frame_scale_t sc;
    ps_fix_proj_job_t job;
    static uint32_t beats[4];
    int32_t out[4];

    printf("\n---- D. 入参门禁 ----\n");
    CHECK(ps_fix_prepare_meas(NULL, ROWS, COLS, CHS, g_meas, NULL) == PS_FIX_DRV_ERR_ARGS,
          "空 beats 被拒");
    CHECK(ps_fix_prepare_meas(beats, ROWS, COLS, CHS, NULL, NULL) == PS_FIX_DRV_ERR_ARGS,
          "空 meas 被拒");
    CHECK(ps_fix_prepare_meas(beats, 0u, COLS, CHS, out, NULL) == PS_FIX_DRV_ERR_ARGS,
          "rows=0 被拒");
    CHECK(ps_fix_prepare_meas(beats, ROWS, COLS, 0u, out, NULL) == PS_FIX_DRV_ERR_ARGS,
          "channels=0 被拒");
    CHECK(ps_fix_fold_frame_scale(NULL, 1.0f, &sc) == PS_FIX_DRV_ERR_ARGS,
          "空 frame 折算被拒");
    {
        ps_meas_norm_result_t f;
        memset(&f, 0, sizeof(f));
        f.exponent = 0; f.frac_bits = PS_MEAS_NORM_FRAC_BITS;
        CHECK(ps_fix_fold_frame_scale(&f, 0.0f, &sc) == PS_FIX_DRV_ERR_ARGS &&
              ps_fix_fold_frame_scale(&f, -1.0f, &sc) == PS_FIX_DRV_ERR_ARGS,
              "非正 u_scale 被拒");
    }
    memset(&job, 0, sizeof(job));
    CHECK(ps_fix_proj_attach_meas(&job, 0ull, 1.0f, NULL, NULL) == PS_FIX_DRV_ERR_ARGS,
          "attach 空地址被拒");
    CHECK(ps_fix_proj_attach_meas(NULL, fake_addr(1), 1.0f, NULL, NULL)
          == PS_FIX_DRV_ERR_ARGS, "attach 空 job 被拒");
}

/* ---------------------------------------------------------------- 主入口 */

int main(int argc, char **argv)
{
    const char *dir = (argc > 1) ? argv[1] : "ps_tmp/meas_norm";

    printf("==== 24 位打包拍 -> meas_norm -> 定点投影核入参 接缝自检 ====\n");
    printf("向量目录：%s\n", dir);

    test_prepare_meas(dir);
    test_zero_frame_scale();
    test_gates();

    printf("\n------------------------------------------------------------\n");
    printf("通过 %d ，失败 %d\n", g_pass, g_fail);
    if (g_fail != 0) {
        printf("结果：FAIL\n");
        return 1;
    }
    printf("结果：PASS（覆盖信号调理级与寄存器入参；DMA 实际落位等板卡）\n");
    return 0;
}
