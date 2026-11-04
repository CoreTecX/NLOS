/* ============================================================================
 * PenumbraSight · 「预处理链输出 -> 定点核输入」转换层的主机对拍测试
 * src/software/test_meas_norm.c
 *
 * 可立即运行（不依赖板卡、不依赖 Xilinx 头文件、不跑 Vivado）：
 *     gcc -std=c99 -Wall -Wextra -Imeas_norm \
 *         meas_norm/meas_norm.c test_meas_norm.c -o bin/test_meas_norm
 *     bin/test_meas_norm ps_tmp/meas_norm
 * 退出码非零表示失败。`make -C src/software test-meas-norm` 会先跑 Python
 * 生成向量、再跑本程序。
 *
 * 验什么
 * ---------------------------------------------------------------------------
 *   1. 逐位对拍：C 的码字与 quantize.shared_exponent（Python 权威口径）**逐位相等**，
 *      覆盖 4 组帧输入：T2 接缝真实帧、全零退化帧、极小退化帧、signed 24 bit 满量程帧。
 *   2. 符号扩展：同一帧走 ps_meas_norm_from_rtl24（吃 24 bit 打包拍）与
 *      ps_meas_norm_q216（吃 int32）结果相同。
 *   3. 布局：输出满足 meas[ch*M + (r*cols+c)]；并核对它与交织序 [m*ch+ch] 不同
 *      （在通道内容非对称的两组上），排除「原样透传」也能蒙混过关。
 *   4. 指数口径：C 由 absmax 算出的指数 == Python 写下的指数。
 *   5. clamp 探针：帧级路径够不到饱和（指数由本帧 absmax 推出，最大码字模长 <= 2^15），
 *      故用**外部给定**的小指数把样本顶到 18 bit 包络边界，验证 C 的夹取与
 *      quantize.py 同界（负侧 -131071）。
 *
 * 不验什么
 * ---------------------------------------------------------------------------
 *   板上时序、DMA 搬运、寄存器写入均不在本测试范围；本层不碰寄存器。
 * ==========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "meas_norm.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do {                                   \
        if (cond) { g_pass++; printf("  PASS  %s\n", (msg)); }  \
        else { g_fail++; printf("  FAIL  %s\n", (msg)); }       \
    } while (0)

/* ---------------------------------------------------------------- 文件读取 */

static void *read_blob(const char *dir, const char *name, size_t elem,
                       size_t count, int *ok)
{
    char path[1024];
    FILE *f;
    void *buf;
    size_t got;

    *ok = 0;
    if (snprintf(path, sizeof(path), "%s/%s", dir, name) >= (int)sizeof(path)) {
        printf("  (路径过长: %s/%s)\n", dir, name);
        return NULL;
    }
    f = fopen(path, "rb");
    if (f == NULL) {
        printf("  (打不开 %s)\n", path);
        return NULL;
    }
    buf = malloc(elem * count);
    if (buf == NULL) {
        fclose(f);
        return NULL;
    }
    got = fread(buf, elem, count, f);
    fclose(f);
    if (got != count) {
        printf("  (短读 %s: %lu/%lu)\n", path,
               (unsigned long)got, (unsigned long)count);
        free(buf);
        return NULL;
    }
    *ok = 1;
    return buf;
}

/* ---------------------------------------------------------------- 单帧用例 */

static int run_frame_case(const char *dir, const char *case_name,
                          unsigned long rows, unsigned long cols,
                          unsigned long ch, unsigned long n,
                          int exponent, unsigned long sat_events,
                          long raw_absmax, long code_absmax)
{
    size_t m_blk = (size_t)rows * (size_t)cols;
    int ok_s = 0, ok_w = 0, ok_e = 0;
    int32_t *samples = NULL;
    uint32_t *beats = NULL;
    int32_t *expect = NULL;
    int32_t *got_s32 = NULL;
    int32_t *got_w24 = NULL;
    char name[128];
    int all_equal = 1;
    int interleaved_equal = 1;
    long got_code_absmax = 0;
    ps_meas_norm_result_t r32, r24;
    unsigned long i;
    int rc;

    snprintf(name, sizeof(name), "%s.s32.bin", case_name);
    samples = (int32_t *)read_blob(dir, name, sizeof(int32_t), (size_t)n, &ok_s);
    snprintf(name, sizeof(name), "%s.w24.bin", case_name);
    beats = (uint32_t *)read_blob(dir, name, sizeof(uint32_t), (size_t)n, &ok_w);
    snprintf(name, sizeof(name), "%s.exp.bin", case_name);
    expect = (int32_t *)read_blob(dir, name, sizeof(int32_t), (size_t)n, &ok_e);

    if (!ok_s || !ok_w || !ok_e) {
        printf("  FAIL  [%s] 向量读取不完整\n", case_name);
        g_fail++;
        free(samples); free(beats); free(expect);
        return -1;
    }

    got_s32 = (int32_t *)malloc(sizeof(int32_t) * (size_t)n);
    got_w24 = (int32_t *)malloc(sizeof(int32_t) * (size_t)n);
    if (got_s32 == NULL || got_w24 == NULL) {
        printf("  FAIL  [%s] 输出缓冲分配失败\n", case_name);
        g_fail++;
        free(samples); free(beats); free(expect);
        free(got_s32); free(got_w24);
        return -1;
    }

    printf("\n---- case %s (rows=%lu cols=%lu ch=%lu n=%lu) ----\n",
           case_name, rows, cols, ch, n);

    rc = ps_meas_norm_q216(samples, (uint32_t)rows, (uint32_t)cols,
                           (uint32_t)ch, got_s32, &r32);
    CHECK(rc == PS_MEAS_OK, "ps_meas_norm_q216 返回 OK");

    rc = ps_meas_norm_from_rtl24(beats, (uint32_t)rows, (uint32_t)cols,
                                 (uint32_t)ch, got_w24, &r24);
    CHECK(rc == PS_MEAS_OK, "ps_meas_norm_from_rtl24 返回 OK");

    /* 1) 逐位对拍 Python（int32 入口） */
    for (i = 0; i < n; i++) {
        if (got_s32[i] != expect[i]) {
            all_equal = 0;
            printf("        首个不等 @%lu: C=%ld Python=%ld\n",
                   (unsigned long)i, (long)got_s32[i], (long)expect[i]);
            break;
        }
    }
    CHECK(all_equal, "int32 入口与 quantize.shared_exponent 逐位相等");

    /* 2) 逐位对拍 Python（24 bit 打包入口），顺带验符号扩展路径 */
    all_equal = 1;
    for (i = 0; i < n; i++) {
        if (got_w24[i] != expect[i]) {
            all_equal = 0;
            printf("        首个不等 @%lu: C24=%ld Python=%ld\n",
                   (unsigned long)i, (long)got_w24[i], (long)expect[i]);
            break;
        }
    }
    CHECK(all_equal, "24 bit 打包入口逐位相等（符号扩展正确）");

    /* 3) 布局：由样本按索引公式独立重算，不看期望文件（避免与文件排序循环论证）。
     *    样本 i 属于流序第 (r*cols+c) 块的第 ch 通道，应落到 meas[ch*M + i/ch]。 */
    all_equal = 1;
    for (i = 0; i < n; i++) {
        size_t m = i / (size_t)ch;
        size_t c = i % (size_t)ch;
        int32_t code = ps_meas_quantize_code(samples[i], r32.exponent, NULL);
        if (got_s32[c * m_blk + m] != code) { all_equal = 0; break; }
    }
    CHECK(all_equal, "输出布局满足 meas[ch*M + m]（m = r*cols + c）");

    /* 与「交织序」区分：非对称通道内容下，通道优先块不应等于原样透传。 */
    for (i = 0; i < n; i++) {
        int32_t code = ps_meas_quantize_code(samples[i], r32.exponent, NULL);
        if (got_s32[i] != code) { interleaved_equal = 0; break; }
    }
    if (strcmp(case_name, "t2_seam") == 0 || strcmp(case_name, "full_scale") == 0) {
        CHECK(!interleaved_equal, "通道优先块与交织序不同（重排确实生效）");
    }

    /* 4) 指数口径：C 由 absmax 算出的指数 == Python 写下的指数 */
    CHECK(r32.exponent == exponent && r24.exponent == exponent,
          "共享指数与 Python 一致");
    CHECK(ps_meas_shared_exponent(r32.raw_absmax) == exponent,
          "ps_meas_shared_exponent(absmax) 与 Python 一致");
    CHECK(r32.divisor_exp == PS_MEAS_NORM_FRAC_BITS - exponent,
          "divisor_exp = frac_bits - exponent");

    /* 统计标量 */
    CHECK((long)r32.raw_absmax == raw_absmax, "raw_absmax 与 Python 一致");
    for (i = 0; i < n; i++) {
        long a = (long)got_s32[i];
        if (a < 0) { a = -a; }
        if (a > got_code_absmax) { got_code_absmax = a; }
    }
    CHECK(got_code_absmax == code_absmax, "code_absmax 与 Python 一致");
    CHECK((unsigned long)r32.sat_events == sat_events, "帧级 sat_events 与 Python 一致");
    CHECK(got_code_absmax <= (long)PS_MEAS_NORM_CODE_MAX,
          "码字落在 Q2.16 signed 18 bit 包络内");

    free(samples); free(beats); free(expect); free(got_s32); free(got_w24);
    return 0;
}

/* ---------------------------------------------------------------- clamp 探针 */

static void run_clamp_probe(const char *dir)
{
    int ok_a = 0, ok_b = 0, ok_t = 0;
    int32_t *probe = (int32_t *)read_blob(dir, "clamp_probe.s32.bin",
                                          sizeof(int32_t), 9, &ok_a);
    int32_t *expect = (int32_t *)read_blob(dir, "clamp_probe.exp.bin",
                                           sizeof(int32_t), 9, &ok_b);
    char path[1024];
    FILE *f = NULL;
    int exponent = 0;
    int all_equal = 1;
    unsigned long nitems = 9;
    unsigned long i;

    if (snprintf(path, sizeof(path), "%s/clamp_probe.exponent.txt", dir)
            < (int)sizeof(path)) {
        f = fopen(path, "r");
    }
    if (f != NULL) {
        ok_t = (fscanf(f, "%d", &exponent) == 1) ? 1 : 0;
        fclose(f);
    }

    printf("\n---- case clamp_probe (e=%d) ----\n", exponent);
    if (!ok_a || !ok_b || !ok_t) {
        printf("  FAIL  clamp 探针向量不完整\n");
        g_fail++;
        free(probe); free(expect);
        return;
    }

    for (i = 0; i < nitems; i++) {
        int sat = 0;
        int32_t code = ps_meas_quantize_code(probe[i], exponent, &sat);
        if (code != expect[i]) {
            all_equal = 0;
            printf("        @%lu: v=%ld C=%ld Python=%ld\n",
                   i, (long)probe[i], (long)code, (long)expect[i]);
        }
    }
    CHECK(all_equal, "clamp 边界逐位相等（负侧 -131071，与 quantize.py 同界）");
    CHECK(expect[0] == PS_MEAS_NORM_CODE_MIN && expect[3] == PS_MEAS_NORM_CODE_MAX,
          "探针在 ±2^17 处被夹到 ±131071");

    free(probe); free(expect);
}

/* ---------------------------------------------------------------- 指数口径表 */

static void run_exp_table(const char *dir)
{
    char path[1024];
    FILE *f;
    char line[256];
    int n = 0;
    int bad = 0;

    printf("\n---- case exp_table (2 的幂边界) ----\n");
    if (snprintf(path, sizeof(path), "%s/exp_table.txt", dir) >= (int)sizeof(path)) {
        printf("  FAIL  路径过长\n");
        g_fail++;
        return;
    }
    f = fopen(path, "r");
    if (f == NULL) {
        printf("  FAIL  打不开 exp_table.txt\n");
        g_fail++;
        return;
    }
    while (fgets(line, sizeof(line), f) != NULL) {
        long a;
        int e;
        if (sscanf(line, "%ld %d", &a, &e) != 2) { continue; }
        if (ps_meas_shared_exponent((int32_t)a) != e) {
            bad++;
            if (bad <= 3) {
                printf("        absmax=%ld C=%d Python=%d\n",
                       a, ps_meas_shared_exponent((int32_t)a), e);
            }
        }
        n++;
    }
    fclose(f);
    CHECK(n >= 25 && bad == 0,
          "ps_meas_shared_exponent 与 Python 在 2 的幂边界上逐值一致");
}

/* ---------------------------------------------------------------- 主入口 */

int main(int argc, char **argv)
{
    const char *dir = (argc > 1) ? argv[1] : "ps_tmp/meas_norm";
    char path[1024];
    FILE *f;

    printf("==== meas_norm 主机对拍（对 quantize.shared_exponent 逐位比较）====\n");
    printf("向量目录：%s\n", dir);

    if (snprintf(path, sizeof(path), "%s/manifest.txt", dir) >= (int)sizeof(path)) {
        printf("路径过长\n");
        return 2;
    }
    f = fopen(path, "r");
    if (f == NULL) {
        printf("打不开 manifest：%s\n（先跑 make -C src/software test-meas-norm）\n", path);
        return 2;
    }

    {
        char line[512];
        int nc = 0;
        while (fgets(line, sizeof(line), f) != NULL) {
            char case_name[64];
            unsigned long rows, cols, ch, n, sat;
            int exponent;
            long raw_absmax, code_absmax;

            if (line[0] == '#' || line[0] == '\n') { continue; }
            if (sscanf(line, "%63s %lu %lu %lu %lu %d %lu %ld %ld",
                       case_name, &rows, &cols, &ch, &n, &exponent, &sat,
                       &raw_absmax, &code_absmax) != 9) {
                continue;
            }
            if (rows * cols * ch != n) {
                printf("  FAIL  [%s] manifest 尺寸不自洽\n", case_name);
                g_fail++;
                continue;
            }
            run_frame_case(dir, case_name, rows, cols, ch, n, exponent, sat,
                           raw_absmax, code_absmax);
            nc++;
        }
        fclose(f);
        CHECK(nc >= 3, "manifest 提供至少 3 组帧输入");
        CHECK(nc == 4, "四组帧输入全部执行（t2_seam/all_zero/tiny/full_scale）");
    }

    run_exp_table(dir);
    run_clamp_probe(dir);

    printf("\n==== 结果：PASS %d / FAIL %d ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
