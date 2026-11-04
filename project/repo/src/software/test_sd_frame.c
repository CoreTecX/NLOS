/* ============================================================================
 * PenumbraSight · SD 卡帧读取层的主机自检
 * src/software/test_sd_frame.c
 *
 * 用法：test_sd_frame <file.psf> [block] [--dump-raw <out.bin>]
 *   例：test_sd_frame tests/vectors/sd_frame/tiny8x8.psf 2
 *       test_sd_frame build/psf/xxx_f000.psf 10 --dump-raw src/software/ps_tmp/sd_frame/xxx.raw
 *
 * 这一趟测什么（都是能失败的判据，不是「打印出来看看」）
 * ---------------------------------------------------------------------------
 *   A. 两条读取路径互相比对：流式（ps_sd_frame_read_file，不整文件缓冲）与
 *      内存（parse + verify + payload_codes）必须给出**同一组** crc32/sum/sumsq/
 *      min/max，并且填出的码字缓冲**逐字相同**。两条路径各写一遍校验和逻辑，
 *      比对能查出「某一侧漏了半块」这类只在长文件上出现的错。
 *   B. 文件自洽：头里的 crc32/sum/sumsq/min/max 与实测值一致。
 *   C. 块和：ps_sd_frame_block_sum_rggb 的结果与测试内**另一套写法**
 *       （按原始光栅序逐像素分类累加，不经平面抽取）逐值相同 —— 这是对
 *       RGGB 相位与双绿求和的交叉验证，索引写错就会在这里分叉。
 *   D. 四类负例：魔数错、版本错、截断、payload 翻一个字节。
 *       每一类都必须返回**指定**的错误码（不是「非零就行」）：
 *       返回了别的错码说明检查顺序或判据写错了，同样算失败。
 *   E. --dump-raw：把读回的 payload 写成裸 u16 小端，供 Python 侧做**逐值**比对
 *       （CRC 相等只是概率意义，逐字节相等才是同一份数据）。
 *
 * 边界：本测试不碰 SD 卡、不碰 DMA、不碰板卡。「能读文件」不等于「能上板跑」。
 * ==========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "psm_format.h"   /* ps_crc32：块和数组的 CRC 与 Python zlib.crc32 同参数 */
#include "sd_frame.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(int cond, const char *what)
{
    if (cond) {
        g_pass++;
    } else {
        g_fail++;
        printf("  FAIL: %s\n", what);
    }
}

static unsigned char *read_whole_file(const char *path, size_t *len_out)
{
    FILE *f = fopen(path, "rb");
    unsigned char *buf;
    long n;
    size_t got;
    if (f == NULL) {
        printf("  FAIL: cannot open %s\n", path);
        g_fail++;
        return NULL;
    }
    if (fseek(f, 0L, SEEK_END) != 0) { fclose(f); return NULL; }
    n = ftell(f);
    if (n <= 0) { fclose(f); return NULL; }
    rewind(f);
    buf = (unsigned char *)malloc((size_t)n);
    if (buf == NULL) { fclose(f); return NULL; }
    got = fread(buf, 1u, (size_t)n, f);
    fclose(f);
    if (got != (size_t)n) {
        free(buf);
        return NULL;
    }
    *len_out = got;
    return buf;
}

static const char *base_name_of(const char *p)
{
    /* 机读行里只放文件名：路径可能含非 ASCII 字符，混进机读行会让
     * 父进程的键值解析面临编码问题（数值键本身永远是 ASCII）。 */
    const char *b = p;
    for (; *p != '\0'; p++) {
        if (*p == '\\' || *p == '/') {
            b = p + 1;
        }
    }
    return b;
}

/* 独立实现的块和参考：按原始光栅序逐像素分类累加（不抽平面、不用 reshape 思路）。
 * 与库里的实现是两套写法，逐值相同才有意义。 */
static void ref_block_sum_rggb(const uint16_t *codes, uint32_t side, uint32_t block,
                               uint32_t *out)
{
    uint32_t plane = side / 2u;
    uint32_t wall_n = plane / block;
    uint32_t r;
    uint32_t c;
    memset(out, 0, sizeof(uint32_t) * (size_t)3u * (size_t)wall_n * (size_t)wall_n);
    for (r = 0u; r < side; r++) {
        for (c = 0u; c < side; c++) {
            uint32_t pr = r / 2u;
            uint32_t pc = c / 2u;
            uint32_t br = pr / block;
            uint32_t bc = pc / block;
            uint32_t ch;
            if (((r & 1u) == 0u) && ((c & 1u) == 0u)) {
                ch = 0u;                       /* R */
            } else if (((r & 1u) == 1u) && ((c & 1u) == 1u)) {
                ch = 2u;                       /* B */
            } else {
                ch = 1u;                       /* G1 与 G2 都进 G，按和累加 */
            }
            out[ch * wall_n * wall_n + br * wall_n + bc] += (uint32_t)codes[(size_t)r * side + c];
        }
    }
}

int main(int argc, char **argv)
{
    const char *path = NULL;
    const char *base_name = NULL;
    const char *dump_path = NULL;
    uint32_t block = 10u;
    unsigned char *file = NULL;
    size_t file_len = 0u;
    ps_sd_frame_header_t hdr_m;    /* 内存路径 */
    ps_sd_frame_header_t hdr_s;    /* 流式路径 */
    ps_sd_frame_check_t chk_m;
    ps_sd_frame_check_t chk_s;
    uint16_t *codes_m = NULL;
    uint16_t *codes_s = NULL;
    size_t words;
    char reason[256];
    int rc;

    if (argc < 2) {
        printf("usage: test_sd_frame <file.psf> [block] [--dump-raw out.bin]\n");
        return 2;
    }
    path = argv[1];
    base_name = base_name_of(path);
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--dump-raw") == 0 && i + 1 < argc) {
            dump_path = argv[++i];
        } else {
            block = (uint32_t)strtoul(argv[i], NULL, 10);
        }
    }

    printf("==== test_sd_frame: %s (block=%u) ====\n", path, block);

    file = read_whole_file(path, &file_len);
    if (file == NULL) {
        printf("RESULT: FAIL (cannot read file)\n");
        return 1;
    }

    /* ---------------- A/B: 内存路径 ---------------- */
    rc = ps_sd_frame_parse_header(file, file_len, &hdr_m, reason, sizeof(reason));
    check(rc == PS_SD_OK, "parse_header on a good file must succeed");
    if (rc != PS_SD_OK) {
        printf("  reason: %s\n", reason);
        free(file);
        printf("RESULT: PASS %d / FAIL %d\n", g_pass, g_fail);
        return 1;
    }

    words = (size_t)hdr_m.width * (size_t)hdr_m.height;
    codes_m = (uint16_t *)malloc(words * sizeof(uint16_t));
    codes_s = (uint16_t *)malloc(words * sizeof(uint16_t));
    if (codes_m == NULL || codes_s == NULL) {
        printf("RESULT: FAIL (out of memory)\n");
        free(file);
        free(codes_m);
        free(codes_s);
        return 1;
    }
    memset(codes_m, 0xAA, words * sizeof(uint16_t));
    memset(codes_s, 0x55, words * sizeof(uint16_t));

    rc = ps_sd_frame_verify(file, file_len, &hdr_m, &chk_m);
    check(rc == PS_SD_OK, "verify: header checksums must match the payload");
    check(chk_m.words == (uint32_t)words, "verify: word count == width*height");
    check(chk_m.min == hdr_m.payload_min, "verify: min matches header");
    check(chk_m.max == hdr_m.payload_max, "verify: max matches header");
    check(chk_m.crc32 == hdr_m.payload_crc32, "verify: crc32 matches header");

    rc = ps_sd_frame_payload_codes(file, file_len, &hdr_m, codes_m, words);
    check(rc == PS_SD_OK, "payload_codes: fill caller buffer");

    /* ---------------- A: 流式路径 ---------------- */
    rc = ps_sd_frame_read_file(path, &hdr_s, codes_s, words, &chk_s, reason, sizeof(reason));
    check(rc == PS_SD_OK, "read_file (streaming) must succeed");
    if (rc != PS_SD_OK) {
        printf("  reason: %s\n", reason);
    }
    check(memcmp(&hdr_m, &hdr_s, sizeof(hdr_m)) == 0, "streaming and memory header parse identically");
    check(memcmp(&chk_m, &chk_s, sizeof(chk_m)) == 0, "streaming and memory checksums agree");
    check(memcmp(codes_m, codes_s, words * sizeof(uint16_t)) == 0,
          "streaming and memory payload are word-for-word identical");

    /* ---------------- 头部语义 ---------------- */
    check(hdr_m.version == PS_SD_VERSION, "version == 1");
    check(hdr_m.header_bytes == PS_SD_HEADER_BYTES, "header_bytes == 128");
    check(hdr_m.container_bits == PS_SD_CONTAINER_BITS, "container_bits == 16");
    check(hdr_m.value_packing == PS_SD_PACKING_PACKED || hdr_m.value_packing == PS_SD_PACKING_FULLSCALE,
          "value_packing is a known value");
    check(hdr_m.bayer_phase <= PS_SD_BAYER_GBRG, "bayer_phase in 0..3");
    check(hdr_m.payload_bytes == (uint32_t)((size_t)hdr_m.width * hdr_m.height * 2u),
          "payload_bytes == width*height*2");

    /* ---------------- C: 块和（两套写法互相比对） ---------------- */
    {
        uint32_t plane = hdr_m.width / 2u;
        int block_ok = (block > 0u) && ((plane % block) == 0u);
        if (block_ok) {
            uint32_t wall_n = plane / block;
            size_t n = (size_t)3u * wall_n * wall_n;
            uint32_t *sums = (uint32_t *)malloc(n * sizeof(uint32_t));
            uint32_t *refs = (uint32_t *)malloc(n * sizeof(uint32_t));
            if (sums == NULL || refs == NULL) {
                printf("RESULT: FAIL (out of memory for block sums)\n");
                g_fail++;
            } else {
                rc = ps_sd_frame_block_sum_rggb(codes_s, hdr_m.width, block, sums, n);
                check(rc == PS_SD_OK, "block_sum_rggb returns OK");
                ref_block_sum_rggb(codes_s, hdr_m.width, block, refs);
                check(memcmp(sums, refs, n * sizeof(uint32_t)) == 0,
                      "block sums: library == independent reference");
                {
                    uint64_t total = 0u;
                    uint64_t ch[3] = {0u, 0u, 0u};
                    uint32_t mx = 0u;
                    int bits;
                    size_t i;
                    size_t per = (size_t)wall_n * (size_t)wall_n;
                    for (i = 0u; i < n; i++) {
                        total += (uint64_t)sums[i];
                        ch[i / per] += (uint64_t)sums[i];
                        if (sums[i] > mx) { mx = sums[i]; }
                    }
                    bits = 0;
                    while ((mx >> bits) != 0u && bits < 32) { bits++; }
                    if (bits == 0) { bits = 1; }
                    printf("SD_FRAME_BLOCK block=%u wall_n=%u total=%llu ch0=%llu ch1=%llu ch2=%llu "
                           "max=%u bits=%d crc32=0x%08X\n",
                           block, wall_n, (unsigned long long)total,
                           (unsigned long long)ch[0], (unsigned long long)ch[1],
                           (unsigned long long)ch[2], mx, bits,
                           ps_crc32(0u, (const uint8_t *)sums, n * sizeof(uint32_t)));
                }
                free(sums);
                free(refs);
            }
        } else {
            printf("  note: block=%u does not divide plane side %u; block-sum checks skipped\n",
                   block, plane);
        }
    }

    /* ---------------- D: 四类负例 ---------------- */
    {
        unsigned char *tmp = (unsigned char *)malloc(file_len);
        if (tmp == NULL) {
            check(0, "negative tests need a scratch buffer");
        } else {
            ps_sd_frame_header_t h2;
            char r2[256];

            memcpy(tmp, file, file_len);
            tmp[0] = (unsigned char)'X';
            rc = ps_sd_frame_parse_header(tmp, file_len, &h2, r2, sizeof(r2));
            check(rc == PS_SD_ERR_BAD_HEADER, "bad magic -> PS_SD_ERR_BAD_HEADER");

            memcpy(tmp, file, file_len);
            tmp[PS_SD_OFF_VERSION] = 9u;
            tmp[PS_SD_OFF_VERSION + 1u] = 0u;
            rc = ps_sd_frame_parse_header(tmp, file_len, &h2, r2, sizeof(r2));
            check(rc == PS_SD_ERR_BAD_HEADER, "bad version -> PS_SD_ERR_BAD_HEADER");

            /* 截断有两种：头都没读全（parse 拦）、头在而 payload 短了（verify 拦）。
             * 只把文件砍 4 字节只能测到后者 —— 头还完整时 parse 本就该返回 OK，
             * 所以这里显式给 parse 一个不足 128 字节的输入。 */
            rc = ps_sd_frame_parse_header(file, 100u, &h2, r2, sizeof(r2));
            check(rc == PS_SD_ERR_TRUNCATED, "buffer shorter than the header -> parse says TRUNCATED");

            rc = ps_sd_frame_verify(file, file_len - 4u, &hdr_m, &chk_m);
            check(rc == PS_SD_ERR_TRUNCATED, "short file -> verify says TRUNCATED");

            memcpy(tmp, file, file_len);
            tmp[PS_SD_HEADER_BYTES + 1u] ^= 0x01u;   /* 翻 payload 第 1 个字的高字节 */
            rc = ps_sd_frame_verify(tmp, file_len, &hdr_m, &chk_m);
            check(rc == PS_SD_ERR_CHECKSUM, "one flipped payload byte -> PS_SD_ERR_CHECKSUM");

            memcpy(tmp, file, file_len);
            tmp[PS_SD_HEADER_BYTES + 1u] ^= 0x01u;
            rc = ps_sd_frame_read_file(path, &h2, NULL, 0u, &chk_m, r2, sizeof(r2));
            check(rc == PS_SD_OK, "verify_file path still OK on the pristine file");

            free(tmp);
        }
    }

    /* ---------------- E: dump ---------------- */
    if (dump_path != NULL) {
        FILE *f = fopen(dump_path, "wb");
        if (f == NULL) {
            check(0, "cannot open --dump-raw output");
        } else {
            size_t wrote = fwrite(codes_s, sizeof(uint16_t), words, f);
            fclose(f);
            check(wrote == words, "--dump-raw wrote every word");
            printf("  dump-raw -> %s (%llu words)\n", dump_path, (unsigned long long)words);
        }
    }

    printf("SD_FRAME_READ file=%s w=%u h=%u valid_bits=%u container_bits=%u bayer_phase=%u "
           "packing=%u rule_ok=%u frame_index=%u num=%llu den=%llu crc32=0x%08X "
           "sum=%llu sumsq=%llu min=%u max=%u median_code=%u p999_code=%u clipped=%u\n",
           base_name, hdr_m.width, hdr_m.height, hdr_m.valid_bits, hdr_m.container_bits,
           hdr_m.bayer_phase, hdr_m.value_packing, hdr_m.scale_rule_ok, hdr_m.frame_index,
           (unsigned long long)hdr_m.scale_num, (unsigned long long)hdr_m.scale_den,
           chk_s.crc32, (unsigned long long)chk_s.sum, (unsigned long long)chk_s.sumsq,
           chk_s.min, chk_s.max, hdr_m.median_code, hdr_m.p999_code, hdr_m.clipped_pixels);

    free(file);
    free(codes_m);
    free(codes_s);

    printf("RESULT: PASS %d / FAIL %d\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
