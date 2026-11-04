/* ============================================================================
 * PenumbraSight · SD 卡帧（.psf）板端读取层实现
 * src/software/sd_frame/sd_frame.c
 *
 * 逐字段口径的权威是 docs/sd_frame_format.md §2 的字段表；写入端是
 * build/scripts/pack_paper_frame.py。三处（文档 / 本文件 / 写入端）任一改动，
 * 另两处必须同时改 —— 字段偏移错一位不会报错，只会让 width 变成 height。
 *
 * 两条实现路径，同一个校验口径
 * ---------------------------------------------------------------------------
 *   · 内存路径：调用方已有整份文件字节（mmap / DMA / 板上缓存），
 *     ps_sd_frame_parse_header + ps_sd_frame_verify + ps_sd_frame_payload_codes；
 *   · 文件路径：ps_sd_frame_read_file 自己按块流式读，**不整文件缓冲**，
 *     CRC 链式累加、sum/sumsq/min/max 增量累加（见下）。
 *   两条路径对同一份文件必须给出同一组 crc32/sum/sumsq/min/max ——
 *   主机自检查的就是这一条（test_sd_frame.c 会两条都跑一遍再互相比对）。
 *
 * 为什么流式而不是「读全再算」
 * ---------------------------------------------------------------------------
 *   一帧 7,372,800 字节。整文件缓冲意味着要么 malloc（本层不用动态分配），
 *   要么在栈上开 7 MB（板端栈只有几百 KB）。CRC32 天然可链式（ps_crc32 的入参
 *   就是上一块的返回值），sum/sumsq/min/max 也都能增量累加，所以流式不需要
 *   任何妥协：校验口径与内存路径完全相同。
 *
 * 为什么 payload 的字节序要按头里的 flags 判
 * ---------------------------------------------------------------------------
 *   头部字段一律小端（格式如此规定），但 payload 的字节序被单独记在 flags.bit0。
 *   本层只声明支持小端 payload；bit0 为 0 时直接判 BAD_HEADER 而不是猜 ——
 *   猜错的后果是每个像素都是 256 倍的错值，而且肉眼看不出来。
 * ==========================================================================*/

#include "sd_frame.h"

#include <stdio.h>
#include <string.h>

#include "psm_format.h"   /* ps_crc32：与 Python zlib.crc32 同参数的唯一实现 */

/* ------------------------------------------------------------ 小端取字段 */

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t rd_u64(const uint8_t *p)
{
    uint64_t lo = (uint64_t)rd_u32(p);
    uint64_t hi = (uint64_t)rd_u32(p + 4);
    return lo | (hi << 32);
}

/* ------------------------------------------------------------ 内部工具 */

static void set_reason(char *reason, size_t cap, const char *text)
{
    if (reason != NULL && cap > 0u) {
        size_t i = 0u;
        while (text[i] != '\0' && i + 1u < cap) {
            reason[i] = text[i];
            ++i;
        }
        reason[i] = '\0';
    }
}

static int magic_ok(const uint8_t *p)
{
    return p[0] == (uint8_t)'P' && p[1] == (uint8_t)'S' &&
           p[2] == (uint8_t)'D' && p[3] == (uint8_t)'F';
}

/* ------------------------------------------------------------ 解析头部 */

int ps_sd_frame_parse_header(const uint8_t *buf, size_t len,
                             ps_sd_frame_header_t *hdr,
                             char *reason, size_t reason_cap)
{
    uint64_t expect_payload;
    if (reason != NULL && reason_cap > 0u) {
        reason[0] = '\0';
    }
    if (buf == NULL || hdr == NULL) {
        set_reason(reason, reason_cap, "null buffer or header out");
        return PS_SD_ERR_ARGS;
    }
    /* 先把出参清零：结构体里有填充字节，不清零的话「两次解析结果 byte 相同」
     * 这种比对会被未初始化的填充位搞成随机结果（主机自检里真的踩到过）。 */
    (void)memset(hdr, 0, sizeof(*hdr));
    if (len < (size_t)PS_SD_HEADER_BYTES) {
        set_reason(reason, reason_cap, "fewer than 128 bytes: not a psf file");
        return PS_SD_ERR_TRUNCATED;
    }
    if (!magic_ok(buf + PS_SD_OFF_MAGIC)) {
        set_reason(reason, reason_cap, "bad magic (want 'PSDF')");
        return PS_SD_ERR_BAD_HEADER;
    }

    hdr->version        = (uint32_t)rd_u16(buf + PS_SD_OFF_VERSION);
    hdr->header_bytes   = (uint32_t)rd_u16(buf + PS_SD_OFF_HEADER_BYTES);
    hdr->width          = rd_u32(buf + PS_SD_OFF_WIDTH);
    hdr->height         = rd_u32(buf + PS_SD_OFF_HEIGHT);
    hdr->valid_bits     = (uint32_t)rd_u16(buf + PS_SD_OFF_VALID_BITS);
    hdr->container_bits = (uint32_t)rd_u16(buf + PS_SD_OFF_CONTAINER_BITS);
    hdr->bayer_phase    = (uint32_t)buf[PS_SD_OFF_BAYER_PHASE];
    hdr->value_packing  = (uint32_t)buf[PS_SD_OFF_VALUE_PACKING];
    hdr->flags          = (uint32_t)rd_u16(buf + PS_SD_OFF_FLAGS);
    hdr->frame_index    = rd_u32(buf + PS_SD_OFF_FRAME_INDEX);
    hdr->payload_bytes  = rd_u32(buf + PS_SD_OFF_PAYLOAD_BYTES);
    hdr->scale_num      = rd_u64(buf + PS_SD_OFF_SCALE_NUM);
    hdr->scale_den      = rd_u64(buf + PS_SD_OFF_SCALE_DEN);
    hdr->payload_crc32  = rd_u32(buf + PS_SD_OFF_PAYLOAD_CRC32);
    hdr->clipped_pixels = rd_u32(buf + PS_SD_OFF_CLIPPED_PIXELS);
    hdr->payload_sum    = rd_u64(buf + PS_SD_OFF_PAYLOAD_SUM);
    hdr->payload_sumsq  = rd_u64(buf + PS_SD_OFF_PAYLOAD_SUMSQ);
    hdr->payload_min    = (uint32_t)rd_u16(buf + PS_SD_OFF_PAYLOAD_MIN);
    hdr->payload_max    = (uint32_t)rd_u16(buf + PS_SD_OFF_PAYLOAD_MAX);
    hdr->median_code    = (uint32_t)rd_u16(buf + PS_SD_OFF_MEDIAN_CODE);
    hdr->p999_code      = (uint32_t)rd_u16(buf + PS_SD_OFF_P999_CODE);
    hdr->scale_rule_ok  = (uint32_t)buf[PS_SD_OFF_SCALE_RULE_OK];
    hdr->source_median  = rd_u32(buf + PS_SD_OFF_SOURCE_MEDIAN);
    hdr->source_p999    = rd_u32(buf + PS_SD_OFF_SOURCE_P999);

    if (hdr->version != PS_SD_VERSION) {
        set_reason(reason, reason_cap, "unsupported format version");
        return PS_SD_ERR_BAD_HEADER;
    }
    if (hdr->header_bytes != PS_SD_HEADER_BYTES) {
        set_reason(reason, reason_cap, "header_bytes != 128");
        return PS_SD_ERR_BAD_HEADER;
    }
    if (hdr->width == 0u || hdr->height == 0u) {
        set_reason(reason, reason_cap, "zero width or height");
        return PS_SD_ERR_BAD_HEADER;
    }
    if (hdr->container_bits != PS_SD_CONTAINER_BITS) {
        /* 只支持 16 bit 容器：raw_preproc 的 s_axis_tdata 就是 16 bit。
         * 若哪天要支持 8 bit 容器，需要同时改 payload 填充与块和两处。 */
        set_reason(reason, reason_cap, "container_bits != 16 (unsupported)");
        return PS_SD_ERR_BAD_HEADER;
    }
    if (hdr->valid_bits == 0u || hdr->valid_bits > PS_SD_VALID_BITS_MAX) {
        set_reason(reason, reason_cap, "valid_bits out of range (1..16)");
        return PS_SD_ERR_BAD_HEADER;
    }
    if (hdr->valid_bits > hdr->container_bits) {
        set_reason(reason, reason_cap, "valid_bits > container_bits");
        return PS_SD_ERR_BAD_HEADER;
    }
    if (hdr->bayer_phase > PS_SD_BAYER_GBRG) {
        set_reason(reason, reason_cap, "bayer_phase out of range (0..3)");
        return PS_SD_ERR_BAD_HEADER;
    }
    if (hdr->value_packing != PS_SD_PACKING_PACKED &&
        hdr->value_packing != PS_SD_PACKING_FULLSCALE) {
        set_reason(reason, reason_cap, "value_packing out of range (0..1)");
        return PS_SD_ERR_BAD_HEADER;
    }
    if ((hdr->flags & PS_SD_FLAG_LITTLE_ENDIAN) == 0u) {
        set_reason(reason, reason_cap, "payload is not flagged little-endian");
        return PS_SD_ERR_BAD_HEADER;
    }
    if (hdr->scale_num == 0u || hdr->scale_den == 0u) {
        set_reason(reason, reason_cap, "scale_num or scale_den is zero");
        return PS_SD_ERR_BAD_HEADER;
    }

    expect_payload = (uint64_t)hdr->width * (uint64_t)hdr->height *
                     (uint64_t)hdr->container_bits / 8u;
    if ((uint64_t)hdr->payload_bytes != expect_payload) {
        set_reason(reason, reason_cap, "payload_bytes != width*height*container_bits/8");
        return PS_SD_ERR_BAD_HEADER;
    }
    return PS_SD_OK;
}

/* ------------------------------------------------------------ 校验 payload */
/*
 * 内存路径与文件路径共用这一段「逐块喂」的累加器，避免出现两套口径。
 * crc 的初值 0 与 zlib.crc32(..., 0) 同义（见 psm_crc32.c）。
 */
typedef struct {
    uint32_t crc;
    uint64_t sum;
    uint64_t sumsq;
    uint32_t vmin;
    uint32_t vmax;
    uint32_t words;
} ps_sd_accum_t;

static void accum_init(ps_sd_accum_t *a)
{
    a->crc = 0u;
    a->sum = 0u;
    a->sumsq = 0u;
    a->vmin = 0xFFFFFFFFu;
    a->vmax = 0u;
    a->words = 0u;
}

/* 喂入一段 payload 字节（长度必须是偶数；不足 2 字节的尾巴由调用方拦下）。 */
static void accum_feed(ps_sd_accum_t *a, const uint8_t *p, size_t nbytes)
{
    size_t i;
    a->crc = ps_crc32(a->crc, p, nbytes);
    for (i = 0u; i + 1u < nbytes; i += 2u) {
        uint32_t w = (uint32_t)rd_u16(p + i);
        a->sum += (uint64_t)w;
        a->sumsq += (uint64_t)w * (uint64_t)w;
        if (w < a->vmin) { a->vmin = w; }
        if (w > a->vmax) { a->vmax = w; }
        a->words += 1u;
    }
}

static void accum_store(const ps_sd_accum_t *a, ps_sd_frame_check_t *out)
{
    if (out == NULL) {
        return;
    }
    (void)memset(out, 0, sizeof(*out));   /* 同上：填充字节不留随机值 */
    out->crc32 = a->crc;
    out->sum = a->sum;
    out->sumsq = a->sumsq;
    out->min = (a->words == 0u) ? 0u : a->vmin;
    out->max = a->vmax;
    out->words = a->words;
}

static int accum_matches_header(const ps_sd_accum_t *a,
                                const ps_sd_frame_header_t *hdr,
                                char *reason, size_t reason_cap)
{
    if (a->crc != hdr->payload_crc32) {
        set_reason(reason, reason_cap, "crc32 mismatch");
        return PS_SD_ERR_CHECKSUM;
    }
    if (a->sum != hdr->payload_sum) {
        set_reason(reason, reason_cap, "payload sum mismatch");
        return PS_SD_ERR_CHECKSUM;
    }
    if (a->sumsq != hdr->payload_sumsq) {
        set_reason(reason, reason_cap, "payload sumsq mismatch");
        return PS_SD_ERR_CHECKSUM;
    }
    if ((uint32_t)a->vmin != hdr->payload_min) {
        set_reason(reason, reason_cap, "payload min mismatch");
        return PS_SD_ERR_CHECKSUM;
    }
    if (a->vmax != hdr->payload_max) {
        set_reason(reason, reason_cap, "payload max mismatch");
        return PS_SD_ERR_CHECKSUM;
    }
    return PS_SD_OK;
}

int ps_sd_frame_verify(const uint8_t *buf, size_t len,
                       const ps_sd_frame_header_t *hdr,
                       ps_sd_frame_check_t *out)
{
    ps_sd_accum_t a;
    if (buf == NULL || hdr == NULL) {
        return PS_SD_ERR_ARGS;
    }
    if (len < (size_t)hdr->header_bytes + (size_t)hdr->payload_bytes) {
        return PS_SD_ERR_TRUNCATED;
    }
    accum_init(&a);
    accum_feed(&a, buf + hdr->header_bytes, (size_t)hdr->payload_bytes);
    accum_store(&a, out);
    return accum_matches_header(&a, hdr, NULL, 0u);
}

/* ------------------------------------------------------------ 填充码字缓冲 */

int ps_sd_frame_payload_codes(const uint8_t *buf, size_t len,
                              const ps_sd_frame_header_t *hdr,
                              uint16_t *dst, size_t capacity_words)
{
    size_t words;
    size_t i;
    const uint8_t *p;
    if (buf == NULL || hdr == NULL || dst == NULL) {
        return PS_SD_ERR_ARGS;
    }
    words = (size_t)hdr->width * (size_t)hdr->height;
    if (capacity_words < words) {
        return PS_SD_ERR_BUFFER;
    }
    if (len < (size_t)hdr->header_bytes + (size_t)hdr->payload_bytes) {
        return PS_SD_ERR_TRUNCATED;
    }
    p = buf + hdr->header_bytes;
    for (i = 0u; i < words; i++) {
        dst[i] = rd_u16(p + i * 2u);
    }
    return PS_SD_OK;
}

/* ------------------------------------------------------------ 文件入口 */

/* 流式读的块大小：8192 字节 = 4096 个 16 bit 字。
 * 取这个数的理由：能在板端栈上放得下（8 KB），又是 SD 扇区（512 B）的整数倍，
 * 于是每次 fread 都对上整扇区，不会出现「读 1 字节」的碎片访问。 */
#define PS_SD_CHUNK_BYTES 8192u

int ps_sd_frame_read_file(const char *path,
                          ps_sd_frame_header_t *hdr,
                          uint16_t *dst, size_t dst_capacity_words,
                          ps_sd_frame_check_t *check,
                          char *reason, size_t reason_cap)
{
    FILE *f;
    uint8_t head[PS_SD_HEADER_BYTES];
    uint8_t chunk[PS_SD_CHUNK_BYTES];
    ps_sd_frame_header_t local;
    ps_sd_frame_header_t *h = (hdr != NULL) ? hdr : &local;
    ps_sd_accum_t a;
    uint64_t remaining;
    uint64_t expect_words;
    size_t nread;
    int rc;

    if (reason != NULL && reason_cap > 0u) {
        reason[0] = '\0';
    }
    if (path == NULL) {
        set_reason(reason, reason_cap, "null path");
        return PS_SD_ERR_ARGS;
    }
    if (dst == NULL && dst_capacity_words != 0u) {
        set_reason(reason, reason_cap, "dst is null but capacity is non-zero");
        return PS_SD_ERR_ARGS;
    }

    f = fopen(path, "rb");
    if (f == NULL) {
        set_reason(reason, reason_cap, "cannot open file");
        return PS_SD_ERR_IO;
    }

    nread = fread(head, 1u, sizeof(head), f);
    if (nread != sizeof(head)) {
        fclose(f);
        set_reason(reason, reason_cap, "short read on header");
        return PS_SD_ERR_TRUNCATED;
    }
    rc = ps_sd_frame_parse_header(head, sizeof(head), h, reason, reason_cap);
    if (rc != PS_SD_OK) {
        fclose(f);
        return rc;
    }

    /* 缓冲区容量先判，再开始搬 —— 避免搬到一半才发现装不下。 */
    expect_words = (uint64_t)h->width * (uint64_t)h->height;
    if (dst != NULL && (uint64_t)dst_capacity_words < expect_words) {
        fclose(f);
        set_reason(reason, reason_cap, "destination buffer too small");
        return PS_SD_ERR_BUFFER;
    }

    accum_init(&a);
    remaining = (uint64_t)h->payload_bytes;
    while (remaining > 0u) {
        size_t want = (remaining < (uint64_t)sizeof(chunk))
                          ? (size_t)remaining : sizeof(chunk);
        nread = fread(chunk, 1u, want, f);
        if (nread == 0u || (nread & 1u) != 0u) {
            /* 奇数个字节一定是短读：payload 长度恒为偶数（16 bit 容器）。 */
            fclose(f);
            set_reason(reason, reason_cap, "payload shorter than declared");
            return PS_SD_ERR_TRUNCATED;
        }
        if (dst != NULL) {
            size_t i;
            for (i = 0u; i + 1u < nread; i += 2u) {
                dst[a.words + i / 2u] = rd_u16(chunk + i);
            }
        }
        accum_feed(&a, chunk, nread);
        remaining -= (uint64_t)nread;
        if (nread < want && remaining > 0u) {
            fclose(f);
            set_reason(reason, reason_cap, "payload shorter than declared");
            return PS_SD_ERR_TRUNCATED;
        }
    }

    /* 多出来的字节说明这不是「一帧一文件」，必须拦住：
     * 若允许，两帧首尾相接的文件会被当成一帧读走，画面下半截是下一帧的数据。 */
    if (fgetc(f) != EOF) {
        fclose(f);
        set_reason(reason, reason_cap, "trailing bytes after payload");
        return PS_SD_ERR_BAD_HEADER;
    }
    fclose(f);

    accum_store(&a, check);
    return accum_matches_header(&a, h, reason, reason_cap);
}

int ps_sd_frame_verify_file(const char *path, ps_sd_frame_header_t *hdr,
                            ps_sd_frame_check_t *check,
                            char *reason, size_t reason_cap)
{
    return ps_sd_frame_read_file(path, hdr, NULL, 0u, check, reason, reason_cap);
}

/* ------------------------------------------------------------ 语义辅助 */

int ps_sd_frame_dequantize(const ps_sd_frame_header_t *hdr, uint16_t code,
                           uint64_t *num_out, uint64_t *den_out)
{
    if (hdr == NULL || num_out == NULL || den_out == NULL) {
        return PS_SD_ERR_ARGS;
    }
    if (hdr->scale_num == 0u || hdr->scale_den == 0u) {
        return PS_SD_ERR_BAD_HEADER;
    }
    /* v_hat = code * den / num（有理数，不做除法、不用浮点） */
    *num_out = (uint64_t)code * hdr->scale_den;
    *den_out = hdr->scale_num;
    return PS_SD_OK;
}

int ps_sd_frame_block_sum_rggb(const uint16_t *codes, uint32_t side,
                               uint32_t block, uint32_t *out,
                               size_t out_words)
{
    uint32_t plane;
    uint32_t wall_n;
    uint32_t br;
    uint32_t bc;
    size_t need;

    if (codes == NULL || out == NULL || block == 0u) {
        return PS_SD_ERR_ARGS;
    }
    if (side == 0u || (side % 2u) != 0u) {
        return PS_SD_ERR_ARGS;   /* RGGB 解交织要求偶数边长 */
    }
    plane = side / 2u;
    if ((plane % block) != 0u) {
        return PS_SD_ERR_ARGS;
    }
    wall_n = plane / block;
    need = (size_t)3u * (size_t)wall_n * (size_t)wall_n;
    if (out_words < need) {
        return PS_SD_ERR_BUFFER;
    }

    for (br = 0u; br < wall_n; br++) {
        for (bc = 0u; bc < wall_n; bc++) {
            uint32_t sum_r = 0u;
            uint32_t sum_g = 0u;
            uint32_t sum_b = 0u;
            uint32_t ii;
            for (ii = 0u; ii < block; ii++) {
                uint32_t i = br * block + ii;
                uint32_t jj;
                const uint16_t *row0 = codes + (size_t)(2u * i) * (size_t)side;
                const uint16_t *row1 = row0 + (size_t)side;
                for (jj = 0u; jj < block; jj++) {
                    uint32_t j = bc * block + jj;
                    uint32_t c0 = 2u * j;
                    uint32_t c1 = c0 + 1u;
                    sum_r += (uint32_t)row0[c0];                  /* R  = raw[2i  ,2j  ] */
                    sum_g += (uint32_t)row0[c1] + (uint32_t)row1[c0]; /* G1 + G2         */
                    sum_b += (uint32_t)row1[c1];                  /* B  = raw[2i+1,2j+1] */
                }
            }
            out[0u * (size_t)wall_n * (size_t)wall_n + (size_t)br * wall_n + bc] = sum_r;
            out[1u * (size_t)wall_n * (size_t)wall_n + (size_t)br * wall_n + bc] = sum_g;
            out[2u * (size_t)wall_n * (size_t)wall_n + (size_t)br * wall_n + bc] = sum_b;
        }
    }
    return PS_SD_OK;
}
