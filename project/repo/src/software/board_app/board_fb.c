/* ============================================================================
 * PenumbraSight · 结果 -> PS 显示帧缓冲 的格式转换实现
 * src/software/board_app/board_fb.c
 *
 * 定标用 int64 中间量：lo/hi 是定标端点，输入是任意 int32，二者相减可能溢出
 * int32（例如 lo=-2^31）。先转 int64 再做乘除，避免 UB。
 * ==========================================================================*/

#include "board_fb.h"

#include <stddef.h>

uint32_t ps_board_fb_pack_xrgb(uint32_t r, uint32_t g, uint32_t b)
{
    return ((r & 0xFFu) << 16) | ((g & 0xFFu) << 8) | (b & 0xFFu);
}

void ps_board_fb_fill(uint32_t *fb, uint32_t words, uint32_t color)
{
    uint32_t i;
    if (!fb) { return; }
    for (i = 0u; i < words; ++i) { fb[i] = color; }
}

void ps_board_fb_fill_gray(uint32_t *fb, uint32_t words)
{
    ps_board_fb_fill(fb, words,
                     ps_board_fb_pack_xrgb(PS_BOARD_FB_GRAY_R,
                                           PS_BOARD_FB_GRAY_G,
                                           PS_BOARD_FB_GRAY_B));
}

/* 把一个 int32 定标到 [0,255]，端点饱和。 */
static uint32_t scale_u8(int32_t v, int32_t lo, int32_t hi)
{
    long long span = (long long)hi - (long long)lo;
    long long x = (long long)v - (long long)lo;
    long long q;
    if (span <= 0) { return 0u; }
    if (x <= 0) { return 0u; }
    if (x >= span) { return 255u; }
    q = (x * 255ll) / span;
    if (q < 0ll) { q = 0ll; }
    if (q > 255ll) { q = 255ll; }
    return (uint32_t)q;
}

int ps_board_fb_pack_mono_xrgb(const int32_t *plane, uint32_t rows, uint32_t cols,
                               uint32_t stride_words, uint32_t *fb,
                               int32_t lo, int32_t hi)
{
    uint32_t y, x;
    if (!plane || !fb || rows == 0u || cols == 0u || stride_words < cols) {
        return PS_BOARD_FB_ERR_ARGS;
    }
    if (hi <= lo) { return PS_BOARD_FB_ERR_RANGE; }

    for (y = 0u; y < rows; ++y) {
        const int32_t *src = plane + (size_t)y * cols;
        uint32_t *dst = fb + (size_t)y * stride_words;
        for (x = 0u; x < cols; ++x) {
            uint32_t g = scale_u8(src[x], lo, hi);
            dst[x] = ps_board_fb_pack_xrgb(g, g, g);
        }
    }
    return PS_BOARD_FB_OK;
}

int ps_board_fb_pack_rgb_xrgb(const int32_t *r, const int32_t *g, const int32_t *b,
                              uint32_t rows, uint32_t cols,
                              uint32_t stride_words, uint32_t *fb,
                              int32_t lo, int32_t hi)
{
    uint32_t y, x;
    if (!fb || rows == 0u || cols == 0u || stride_words < cols) {
        return PS_BOARD_FB_ERR_ARGS;
    }
    if (hi <= lo) { return PS_BOARD_FB_ERR_RANGE; }

    for (y = 0u; y < rows; ++y) {
        const int32_t *rp = r ? r + (size_t)y * cols : NULL;
        const int32_t *gp = g ? g + (size_t)y * cols : NULL;
        const int32_t *bp = b ? b + (size_t)y * cols : NULL;
        uint32_t *dst = fb + (size_t)y * stride_words;
        for (x = 0u; x < cols; ++x) {
            uint32_t rv = rp ? scale_u8(rp[x], lo, hi) : 0u;
            uint32_t gv = gp ? scale_u8(gp[x], lo, hi) : 0u;
            uint32_t bv = bp ? scale_u8(bp[x], lo, hi) : 0u;
            dst[x] = ps_board_fb_pack_xrgb(rv, gv, bv);
        }
    }
    return PS_BOARD_FB_OK;
}
