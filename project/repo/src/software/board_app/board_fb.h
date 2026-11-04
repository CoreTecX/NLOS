/* ============================================================================
 * PenumbraSight · 结果 -> PS 显示帧缓冲 的格式转换
 * src/software/board_app/board_fb.h
 *
 * 这一层只做一件事：把计算结果的定点图像（int32，Q6.18 或 Q2.16）映射成 PS 侧
 * DP 扫描输出要的线性帧缓冲行（XRGB8888）。板载 mini-DP 接在 PS 侧、走 linux-xlnx
 * 的 xlnx_dp 驱动（见 docs/board_top_app.md §1），所以帧缓冲是一个普通的线性
 * 像素数组，PS 把它交给 DRM 即可 —— 不涉及任何 PL 视频 IP。
 *
 * 为什么单独成一层、并要求主机可测
 * ---------------------------------------------------------------------------
 * 定标与饱和是最容易「看着都合理但整体偏暗/过曝」的一类错：映射式写错一格不会
 * 崩，只会让画面对比度不对。故把它抽出来，用 lo/hi 两个显式端点定标，主机侧对
 * 「lo->0、hi->255、中点->127、越界饱和、通道序」逐条判。
 *
 * 显示精度声明
 * ---------------------------------------------------------------------------
 * 这里只保证**格式与定标**正确，不保证「画面对不对」：真实 DP 扫描输出、颜色
 * 空间、时序都在板上才成立（见文件尾）。灰度满屏（§6.6 的灰屏）用 fill_gray。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_BOARD_FB_H
#define PENUMBRASIGHT_BOARD_FB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* XRGB8888：bit23..16 = R、bit15..8 = G、bit7..0 = B；X 位恒 0。
 * 取这个格式是因为它是 DRM 线性格式里最省事的一种（不含 alpha 通道）。 */
#define PS_BOARD_FB_XRGB8888 0u

/* §6.6 灰屏：无法出图（几何哈希不符、低可信度等）时整屏铺这个颜色。 */
#define PS_BOARD_FB_GRAY_R 128u
#define PS_BOARD_FB_GRAY_G 128u
#define PS_BOARD_FB_GRAY_B 128u

#define PS_BOARD_FB_OK          0
#define PS_BOARD_FB_ERR_ARGS  (-1)   /* 空指针 / 尺寸 0 / stride 不够 */
#define PS_BOARD_FB_ERR_RANGE (-2)   /* hi <= lo，定标区间无效 */

uint32_t ps_board_fb_pack_xrgb(uint32_t r, uint32_t g, uint32_t b);

/* 把整块帧缓冲铺成一个颜色（words 个像素）。 */
void ps_board_fb_fill(uint32_t *fb, uint32_t words, uint32_t color);

/* §6.6 灰屏。 */
void ps_board_fb_fill_gray(uint32_t *fb, uint32_t words);

/* 单通道 -> 灰度 XRGB。plane 长度须 >= rows*cols；fb 每行 stride_words 个像素。
 * 定标：v <= lo -> 0；v >= hi -> 255；中间线性。要求 hi > lo。 */
int ps_board_fb_pack_mono_xrgb(const int32_t *plane, uint32_t rows, uint32_t cols,
                               uint32_t stride_words, uint32_t *fb,
                               int32_t lo, int32_t hi);

/* 三通道（通道优先，各 rows*cols）-> XRGB。r/g/b 允许为 NULL（该通道填 0）。
 * 定标同上，三个通道共用 lo/hi。 */
int ps_board_fb_pack_rgb_xrgb(const int32_t *r, const int32_t *g, const int32_t *b,
                              uint32_t rows, uint32_t cols,
                              uint32_t stride_words, uint32_t *fb,
                              int32_t lo, int32_t hi);

#ifdef __cplusplus
}
#endif

#endif /* PENUMBRASIGHT_BOARD_FB_H */
