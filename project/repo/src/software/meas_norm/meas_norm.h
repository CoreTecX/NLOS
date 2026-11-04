/* ============================================================================
 * PenumbraSight · PS 侧「预处理链输出 -> 定点核输入」那一级转换
 * src/software/meas_norm/meas_norm.h
 *
 * 这一层补的是一个**实现缺口**：RTL 预处理链（raw_preproc -> vertical_diff）
 * 只产出 signed 24 bit 的原始整数差分；定点定位核的 meas 要的是通道优先块
 * meas[ch*M + m] 里的 Q2.16 signed 18 bit 码字。两者之间缺一级转换，仓库里
 * 此前只有 Python 版（build 区的 seam_prepare.py）。本层把这一级用 C99 落到
 * PS 侧，接口按板端形态设计（输入指向 RTL 输出缓冲、输出指向核要的缓冲）。
 *
 * 三件事（与 T2 接缝实测的不一致项一一对应，见
 * sim/results/pl_t2_rebuild_and_seam_20260929.md §6.3）
 * ---------------------------------------------------------------------------
 *   1. 尺度：RTL 只出原始整数差分（T2 实测 absmax 6.21e6），核的 meas 是
 *      Q2.16 码字（|code| <= 131071），直连超出包络 47.4 倍。这里按**每帧共享
 *      指数**归一化，口径逐值复刻 src/model/penumbrasight_model/quantize.py 的
 *      shared_exponent（不另立一套）。
 *   2. 布局：RTL 流序 [row][col][ch]（ch 最内，逐拍），核要通道优先块
 *      meas[ch*M + m]，m = r*cols + c。这里做一次重排。
 *   3. 位宽：RTL 出 signed 24 bit，核要 int32 字。这里做符号扩展 + 32 位打包。
 *
 * 共享指数的口径（权威实现 = quantize.shared_exponent）
 * ---------------------------------------------------------------------------
 *   每帧（三通道一起）算一个整数指数：
 *       exponent = ceil(log2(max|x|)) + 1        （max|x| == 0 时取 0）
 *   编码：scale = 2^(exponent - frac_bits)（默认 frac_bits = 16）
 *         code  = round_half_even(x / scale)，饱和到 signed mantissa（默认 18 bit）
 *   解码：x = code / 2^(frac_bits - exponent)  （除数 divisor 见结果结构）
 *   取整一律「向最近值、半值取偶」；饱和**不回绕**。
 *
 * 与 quantize.py 的两处边界（本层照 quantize.py 走，详见 .c 的说明）
 * ---------------------------------------------------------------------------
 *   · 饱和上下界**对称**：hi = 2^(mantissa_bits-1) - 1，取 [-hi, +hi]。
 *     quantize.py 的 shared_exponent 用 `saturate(..., -hi, hi)`，即负侧也是
 *     -131071（不是 -131072）。本层同此。
 *   · absmax == 0（全零帧）时 exponent = 0，码字全 0。
 *
 * 依赖纪律（与 board/fix_core.{h,c} 同风格）
 * ---------------------------------------------------------------------------
 *   · 只依赖 <stdint.h>/<stddef.h>；不做动态分配、不用 libm、不打印、不碰寄存器；
 *   · 输入/输出都是调用方给的缓冲区指针，本层不拥有内存；
 *   · 因此裸机与 PetaLinux 用户态都能直接编译进去。
 *
 * 未验证声明
 * ---------------------------------------------------------------------------
 *   · 主机侧已与 quantize.py 逐位对拍（见 test_meas_norm.c / make test-meas-norm）；
 *   · **未在板上跑过**，也**未**做 RTL 全尺寸 xsim 端到端；本层只是把 T2 接缝
 *     那级转换从 Python 搬到 C，口径结论沿用 T2 报告。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_MEAS_NORM_H
#define PENUMBRASIGHT_MEAS_NORM_H

#include <stdint.h>

/* ------------------------------------------------------------ 静态格式 */

/* 方案 §6.3 的 Q2.16：mantissa 18 bit（含符号）、小数 16 bit，整数位 = 2。 */
#define PS_MEAS_NORM_MANTISSA_BITS 18
#define PS_MEAS_NORM_FRAC_BITS     16

/* 码字包络：hi = 2^(mantissa_bits-1) - 1；上下界对称（同 quantize.py）。 */
#define PS_MEAS_NORM_CODE_MAX  ((int32_t)((1 << (PS_MEAS_NORM_MANTISSA_BITS - 1)) - 1))
#define PS_MEAS_NORM_CODE_MIN  (-PS_MEAS_NORM_CODE_MAX)

/* RTL 差分的位宽与取值容器（vertical_diff 的 DIFF_BITS = 24，signed）。 */
#define PS_MEAS_RTL_DIFF_BITS  24
#define PS_MEAS_RTL_TAG_MASK   0x00FFFFFFu   /* 低 24 bit 有效 */
#define PS_MEAS_RTL_SIGN_BIT   0x00800000u   /* bit23 为符号位 */

/* ------------------------------------------------------------ 返回码 */

#define PS_MEAS_OK         0
#define PS_MEAS_ERR_ARGS  (-1)

/* ------------------------------------------------------------ 一帧结果 */

typedef struct {
    uint32_t n_samples;      /* = rows * cols * channels */
    uint32_t channels;       /* 通道数（RTL 侧恒为 3） */
    int32_t  raw_absmax;     /* 本帧 max|x|（原始整数差分） */
    int      exponent;       /* 共享指数 e */
    int      frac_bits;      /* 恒为 PS_MEAS_NORM_FRAC_BITS */
    int      mantissa_bits;  /* 恒为 PS_MEAS_NORM_MANTISSA_BITS */
    int32_t  code_absmax;    /* 本帧码字 max|code| */
    uint32_t sat_events;     /* 被饱和夹取的样本数（帧级恒为 0，见 .c 说明） */
    int      divisor_exp;    /* 解码：x = code / 2^divisor_exp，divisor_exp = frac_bits - e */
} ps_meas_norm_result_t;

/* ------------------------------------------------------------ 元件（可单独用） */

/* 符号扩展：把 RTL 拍的低 24 bit 二补码整数扩成 int32。
 * 输入只取低 24 bit；越界的位被忽略（不产生 UB，见 .c）。 */
int32_t ps_meas_sext24(uint32_t beat);

/* 共享指数：ceil(log2(absmax)) + 1；absmax <= 0 时返回 0。
 * 与 quantize.shared_exponent 的 exp 逐值一致（证明见 .c 注释）。 */
int ps_meas_shared_exponent(int32_t absmax);

/* 单样本量化：code = round_half_even(v / 2^(exponent - frac_bits))，饱和到
 * [CODE_MIN, CODE_MAX]。与 quantize.shared_exponent 的编码行同口径。
 * sat 非空时写出 1（该样本被夹取）/ 0。 */
int32_t ps_meas_quantize_code(int32_t v, int exponent, int *sat);

/* ------------------------------------------------------------ 一帧转换 */

/* 核心入口：samples 是**已符号扩展**的 int32，流序 [row][col][ch]（ch 最内），
 * 长度 = rows*cols*channels。输出通道优先块 meas_cf[ch*M + (r*cols + c)]，
 * M = rows*cols（与核的 meas[ch*M + m]、m = r*cols + c 一致）。
 *
 * rows/cols 是**差分帧**的尺寸：vertical_diff 对 WALL_N x WALL_N 出
 * (WALL_N-1) x WALL_N，故 T2 接缝为 rows=95、cols=96、channels=3。
 *
 * result 可为 NULL（只出码字）；非空时写回本帧的指数、包络与饱和统计。
 * 返回 PS_MEAS_OK / PS_MEAS_ERR_ARGS（空指针或尺寸为 0）。 */
int ps_meas_norm_q216(const int32_t *samples, uint32_t rows, uint32_t cols,
                      uint32_t channels, int32_t *meas_cf,
                      ps_meas_norm_result_t *result);

/* 板端入口：直接吃 DMA 进来的 RTL 输出拍（每个 uint32 的低 24 bit 是 signed
 * 差分，其余位忽略），内部做符号扩展后与 ps_meas_norm_q216 同路径。
 * n_samples 由 rows*cols*channels 决定；beats 长度须不少于它。
 * 本函数**不**需要中间缓冲（两次扫描，逐拍符号扩展）。 */
int ps_meas_norm_from_rtl24(const uint32_t *beats, uint32_t rows, uint32_t cols,
                            uint32_t channels, int32_t *meas_cf,
                            ps_meas_norm_result_t *result);

#endif /* PENUMBRASIGHT_MEAS_NORM_H */
