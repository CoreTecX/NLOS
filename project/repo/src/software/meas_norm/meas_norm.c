/* ============================================================================
 * PenumbraSight · 「预处理链输出 -> 定点核输入」转换层实现
 * src/software/meas_norm/meas_norm.c
 *
 * 口径权威是 src/model/penumbrasight_model/quantize.py 的 shared_exponent()。
 * 本文件**逐值复刻**它的三个可复算步骤（定标 / 取整 / 饱和），不引入第二套语义。
 * 之所以用整数实现而不是浮点：
 *   · 板端不依赖 libm / fenv；
 *   · 指数是整数、scale 是 2 的幂，除法在 double 里精确，故整数路径与
 *     quantize.py 的 float64 路径**逐位相同**（取值域见下面的证明）。
 *
 * 指数口径的逐值一致性证明
 * ---------------------------------------------------------------------------
 * quantize.shared_exponent 取 exp = ceil(log2(max|x|)) + 1（max|x|==0 时为 0）。
 * 本层输入是 signed 24 bit 整数差分，故 max|x| = A 是 [0, 2^23] 内的整数。
 * 对整数 A >= 1，ceil(log2 A) 等于「A 的最高置位下标」，A 是 2 的幂时取该下标、
 * 否则再 +1；这个量正好等于 (A-1) 的位长 bitlen(A-1)。证明：
 *   设 h = floor(log2 A)。
 *   · A == 2^h（幂）：ceil(log2 A) = h；(A-1) 的位长 = h（例 A=8 -> 7 -> 3）。
 *   · 2^h < A < 2^(h+1)：ceil = h+1；(A-1) 的位长 = h+1（例 A=9 -> 8 -> 4）。
 *   A=1 时 ceil=0，位长(0)=0，一致。
 * 故 exp = bitlen(A-1) + 1 与 math.ceil(math.log2(A)) + 1 在 A <= 2^23 内逐值相等
 * （该范围内 math.log2 对 2 的幂返回精确整数、对非幂与最近整数的距离远大于 ulp，
 *  不存在浮点进位歧义；A 更大时 log2 的舍入可能改变 ceil，超出本层取值域）。
 *
 * 取整口径的逐值一致性
 * ---------------------------------------------------------------------------
 * scale = 2^(exp - frac_bits) 是 2 的幂，x/scale 在 double 里精确（|x| <= 2^23、
 * 结果量级 <= 2^16，远在 double 精确整数+2^-8 分辨率之内）。round_half_even
 * 用整数实现：
 *   exp <= frac_bits：x * 2^(frac_bits-exp) 是整数，无舍入；
 *   exp >  frac_bits：n = x、d = 2^(exp-frac_bits)、half = d/2，
 *                     用 floor 商 q 与 [0,d) 余数 r 判 r 与 half 的关系，
 *                     r == half 时向偶数 q 靠。
 * 该结果与 np.rint(x/scale) 逐位相同。
 *
 * 饱和口径的逐值一致性
 * ---------------------------------------------------------------------------
 * quantize.shared_exponent 用 saturate(..., -hi, hi)，hi = 2^(mantissa_bits-1)-1
 * = 131071，**负侧也是 -131071**（不是 -131072）。本层同此。饱和不回绕。
 *
 * 一条边界事实（本层如实报告，不改口径）
 * ---------------------------------------------------------------------------
 * 在帧级路径上饱和**不可达**：exp 由本帧 absmax 推出，则本帧最大码字模长 =
 * A / 2^(exp-frac_bits) = (A / 2^(ceil(log2 A))) * 2^frac_bits 落在
 * (2^14, 2^15]，恒小于 131071。故 sat_events 在帧级恒为 0；饱和分支只在
 * ps_meas_quantize_code 被以**外部给定的小指数**直接调用时才可能触发
 * （测试里的 clamp 探针即走这条路）。这不影响口径一致性：C 与 quantize.py
 * 的夹取行为仍在同一处、同一组界上。
 * ==========================================================================*/

#include "meas_norm.h"

#include <stddef.h>

/* ------------------------------------------------------------ 元件 */

int32_t ps_meas_sext24(uint32_t beat)
{
    /* 取低 23 位（无符号、必落在 int32 正区），符号位单独减。
     * 这样避免「把超范围 unsigned 转 signed」那种实现定义行为，也不产生告警。 */
    int32_t v = (int32_t)(beat & 0x007FFFFFu);
    if (beat & PS_MEAS_RTL_SIGN_BIT) {
        v -= (int32_t)0x00800000u;    /* 8388608 */
    }
    return v;
}

int ps_meas_shared_exponent(int32_t absmax)
{
    int h = 0;
    uint32_t m;
    if (absmax <= 0) {
        return 0;
    }
    m = (uint32_t)absmax - 1u;        /* bitlen(A-1)，A >= 1 */
    while (m != 0u) {
        h++;
        m >>= 1;
    }
    return h + 1;
}

static int32_t quantize_code_ex(int32_t v, int exponent, int *sat)
{
    int shift = exponent - PS_MEAS_NORM_FRAC_BITS;
    int64_t q;

    if (sat != NULL) {
        *sat = 0;
    }

    if (v == 0) {
        return 0;
    }

    if (shift <= 0) {
        int lsh = -shift;
        if (lsh >= 62) {
            /* 防御：把码字直接顶到对应的边界，避免超宽移位（C 里是 UB）。 */
            if (sat != NULL) { *sat = 1; }
            return (v > 0) ? PS_MEAS_NORM_CODE_MAX : PS_MEAS_NORM_CODE_MIN;
        }
        q = (int64_t)v << lsh;        /* 整数，无舍入 */
    } else {
        int64_t n = (int64_t)v;
        int64_t d = (int64_t)1 << shift;
        int64_t half = d >> 1;
        int64_t r;

        q = n / d;                    /* C99 向零截断 */
        r = n % d;                    /* 余数与 n 同号 */
        if (r < 0) {                  /* 转成 floor 商 + [0,d) 余数 */
            r += d;
            q -= 1;
        }
        if (r > half) {
            q += 1;                   /* 余数超过半格：进位 */
        } else if (r == half) {
            if ((q & 1) != 0) {       /* 正好半格：向偶数靠 */
                q += 1;
            }
        }
    }

    if (q > (int64_t)PS_MEAS_NORM_CODE_MAX) {
        if (sat != NULL) { *sat = 1; }
        return PS_MEAS_NORM_CODE_MAX;
    }
    if (q < (int64_t)PS_MEAS_NORM_CODE_MIN) {
        if (sat != NULL) { *sat = 1; }
        return PS_MEAS_NORM_CODE_MIN;
    }
    return (int32_t)q;
}

int32_t ps_meas_quantize_code(int32_t v, int exponent, int *sat)
{
    return quantize_code_ex(v, exponent, sat);
}

/* ------------------------------------------------------------ 一帧转换 */

/*
 * 两条入口共用同一套「扫描 -> 定标 -> 重排」逻辑，只有「怎么把第 i 个样本取成
 * int32」不同。用一个极小枚举区分，避免把重排循环抄两遍（抄两遍就会有第二套口径）。
 */
#define PS_MEAS_SRC_S32 0
#define PS_MEAS_SRC_W24 1

static int norm_impl(const void *src, int kind, uint32_t rows, uint32_t cols,
                     uint32_t channels, int32_t *meas_cf,
                     ps_meas_norm_result_t *result)
{
    size_t m_blk;
    size_t n;
    size_t i;
    int32_t absmax = 0;
    int exponent;
    int32_t code_absmax = 0;
    uint32_t sat_events = 0;

    if (src == NULL || meas_cf == NULL ||
        rows == 0u || cols == 0u || channels == 0u) {
        return PS_MEAS_ERR_ARGS;
    }

    m_blk = (size_t)rows * (size_t)cols;      /* 核侧 M；通道优先块的步长 */
    n = m_blk * (size_t)channels;
    if (m_blk == 0u || n == 0u) {
        return PS_MEAS_ERR_ARGS;
    }

    /* 第一遍：帧内 max|x|（三通道一起，共享指数）。 */
    for (i = 0u; i < n; i++) {
        int32_t v = (kind == PS_MEAS_SRC_W24)
                        ? ps_meas_sext24(((const uint32_t *)src)[i])
                        : ((const int32_t *)src)[i];
        int32_t a = (v < 0) ? (int32_t)(-(int64_t)v) : v;
        if (a > absmax) {
            absmax = a;
        }
    }
    exponent = ps_meas_shared_exponent(absmax);

    /* 第二遍：逐样本量化 + 通道优先重排。i = (r*cols + c)*channels + ch，
     * 故 i/channels = r*cols + c = m，i%channels = ch。 */
    for (i = 0u; i < n; i++) {
        int32_t v = (kind == PS_MEAS_SRC_W24)
                        ? ps_meas_sext24(((const uint32_t *)src)[i])
                        : ((const int32_t *)src)[i];
        int sat = 0;
        int32_t code = quantize_code_ex(v, exponent, &sat);
        size_t ch = i % (size_t)channels;
        size_t m = i / (size_t)channels;
        int32_t a = (code < 0) ? (int32_t)(-(int64_t)code) : code;

        meas_cf[ch * m_blk + m] = code;
        if (a > code_absmax) {
            code_absmax = a;
        }
        if (sat) {
            sat_events++;
        }
    }

    if (result != NULL) {
        result->n_samples     = (uint32_t)n;
        result->channels      = channels;
        result->raw_absmax    = absmax;
        result->exponent      = exponent;
        result->frac_bits     = PS_MEAS_NORM_FRAC_BITS;
        result->mantissa_bits = PS_MEAS_NORM_MANTISSA_BITS;
        result->code_absmax   = code_absmax;
        result->sat_events    = sat_events;
        result->divisor_exp   = PS_MEAS_NORM_FRAC_BITS - exponent;
    }
    return PS_MEAS_OK;
}

int ps_meas_norm_q216(const int32_t *samples, uint32_t rows, uint32_t cols,
                      uint32_t channels, int32_t *meas_cf,
                      ps_meas_norm_result_t *result)
{
    return norm_impl((const void *)samples, PS_MEAS_SRC_S32,
                     rows, cols, channels, meas_cf, result);
}

int ps_meas_norm_from_rtl24(const uint32_t *beats, uint32_t rows, uint32_t cols,
                            uint32_t channels, int32_t *meas_cf,
                            ps_meas_norm_result_t *result)
{
    return norm_impl((const void *)beats, PS_MEAS_SRC_W24,
                     rows, cols, channels, meas_cf, result);
}
