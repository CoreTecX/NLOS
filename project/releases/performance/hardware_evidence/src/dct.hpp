#pragma once
#include <cmath>
#include <vector>

#if defined(__FAST_MATH__)
#error "M4 requires strict FP64: build with -fno-fast-math -ffp-contract=off"
#endif
#if defined(__aarch64__) && !defined(PS_DCT_PORTABLE)
#include <arm_neon.h>
#define PS_DCT_HAS_NEON 1
#else
#define PS_DCT_HAS_NEON 0
#endif

namespace ps {
// All reductions traverse r in the same order as the frozen scalar DCT.
// SIMD lanes represent independent outputs; there is no horizontal reduction.
template<int H, int W> class SeparableRInverse {
    static constexpr int S = H * W * 3;
    double ch[H][H], cw[W][W], cht[H][H], cwt[W][W];
    std::vector<double> t, b;

    template<int Length, int Lanes>
    static void transform(const double* input, double* output,
                          const double (&coeff)[Length][Length]) {
        for (int k = 0; k < Length; ++k) {
            int q = 0;
            for (; q + 6 <= Lanes; q += 6) {
#if PS_DCT_HAS_NEON
                float64x2_t s0 = vdupq_n_f64(0.0);
                float64x2_t s1 = vdupq_n_f64(0.0);
                float64x2_t s2 = vdupq_n_f64(0.0);
                for (int r = 0; r < Length; ++r) {
                    const double* x = input + r * Lanes + q;
                    const float64x2_t w = vdupq_n_f64(coeff[k][r]);
                    s0 = vaddq_f64(s0, vmulq_f64(w, vld1q_f64(x)));
                    s1 = vaddq_f64(s1, vmulq_f64(w, vld1q_f64(x + 2)));
                    s2 = vaddq_f64(s2, vmulq_f64(w, vld1q_f64(x + 4)));
                }
                vst1q_f64(output + k * Lanes + q, s0);
                vst1q_f64(output + k * Lanes + q + 2, s1);
                vst1q_f64(output + k * Lanes + q + 4, s2);
#else
                double s[6] = {};
                for (int r = 0; r < Length; ++r) {
                    const double* x = input + r * Lanes + q;
                    const double w = coeff[k][r];
                    for (int lane = 0; lane < 6; ++lane) s[lane] += w * x[lane];
                }
                for (int lane = 0; lane < 6; ++lane)
                    output[k * Lanes + q + lane] = s[lane];
#endif
            }
            // H=29 gives 87 lanes in the width pass: handle the final RGB pixel.
            for (; q < Lanes; ++q) {
                double s = 0;
                for (int r = 0; r < Length; ++r) s += coeff[k][r] * input[r * Lanes + q];
                output[k * Lanes + q] = s;
            }
        }
    }

    template<int Rows, int Cols>
    static void transpose(const double* input, double* output) {
        for (int r = 0; r < Rows; ++r)
            for (int c = 0; c < Cols; ++c)
                for (int z = 0; z < 3; ++z)
                    output[(c * Rows + r) * 3 + z] = input[(r * Cols + c) * 3 + z];
    }

public:
    static constexpr bool uses_neon = PS_DCT_HAS_NEON != 0;
    SeparableRInverse() : t(S), b(S) {
        static_assert(H > 0 && W > 0, "DCT dimensions must be positive");
        const double pi = std::acos(-1.);
        for (int k = 0; k < H; ++k) for (int j = 0; j < H; ++j) {
            ch[k][j] = std::sqrt((k ? 2. : 1.) / H) * std::cos(pi * (j + .5) * k / H);
            cht[j][k] = ch[k][j];
        }
        for (int k = 0; k < W; ++k) for (int j = 0; j < W; ++j) {
            cw[k][j] = std::sqrt((k ? 2. : 1.) / W) * std::cos(pi * (j + .5) * k / W);
            cwt[j][k] = cw[k][j];
        }
    }

    // Preconditions are the same as RInverse: a/y have H*W*3 entries, spec has
    // H*W finite positive entries validated by Model. a and y may alias.
    // No allocations occur during run; both scratch buffers are owned per solve.
    // Keep a visible kernel boundary for auditing the shipped ARM instructions.
#if defined(__GNUC__)
    __attribute__((noinline))
#endif
    void run(const std::vector<double>& a, const std::vector<double>& spec,
             std::vector<double>& y) {
        transform<H, W * 3>(a.data(), t.data(), ch);
        transpose<H, W>(t.data(), b.data());
        transform<W, H * 3>(b.data(), t.data(), cw);
        for (int k = 0; k < H; ++k) for (int j = 0; j < W; ++j)
            for (int c = 0; c < 3; ++c)
                b[(k * W + j) * 3 + c] = t[(j * H + k) * 3 + c] / spec[k * W + j];
        transform<H, W * 3>(b.data(), t.data(), cht);
        transpose<H, W>(t.data(), b.data());
        transform<W, H * 3>(b.data(), t.data(), cwt);
        transpose<W, H>(t.data(), y.data());
    }
};
} // namespace ps
