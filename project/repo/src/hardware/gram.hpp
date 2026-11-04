// ============================================================================
// PenumbraSight M3 算子核 · Gram 累加
//
// 设计目标：同一份源码在两个环境下都能工作
//   g++       : 主机侧功能验证（不需要 Vivado）
//   Vitis HLS : 综合（定义 __SYNTHESIS__）
//
// 职责（X1 接口冻结第四节 M3 的第二部分）：
//   一次遍历墙点，同时累加 A_d^T A_d 的**上三角**与 A_d^T y。
//
// 累加结构（决定了 BRAM 占用）
// ---------------------------------------------------------------------------
// A_d 的第 (a, b) 行是一条长度为 N 的向量（a 是 z 索引、b 是 x 索引）：
//     A_d[(a,b), i] = A[(a+1, b), i] - A[(a, b), i]
// 于是累加是「逐行的外积」：
//     G[i, i'] += A_d[(a,b), i] * A_d[(a,b), i']      （i <= i'，上三角）
//     aty[i]   += A_d[(a,b), i] * yd[a*n + b]
// 每个墙点行都要遍历全部 (i, i') 对，故总 MAC = (n-1)*n * N*(N+1)/2 = 8.59e9。
//
// 为什么分块、分几块（X9 核算结论，见 sim/results/x9_gram_hls.md）
// ---------------------------------------------------------------------------
// 上三角 545,490 个元素。单片全存的峰值：
//   17 bit -> 29.09% BRAM，20 bit -> 34.19%，24 bit -> 40.99%
// 判据是峰值 ≤ 30%，故 20 bit 起必须分块。**分块几乎不要钱**：
// 累加 MAC 不随块数增长（每行该累加的对数总量不变），只有「A_d 行生成」
// 要多跑几遍，而它只占 Gram 的 0.38%。
//   2 块 / 20 bit：峰值 17.20%，Gram 相对误差 1.18e-05，代价 +0.38%   <- 默认
//   2 块 / 24 bit：峰值 20.60%，Gram 相对误差 7.35e-07，代价 +0.38%
//
// 关键实现约束
// ---------------------------------------------------------------------------
// G1 分块边界按**隐藏点编号 i** 切，不按墙点切。第 b 块的条目数
//    = Σ_{i=lo}^{hi-1}(N-i)；均分累计量即 2 块均衡（N=1044 时切点 306）。
// G2 每块一块独立累加器，块间不复用。峰值 = 最大块 + 状态。
// G3 A_d 行由两行 A 现场差分，不落盘、不物化；每次 (a,b) 都要重算两个权重行，
//    这是分块与流式的共同代价，已计入 0.38%。
// G4 权重不含任何二维表：d2 = dx^2(x,i) + dz^2(z,i) + y_w^2 现场算。
//    X9 核算：实时算 ≤ 0.39% of Gram，而 R1 的两张表要 263,088 项。
// G5 1/d2^2 用 LUT + 线性插值（512 项，相对误差 2.33e-06）。
// G6 **可见性矩形是逐隐藏点的**（rect[i]），不是全局一个。写错这里等于用真值
//    位置以外的几何，且不会报错，只会让 Gram 静默偏掉。
// G7 不用 new/delete（不可综合）。所有暂存由调用方传入。
// ============================================================================

#ifndef PENUMBRASIGHT_GRAM_HPP
#define PENUMBRASIGHT_GRAM_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "geometry.hpp"
#include "transport.hpp"

namespace ps {

#ifndef PS_MAX_GRAM_BLOCKS
#define PS_MAX_GRAM_BLOCKS 4
#endif

// 1/d2^2 的 LUT 项数（线性插值）
#ifndef PS_INV4_LUT
#define PS_INV4_LUT 512
#endif

// ---------------------------------------------------------------- 分块计划

struct GramPlan {
    int n_blocks;
    int n_scene;
    int cuts[PS_MAX_GRAM_BLOCKS + 1];   // 段边界，cuts[0]=0、cuts[n_blocks]=N
    int max_entries;                    // 最大块条目数（决定峰值 BRAM）
};

// 累计条目数：前 m 行（i=0..m-1）贡献 Σ(N-i) = m*N - m*(m-1)/2
inline long long gram_cum_entries(int m, int n_scene) {
    return (long long)m * n_scene - (long long)m * (m - 1) / 2;
}

// 按累计条目数均分，给出 K 块的切点。
inline void gram_make_plan(int n_scene, int n_blocks, GramPlan& p) {
    if (n_blocks < 1) n_blocks = 1;
    if (n_blocks > PS_MAX_GRAM_BLOCKS) n_blocks = PS_MAX_GRAM_BLOCKS;
    p.n_blocks = n_blocks;
    p.n_scene = n_scene;
    p.cuts[0] = 0;
    p.cuts[n_blocks] = n_scene;
    const long long total = gram_cum_entries(n_scene, n_scene);
    int prev = 0;
    for (int b = 1; b < n_blocks; ++b) {
        const long long target = total * b / n_blocks;
        // 找最小的 m，使前 m 行的累计条目数 >= 目标；并强制严格递增，避免空块
        int m = prev;
        while (m < n_scene && gram_cum_entries(m, n_scene) < target) ++m;
        if (m <= prev) m = prev + 1;
        if (m > n_scene) m = n_scene;
        p.cuts[b] = m;
        prev = m;
    }
    p.max_entries = 0;
    for (int b = 0; b < n_blocks; ++b) {
        const long long e = gram_cum_entries(p.cuts[b + 1], n_scene)
                            - gram_cum_entries(p.cuts[b], n_scene);
        if ((long long)p.max_entries < e) p.max_entries = (int)e;
    }
}

// 块内偏移基址：base[i] = Σ_{j=lo}^{i-1}(N-j)，base[hi] 即该块总条目数。
// 调用方按 acc[base[i] + (i' - i)] 索引。
inline void gram_block_base(const GramPlan& p, int blk, int* base) {
    const int lo = p.cuts[blk], hi = p.cuts[blk + 1];
    int acc = 0;
    for (int i = lo; i < hi; ++i) {
        base[i] = acc;
        acc += (p.n_scene - i);
    }
    base[hi] = acc;
}

// ---------------------------------------------------------------- inv4 LUT

// 1/d2^2 的 LUT + 线性插值。d2 动态范围只有约 1.9 倍，这是插值精度能到
// 1e-6 的前提（X9 实测 512 项 -> 2.33e-06，占 BRAM 0.038%）。
struct Inv4Lut {
    double lo, hi, inv_step;
    double v[PS_INV4_LUT];

    void init(double d2_lo, double d2_hi) {
        lo = d2_lo;
        hi = d2_hi;
        const double span = (hi > lo) ? (hi - lo) : 1.0;
        inv_step = (PS_INV4_LUT - 1) / span;
        for (int i = 0; i < PS_INV4_LUT; ++i) {
            const double d2 = lo + span * i / (double)(PS_INV4_LUT - 1);
            v[i] = 1.0 / (d2 * d2);
        }
    }

    inline double operator()(double d2) const {
#pragma HLS INLINE
        const double pos = (d2 - lo) * inv_step;
        if (pos <= 0.0) return v[0];
        if (pos >= (double)(PS_INV4_LUT - 1)) return v[PS_INV4_LUT - 1];
        const int i0 = (int)pos;
        const double fr = pos - (double)i0;
        return v[i0] + fr * (v[i0 + 1] - v[i0]);
    }
};

// ---------------------------------------------------------------- 权重行

// 生成一行 A 的权重（长度 N，已含可见性）。z 为行索引、x 为列索引，
// 传入的墙坐标是**一维**的（wall_x 只随 x 变、wall_z 只随 z 变，X1 约定 2）。
// 可见性逐隐藏点取自 rect[i]（约束 G6）。
inline void gram_weight_row(int z, int x, const OccRect* rect,
                            const double* wall_x, const double* wall_z,
                            const double* mon_x, const double* mon_z,
                            const TransportParams& tp, const Inv4Lut& lut,
                            double c_lut, double* row, int n_scene) {
#pragma HLS INLINE
    const double wz = wall_z[z];
    const double dx_wall = wall_x[x];
    const double yw2 = tp.y_w * tp.y_w;
    for (int i = 0; i < n_scene; ++i) {
#pragma HLS PIPELINE II = 1
        double w = 0.0;
        if (visible_at(z, x, rect[i])) {
            const double dx = dx_wall - mon_x[i];
            const double dz = wz - mon_z[i];
            const double d2 = dx * dx + dz * dz + yw2;
            w = c_lut * lut(d2);
        }
        row[i] = w;
    }
}

// ---------------------------------------------------------------- 累加

// 一个块的一趟遍历：累加该块的上三角条目与 A_d^T y。
// 暂存全部由调用方提供（G7）：
//   acc    : 长度 >= 该块条目数（gram_block_base 的 base[hi]）
//   base   : 长度 >= N+1
//   w_lo / w_hi / ad_row : 各长度 >= N
// 返回本块累加的 MAC 数。
inline long long m3_accum_gram_block(const GramPlan& plan, int blk,
                                     const OccRect* rect,
                                     const double* wall_x, const double* wall_z,
                                     const double* mon_x, const double* mon_z,
                                     const TransportParams& tp,
                                     const Inv4Lut& lut, double c_lut,
                                     const double* yd, double* acc, double* aty,
                                     int* base, double* w_lo, double* w_hi,
                                     double* ad_row) {
    const int n = tp.wall_n;
    const int N = plan.n_scene;
    const int lo = plan.cuts[blk], hi = plan.cuts[blk + 1];
    gram_block_base(plan, blk, base);
    const int n_ent = base[hi];

    for (int k = 0; k < n_ent; ++k) acc[k] = 0.0;
    for (int i = 0; i < N; ++i) aty[i] = 0.0;

    long long mac = 0;
    for (int a = 0; a + 1 < n; ++a) {
        for (int b = 0; b < n; ++b) {
            gram_weight_row(a, b, rect, wall_x, wall_z, mon_x, mon_z,
                            tp, lut, c_lut, w_lo, N);
            gram_weight_row(a + 1, b, rect, wall_x, wall_z, mon_x, mon_z,
                            tp, lut, c_lut, w_hi, N);
            for (int i = 0; i < N; ++i) ad_row[i] = w_hi[i] - w_lo[i];

            const double yv = yd[(std::size_t)a * n + b];
            for (int i = 0; i < lo; ++i) aty[i] += ad_row[i] * yv;
            for (int i = lo; i < hi; ++i) {
                const double ai = ad_row[i];
                aty[i] += ai * yv;
                double* dst = acc + base[i];
                for (int j = i; j < N; ++j) {
#pragma HLS PIPELINE II = 1
                    dst[j - i] += ai * ad_row[j];
                }
                mac += (long long)(N - i);
            }
            for (int i = hi; i < N; ++i) aty[i] += ad_row[i] * yv;
        }
    }
    return mac;
}

// 完整 Gram：逐块累加，按块顺序写回连续的 gram_triu（行优先上三角）。
inline long long m3_accum_gram(const GramPlan& plan, const OccRect* rect,
                               const double* wall_x, const double* wall_z,
                               const double* mon_x, const double* mon_z,
                               const TransportParams& tp, const Inv4Lut& lut,
                               double c_lut, const double* yd,
                               double* gram_triu, double* aty, double* acc,
                               int* base, double* w_lo, double* w_hi,
                               double* ad_row) {
    long long mac = 0;
    int write_off = 0;
    for (int b = 0; b < plan.n_blocks; ++b) {
        mac += m3_accum_gram_block(plan, b, rect, wall_x, wall_z, mon_x, mon_z,
                                   tp, lut, c_lut, yd, acc, aty,
                                   base, w_lo, w_hi, ad_row);
        gram_block_base(plan, b, base);
        const int n_ent = base[plan.cuts[b + 1]];
        for (int k = 0; k < n_ent; ++k) gram_triu[write_off + k] = acc[k];
        write_off += n_ent;
    }
    return mac;
}

}  // namespace ps

#endif  // PENUMBRASIGHT_GRAM_HPP
