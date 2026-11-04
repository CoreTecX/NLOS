// ============================================================================
// PenumbraSight M3 算子核 · HLS 风格 C++
//
// 设计目标：同一份源码在两个环境下都能工作
//   g++       : 主机侧功能验证（不需要 Vivado）
//   Vitis HLS : 综合（定义 __SYNTHESIS__）
//
// 职责（X1 接口冻结第四节 M3）：由 M2 的矩形索引 + 墙/隐藏点常数表，
// **流式**产出 A 与 A_d 的列，供定位核（A_dᵀA_d、A_dᵀy）与重建核（A_dx、A_dᵀr）使用。
//
// 为什么是「流式」而不是「物化矩阵」
// ---------------------------------------------------------------------------
// A 是 15876 x 1044，float32 就是 66.3 MB；A_d 同量级。板上放不下，也不能每候选
// 过一次 DDR。本文件的做法是：只保留**两行墙点**的滚动缓冲，边算边差分边吐出，
// 状态量 O(n) 而不是 O(n^2)。这也是 A6 把 Gram 定为 MAC 口径量级大头的原因。
//
// 权重的结构（X1 接口冻结第三节，决定了这条路径可行）
// ---------------------------------------------------------------------------
//   y_w  = wall_y - monitor_y            <- 与遮挡物无关，故与候选无关
//   d2   = (wx-mx)^2 + (wz-mz)^2 + y_w^2
//   A[j,i] = y_w^2 * dA / d2^2 * V[j,i]  <- 只有 V 随候选变
// 权重部分候选无关，可见性 V 是矩形补集。本文件把这两者分开算：
// 权重用滚动方式实时算（含 1/||v||^4，即 X8 任务名里的 inv4 实时计算），
// 可见性用 M2 的 4 个索引直接判。
//
// 尚未决定、交给 X9 的一项
// ---------------------------------------------------------------------------
// 实时算 1/d2^2 需要除法。硬件上除法要几个周期，且每候选要做 n^2 x N 次。
// 两条候选路线（X9 定）：
//   R1 预存分开的 dx2[x,i] 与 dz2[z,i] 两张表（各 126 x 1044），
//      d2 = dx2 + dz2 + y_w^2 只需一次加法，再由倒数表/牛顿迭代取代除法；
//   R2 按 S0-2-b 的低秩可分形式预存 p_k(j) 与 q_k(i)，把 1/||v||^4 写成 R 项之和。
//      注意 A6 已否证「用低秩近似构造差分 Gram」——R2 只可用于生成 A_d 本身，
//      不可用于 A_dᵀA_d。这条约束写在 X1 接口冻结里，不要违反。
// 本文件按 R1 的分解方式组织代码（dx2/dz2 可分离），但不预存表，先保证数值正确。
// ============================================================================

#ifndef PENUMBRASIGHT_TRANSPORT_HPP
#define PENUMBRASIGHT_TRANSPORT_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "geometry.hpp"

namespace ps {

// ---------------------------------------------------------------- 参数

struct TransportParams {
    double y_w;             // 墙到显示器的距离，与候选无关
    double dA;              // 显示器单个分块面积 monitor.block_area
    int    wall_n;
};

inline void transport_configure(double wall_y, double monitor_y, double block_area,
                                int wall_n, TransportParams& p) {
    p.y_w = wall_y - monitor_y;
    p.dA = block_area;
    p.wall_n = wall_n;
}

// ---------------------------------------------------------------- 可见性

// 墙点 (z, x) 对隐藏点 i 是否可见。
// M2 给的矩形是**被遮挡**的索引区间，故矩形内 = 不可见。
// 注意 rect_empty 时恒可见——这一条要与 M2 的 C1 一起理解：
// 区间完全落在网格外时矩形为空，此时该隐藏点没有任何墙点被它遮挡。
inline bool visible_at(int z, int x, const OccRect& r) {
#pragma HLS INLINE
    if (rect_empty(r)) return true;
    const bool in_x = (x >= (int)r.x_lo) && (x <= (int)r.x_hi);
    const bool in_z = (z >= (int)r.z_lo) && (z <= (int)r.z_hi);
    return !(in_x && in_z);
}

// ---------------------------------------------------------------- 权重

// A[j, i] 的权重部分（不含可见性）：w = y_w^2 * dA / ||v||^4
//   ||v||^2 = dx^2 + dz^2 + y_w^2
// 传入的 dz2_yw2 已含 dz^2 + y_w^2，便于内层循环外提。
inline double weight_from(double dx2_plus, double dz2_yw2,
                          const TransportParams& p) {
#pragma HLS INLINE
    double d2 = dx2_plus + dz2_yw2;
    d2 = d2 * d2;                       // ||v||^4
    return (p.y_w * p.y_w * p.dA) / d2;
}

// ---------------------------------------------------------------- 单行生成

// 生成 A 的第 z 行（长度 n，列索引 = x），写入 row。
// 返回该行是否被矩形切开（便于统计，不影响结果）。
inline void emit_A_row(int z, const OccRect& r,
                       const double* wall_x, double wall_z_minus_mz_sq_yw2,
                       double mx, const TransportParams& p, double* row) {
#pragma HLS INLINE
    const int n = p.wall_n;
    for (int x = 0; x < n; ++x) {
#pragma HLS PIPELINE II = 1
        double v = 0.0;
        if (visible_at(z, x, r)) {
            const double dx = wall_x[x] - mx;
            v = weight_from(dx * dx, wall_z_minus_mz_sq_yw2, p);
        }
        row[x] = v;
    }
}

// ---------------------------------------------------------------- 单列生成

// 生成 A 的第 i 列（长度 n*n，索引 j = z*n + x）。
inline void emit_A_column(int i, const OccRect& r,
                          const double* wall_x, const double* wall_z,
                          const double* mon_x, const double* mon_z,
                          const TransportParams& p, double* col) {
    const int n = p.wall_n;
    const double mx = mon_x[i];
    const double mz = mon_z[i];
    for (int z = 0; z < n; ++z) {
#pragma HLS PIPELINE II = 1
        const double dz = wall_z[z] - mz;
        const double dz2_yw2 = dz * dz + p.y_w * p.y_w;
        emit_A_row(z, r, wall_x, dz2_yw2, mx, p, col + (std::size_t)z * n);
    }
}

// 生成 A_d 的第 i 列（长度 (n-1)*n，索引 (a*n + b)，a 为 z、b 为 x）。
//
//   A_d[(a,b), i] = A[z=a+1, x=b, i] - A[z=a, x=b, i]
//
// 流式实现：只保留两行墙点（row_prev / row_cur）滚动，状态 O(n)。
// 调用方需提供两条长度为 n 的暂存行。
inline void emit_Ad_column(int i, const OccRect& r,
                           const double* wall_x, const double* wall_z,
                           const double* mon_x, const double* mon_z,
                           const TransportParams& p, double* col,
                           double* row_prev, double* row_cur) {
    const int n = p.wall_n;
    const double mx = mon_x[i];
    const double mz = mon_z[i];

    // 第 0 行
    {
        const double dz = wall_z[0] - mz;
        emit_A_row(0, r, wall_x, dz * dz + p.y_w * p.y_w, mx, p, row_prev);
    }
    for (int a = 0; a + 1 < n; ++a) {
#pragma HLS PIPELINE II = 1
        const double dz = wall_z[a + 1] - mz;
        emit_A_row(a + 1, r, wall_x, dz * dz + p.y_w * p.y_w, mx, p, row_cur);
        double* dst = col + (std::size_t)a * n;
        for (int b = 0; b < n; ++b) {
#pragma HLS UNROLL
            dst[b] = row_cur[b] - row_prev[b];
        }
        // 滚动：把 cur 换到 prev（硬件上用寄存器组双缓冲，不做逐元素拷贝）
        double* tmp = row_prev;
        row_prev = row_cur;
        row_cur = tmp;
    }
}

// ---------------------------------------------------------------- 算子应用
//
// 供重建核（X17）使用：A_dx 与 A_dᵀr，同样按两行滚动实现，不物化 A_d。

// y = A_d @ x（x 长度 N，输出 (n-1)*n）。A_d 按列流式生成后与 x[i] 加权累加。
inline void apply_Ad(int n_scene, const OccRect* rect,
                     const double* wall_x, const double* wall_z,
                     const double* mon_x, const double* mon_z,
                     const TransportParams& p, const double* x, double* y,
                     double* row_prev, double* row_cur, double* col) {
    const int n = p.wall_n;
    for (std::size_t k = 0; k < (std::size_t)(n - 1) * n; ++k) y[k] = 0.0;
    for (int i = 0; i < n_scene; ++i) {
#pragma HLS PIPELINE off
        const double xi = x[i];
        if (xi == 0.0) continue;
        emit_Ad_column(i, rect[i], wall_x, wall_z, mon_x, mon_z, p, col,
                       row_prev, row_cur);
        for (std::size_t k = 0; k < (std::size_t)(n - 1) * n; ++k) {
            y[k] += col[k] * xi;
        }
    }
}

// z = A_dᵀ @ r（r 长度 (n-1)*n，输出长度 N）。等价于把 A_d 的列与 r 内积。
inline void apply_AdT(int n_scene, const OccRect* rect,
                      const double* wall_x, const double* wall_z,
                      const double* mon_x, const double* mon_z,
                      const TransportParams& p, const double* r, double* z,
                      double* row_prev, double* row_cur, double* col) {
    const int n = p.wall_n;
    const std::size_t Md = (std::size_t)(n - 1) * n;
    for (int i = 0; i < n_scene; ++i) {
#pragma HLS PIPELINE off
        emit_Ad_column(i, rect[i], wall_x, wall_z, mon_x, mon_z, p, col,
                       row_prev, row_cur);
        double acc = 0.0;
        for (std::size_t k = 0; k < Md; ++k) {
#pragma HLS PIPELINE II = 1
            acc += col[k] * r[k];
        }
        z[i] = acc;
    }
}

}  // namespace ps

#endif  // PENUMBRASIGHT_TRANSPORT_HPP
