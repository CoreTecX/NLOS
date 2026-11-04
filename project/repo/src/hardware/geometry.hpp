// ============================================================================
// PenumbraSight M2 几何算核 · HLS 风格 C++
//
// 设计目标：同一份源码在两个环境下都能工作
//   g++       : 主机侧功能验证（不需要 Vivado）
//   Vitis HLS : 综合（定义 __SYNTHESIS__）
//
// 职责（X1 接口冻结第四节 M2）：由候选位置 ll 生成**遮挡矩形索引**，
// 不产出矩阵、不产出权重。
//
//   ll = (x 参考角, y 平面深度, z 参考角)        t = y / wall_y
//   遮挡判定 q ∈ 矩形（闭区间） 等价于 w 的两个独立区间约束：
//       w_x ∈ [ (x0 - (1-t)m_x)/t , (x1 - (1-t)m_x)/t ]
//       w_z ∈ [ (z0 - (1-t)m_z)/t , (z1 - (1-t)m_z)/t ]
//   墙网格轴对齐且等距，故每个隐藏点的遮挡区是「x 索引区间 x z 索引区间」的直积。
//
// 已验证（见 sim/results/x8a_rect_at_varying_t.md）：
//   1044 个隐藏点 x 5 个 t（覆盖搜索范围两端）= 5220 个组合，参考实现给出的
//   遮挡集合**全部**是轴对齐矩形；本文件的闭式索引换算与其差异为 0 / 8.29e7 元素。
//
// 关键实现约束（实测踩过的坑，务必保留）
// ---------------------------------------------------------------------------
// C1 区间两端必须**各自朝网格内部收**（下端 max(.,0)、上端 min(.,n-1)），
//    裁剪后 lo > hi 才表示「与网格无交」。若先把 lo、hi 各自 clip 到 [0,n-1]
//    再比较，则完全落在网格外的区间会被压成 1 格宽的矩形，**凭空多出遮挡**。
//    实测：t = 0.3398 时 667 / 1044 个隐藏点的 x 区间完全落在网格右侧，
//    错误做法会给每个这样的隐藏点凭空造出 37 x 38 的遮挡区。
// C2 索引换算用 double（综合时用足够宽的定点）。它是**整数判决**，边界错一格
//    会让 A_d 的单元素从 w 变成 0，相对误差量级 1/sqrt(15750) ≈ 0.8%，
//    直接击穿 X8 的 1e-3 判据。
// C3 每候选的仿射常数只算一次（见 GeomAffine），每个隐藏点只剩 2 次乘加。
// ============================================================================

#ifndef PENUMBRASIGHT_GEOMETRY_HPP
#define PENUMBRASIGHT_GEOMETRY_HPP

#include <cmath>
#include <cstdint>

// ---------------------------------------------------------------- 编译期规模
// 综合时通过 -D 覆盖；主机验证时用官方档默认值。

#ifndef PS_WALL_N
#define PS_WALL_N 126          // 墙网格边长
#endif
#ifndef PS_N_SCENE
#define PS_N_SCENE 1044        // 隐藏场景点数 = 29 x 36
#endif

// 索引换算的边界容差。用于吸收 lo/hi 与单元中心「恰好相等」时的浮点舍入。
// 每个单元 0.0031825 m，故 1e-9 相当于 3e-7 个单元的松弛量，
// 足以吸收 double 的舍入（~1e-16 相对），又远小于任何真实几何量。
#ifndef PS_IDX_EPS
#define PS_IDX_EPS 1e-9
#endif

namespace ps {

// ---------------------------------------------------------------- 几何参数

struct GeomParams {
    double wall_y;          // 墙平面 y
    double wall_x_min;      // FOV 下界（注意：是边界，不是首个单元中心）
    double wall_x_max;
    double wall_z_min;
    double wall_z_max;
    double occ_size_dx;     // 遮挡物尺寸
    double occ_size_dz;
    int    wall_n;          // 墙网格边长
};

// 遮挡矩形：墙网格上的闭索引区间，单位是索引（不是坐标）。
// 空集用 x_lo > x_hi 或 z_lo > z_hi 表示，用 rect_empty() 判定。
struct OccRect {
    int16_t x_lo;   // 列方向（x）下界
    int16_t x_hi;
    int16_t z_lo;   // 行方向（z）下界
    int16_t z_hi;
};

inline bool rect_empty(const OccRect& r) {
    return (r.x_lo > r.x_hi) || (r.z_lo > r.z_hi);
}

// 空集标记。约定 x_lo > x_hi 与 z_lo > z_hi 同时成立，便于下游只查一个轴。
inline OccRect rect_make_empty() {
    OccRect r;
    r.x_lo = 1; r.x_hi = 0;
    r.z_lo = 1; r.z_hi = 0;
    return r;
}

// ---------------------------------------------------------------- 候选仿射

// 把每个隐藏点的区间端点化成 A - B*m 的形式（C3）：
//     lo_x[i] = x0/t - ((1-t)/t) * m_x[i] = ax0 - b * m_x[i]
//     hi_x[i] = x1/t - ((1-t)/t) * m_x[i] = ax1 - b * m_x[i]
// 注意 b 对 x、z 两个轴**相同**（因为 (1-t)/t 只依赖 t），
// 所以每候选只需 3 个乘法算出 ax0/ax1/az0/az1 与 b。
struct GeomAffine {
    double ax0, ax1, az0, az1;
    double b;
    double inv_cell_x, inv_cell_z;   // 1 / 单元边长
    double ctr_x, ctr_z;             // 首个单元中心
    int    n;
};

// 由候选位置配置仿射系数。每个候选调用一次。
inline void geom_configure(const double ll[3], const GeomParams& g,
                           GeomAffine& a) {
    const double inv_t = g.wall_y / ll[1];
    a.ax0 = ll[0] * inv_t;
    a.ax1 = (ll[0] + g.occ_size_dx) * inv_t;
    a.az0 = ll[2] * inv_t;
    a.az1 = (ll[2] + g.occ_size_dz) * inv_t;
    a.b = inv_t - 1.0;                      // (1 - t) / t = 1/t - 1

    const double cell_x = (g.wall_x_max - g.wall_x_min) / (double)g.wall_n;
    const double cell_z = (g.wall_z_max - g.wall_z_min) / (double)g.wall_n;
    a.inv_cell_x = 1.0 / cell_x;
    a.inv_cell_z = 1.0 / cell_z;
    a.ctr_x = g.wall_x_min + cell_x * 0.5;
    a.ctr_z = g.wall_z_min + cell_z * 0.5;
    a.n = g.wall_n;
}

// 连续区间 [lo, hi] -> 单元中心网格上的闭索引区间（含 C1 的裁剪）。
// 与 Python 参考保持同一算式顺序（先减首个中心、再乘单元倒数），
// 以便主机测试能逐值比对；代数上等价的改写会改变舍入、破坏边界并列的可复现性。
inline void interval_to_index(double lo, double hi, double ctr, double inv_cell,
                              int n, int16_t& j_lo, int16_t& j_hi) {
    const double u_lo = (lo - ctr) * inv_cell;
    const double u_hi = (hi - ctr) * inv_cell;
    int a = (int)std::ceil(u_lo - PS_IDX_EPS);
    int b = (int)std::floor(u_hi + PS_IDX_EPS);
    if (a < 0) a = 0;            // C1：只朝内收，不越过另一端点
    if (b > n - 1) b = n - 1;
    j_lo = (int16_t)a;
    j_hi = (int16_t)b;
}

// ---------------------------------------------------------------- 顶层

// 单个隐藏点的遮挡矩形。mx/mz 为该隐藏点的 x / z 坐标。
inline OccRect geometry_one(double mx, double mz, const GeomAffine& a) {
    OccRect r;
#pragma HLS INLINE
    // lo_x = ax0 - b*mx,  hi_x = ax1 - b*mx；z 同理
    const double lo_x = a.ax0 - a.b * mx;
    const double hi_x = a.ax1 - a.b * mx;
    const double lo_z = a.az0 - a.b * mz;
    const double hi_z = a.az1 - a.b * mz;

    interval_to_index(lo_x, hi_x, a.ctr_x, a.inv_cell_x, a.n, r.x_lo, r.x_hi);
    interval_to_index(lo_z, hi_z, a.ctr_z, a.inv_cell_z, a.n, r.z_lo, r.z_hi);
    return r;
}

// 一次算完整个隐藏场景（N 个点）。综合时展开为流水线。
// 注：必须是 inline —— 它定义在头文件里，而 host 侧常有多个 TU 同时包含
// geometry.hpp（例如 pdhg_core_fixed.cpp + test_pdhg.cpp），非 inline 会
// 在链接期报 multiple definition。
inline void m2_geometry(const double ll[3], const GeomParams& g,
                        const float* mon_x, const float* mon_z,
                        OccRect* rect, int n_scene) {
#pragma HLS ARRAY_PARTITION variable=rect cyclic factor=4 dim=1
    GeomAffine a;
    geom_configure(ll, g, a);
    for (int i = 0; i < n_scene; ++i) {
#pragma HLS PIPELINE II = 1
        rect[i] = geometry_one((double)mon_x[i], (double)mon_z[i], a);
    }
}

// ---------------------------------------------------------------- 权重侧因子
//
// M2 只产出索引；权重不属于 M2。但有一组**只依赖候选 y 与隐藏点**的量
// 适合在这里顺带产出，避免每候选在 M3 里重算：见 transport.hpp 的说明。

}  // namespace ps

#endif  // PENUMBRASIGHT_GEOMETRY_HPP
