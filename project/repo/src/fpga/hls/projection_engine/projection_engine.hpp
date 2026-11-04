// ============================================================================
// PenumbraSight M4 定位核（候选投影引擎） · projection_engine
//
//   projection_engine.hpp  —— 接口与全部定标约定（本文件）
//   projection_engine_fixed.cpp —— 可综合的核实现 + LANES 显式实例化
//                                  （_fixed 后缀：同目录另有双精度参考层，
//                                   见 src/fpga/hls/readme.md）
//   test_projection.cpp    —— 主机侧 testbench（g++ 8.1，自带自检）
//
// 依据
// ---------------------------------------------------------------------------
//   方案 §6.2 模块接口（projection_engine：输入 测量BRAM / 候选U_K / P,K,M，
//                         输出 243×3 分数、最佳 ID、margin）
//   方案 §6.4 定位引擎（s_p 的式子、16 个 m 并行、RGB 系数复用、margin 定义、
//                       网格边界与低 margin 必须报「位置不可靠」）
//   方案 §5.3 带宽与计算量（定位标量 MAC 数 P·M·K·3）
//   configs/fixed_point_b.yaml（Q2.16 测量、Q1.15 U 系数、48 bit MAC、64 bit 分数）
//
// 数学
// ---------------------------------------------------------------------------
//   s_p = Σ_c Σ_{k=1..K} ( Σ_{m=1..M} U_p[m,k] d_c[m] )²
//
//   定标：measurement 是 Q2.16 码字，尺度 d_scale 来自**每帧共享指数**
//   （quantize.shared_exponent）。U 是 Q1.15 码字，尺度 u_scale 是**每候选独立**
//   （model.psm 候选头里的 float32）。于是
//
//       s_p = Σ_c Σ_k ( acc_{c,k} · u_scale_p · d_scale )²
//           = (u_scale_p · d_scale)² · Σ_c Σ_k acc_{c,k}²
//
//   两个尺度合成**一个** Q16.16 增益 gain_q16[p] = round(u_scale_p·d_scale·2^16)，
//   由 PS 侧 model_loader 在装载模型时算好（243 个 int32），核内不出现浮点。
//   这样做的三个好处，都是可验证的：
//     1) 核内全整数 -> 与 Python bit-exact 定点模型逐码字可比（§6.3 末段判据）；
//     2) 增益不改变「同一候选」内部的相对量，只影响候选间的排序，所以增益的
//        Q16.16 取整误差是**整体相对误差**，不会破坏 margin 的方向；
//     3) gain 的合法范围是 |u_scale·d_scale| < 1，即 |gain_q16| < 2^16，
//        于是 |acc|·|gain| < 2^47·2^16 = 2^63，int64 乘法不会溢出（有判据守着）。
//
// 定点链（每一步都显式，禁止「代数等价改写」）
// ---------------------------------------------------------------------------
//   1. MAC      acc_{c,k} = Σ_m U_code[k·M+m] · d_code[c·M+m]     int64，48 bit 政策
//   2. 增益      p = mul_shift_round(acc, gain_q16, 16)             实数口径的投影系数
//   3. 平方      sq = p²（|p| > 2^31 时按不可信处理：饱和 + score_overflow）
//   4. 累加      score += sq >> PS_PROJ_ENERGY_SHIFT（uint64，饱和，不回绕）
//
//   关于第 4 步的右移：score 的绝对尺度不影响候选排序（同一帧对全部候选是同一个
//   常数），但方案 §6.3 明确 score 是「unsigned 48 或 64 bit」，所以必须保证不越界。
//   本实现默认 PS_PROJ_ENERGY_SHIFT = 0，即不做额外归一，靠 testbench 打印
//   「实测余量位数」来确认 64 bit 够用；若将来测量动态范围变大导致余量不足，
//   只需加大这个常量（单调变换，不改排序，也不用改任何其它代码）。
//
// 与既有原语的复用
// ---------------------------------------------------------------------------
//   * ps_fixed.hpp         ：Q 格式、偶数舍入+饱和、增益、位宽链（quantize.py 对偶）
//   * ../hardware/geometry.hpp：候选网格与几何约定。本文件的 proj_grid_index()
//     与 src/hardware/x10_localize.cpp 的候选展开顺序**逐式一致**
//     （ix = k/(ny*nz)，iy = (k%(ny*nz))/nz，iz = k%nz），保证 FPGA 输出的
//     selected_id 与 PC 侧 Gram/定位路径落在同一个网格点上 —— 这条一旦不一致，
//     板上不会报错，只会「定位到隔壁」。
//   * ../hardware/transport.hpp / gram.hpp：可见性与 A_d 列的既有实现（M2/M3）。
//     本核吃的是它们产出的 U_K 因子，不重算几何。
// ============================================================================

#ifndef PENUMBRASIGHT_HLS_PROJECTION_ENGINE_HPP
#define PENUMBRASIGHT_HLS_PROJECTION_ENGINE_HPP

#include <cstddef>
#include <cstdint>

#include "../ps_fixed.hpp"

namespace ps {

// ---------------------------------------------------------------- 编译期参数

// 能量累加前的归一化右移（见文首第 4 步）。0 = 不归一。改大只影响绝对尺度。
#ifndef PS_PROJ_ENERGY_SHIFT
#define PS_PROJ_ENERGY_SHIFT 0
#endif

// |p| 超过 2^31 时 p² 会越过 62 bit，按「不可信」处理（饱和 + 记 overflow）。
#ifndef PS_PROJ_SQ_MAX
#define PS_PROJ_SQ_MAX ((1LL << 62) - 1)
#endif

// ---------------------------------------------------------------- 结论结构

enum ProjStatus {
    kProjOk      = 0,
    kProjBadArgs = 2
};

// 与方案 §6.2 的输出契约对齐：best_id / second_id / margin_q16，
// 外加 §6.4 末条要求的两个门控位（margin 过低、最佳在网格边界）。
struct ProjResult {
    int      best_id;        // 最佳候选下标（= selected_id 的来源）
    int      second_id;      // 次佳候选下标
    int      margin_q16;     // Q0.16，(s1-s2)/(s1+eps)，eps = 1 个分数 LSB
    int      boundary;       // 1 = 最佳候选落在候选网格边界
    int      low_conf;       // 1 = margin < 阈值 或 boundary（§6.4：位置不可靠）
    int      overflow;       // 1 = 任一级饱和（FixFlags::any()）
    uint64_t best_score;     // s1
    uint64_t second_score;   // s2
};

// ---------------------------------------------------------------- 候选网格

// 线性候选号 -> 三维网格下标。顺序与 src/hardware/x10_localize.cpp 完全一致
// （x 最外层、z 最内层），也与方案 §5.1「网格以标称遮挡位置为中心」的展开一致。
// 任一维 <= 0 时返回 -1，表示调用方没有网格信息（例如固定位置模式）。
inline void proj_grid_index(int id, int nx, int ny, int nz,
                            int& ix, int& iy, int& iz) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (nx <= 0 || ny <= 0 || nz <= 0 || id < 0) {
        ix = iy = iz = -1;
        return;
    }
    const int per_x = ny * nz;
    ix = id / per_x;
    const int rem = id - ix * per_x;
    iy = rem / nz;
    iz = rem - iy * nz;
}

// 最佳候选是否落在网格边界。语义（保守，宁可多报不可漏报）：
//   * 任一分量取到 0 或 n-1 -> 边界；
//   * 某一维只有 1 个取值（n == 1）-> 该维没有邻点、无法比较，也视为边界。
// 方案 §6.4 要求「最佳位置在网格边界 -> 输出位置不可靠，禁止伪装成精确自定位」。
inline int proj_on_boundary(int id, int nx, int ny, int nz) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (nx <= 0 || ny <= 0 || nz <= 0) return 0;
    int ix, iy, iz;
    proj_grid_index(id, nx, ny, nz, ix, iy, iz);
    if (ix == 0 || ix == nx - 1) return 1;
    if (iy == 0 || iy == ny - 1) return 1;
    if (iz == 0 || iz == nz - 1) return 1;
    return 0;
}

// ---------------------------------------------------------------- margin

// margin = (s1 - s2) / (s1 + eps)，eps = 1（分数的一个 LSB）防止除零。
// 输出 Q0.16，饱和到 65536。
//
// 为避免 (s1-s2)<<16 越界，先把 s1、s2 **同时**右移同一个位数把 s1 压到 2^31 以下：
// 同除一个数不改比值，损失的只有低位精度（保留 >= 31 位有效位），对门控无影响。
inline int proj_margin_q16(uint64_t s1, uint64_t s2) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    if (s1 == 0) return 0;
    int sh = 0;
    while (sh < 63 && (s1 >> sh) > (1ull << 31)) ++sh;
    const uint64_t a = s1 >> sh;
    const uint64_t b = s2 >> sh;
    const uint64_t num = (a > b) ? (a - b) : 0ull;
    const uint64_t den = a + 1ull;                     // eps = 1 LSB
    const uint64_t m = (num << 16) / den;              // num<<16 <= 2^47，安全
    return (int)((m > 65535ull) ? 65535ull : m);
}

// ---------------------------------------------------------------- 核

// LANES = 每个周期的 m 并行度：B 档默认 16（3 通道共享 U 读需要 48 个 DSP，
// 方案 §6.4），A 档 8。同一份源码在 8/16/32 下必须给出**逐位相同**的结果 ——
// MAC 全在 int64 里做整数累加，整数加法可结合，所以这条是构造性成立的，
// 前提是没触发 48 bit 饱和（testbench 会同时验证这两点）。
//
// 参数
//   meas      [3*M]      Q2.16 归一化测量码字，通道优先：d_R | d_G | d_B
//   u_codes   [P*K*M]    U 系数，rank-major：候选 p、rank k、测量 m 的顺序，
//                        即 u_codes[p*K*M + k*M + m] —— 与 model.psm 一致，
//                        也与方案 §5.3「定位只顺序读前 K 个 rank」一致
//   gain_q16  [P]        round(u_scale[p] · d_scale · 2^16)，PS 侧算好
//   score_out [P]        总分数（三通道求和）
//   score_ch  [P*3]      每候选每通道分数，可为 nullptr（方案 §6.2 的 243×3）
template <int LANES>
int projection_engine(const int32_t* meas,
                      const int16_t* u_codes,
                      const int32_t* gain_q16,
                      int P, int K, int M,
                      int nx, int ny, int nz,
                      int margin_thresh_q16,
                      uint64_t* score_out,
                      uint64_t* score_ch,
                      ProjResult& res,
                      FixFlags& flags);

// 供 testbench/PS 侧使用的双精度参考：从同样的码字与增益算 s_p（实数口径）。
// 只在主机侧存在（综合时不编入），用途是给核的排序与幅度提供独立对照。
#ifndef __SYNTHESIS__
double projection_score_reference(const int32_t* meas,
                                  const int16_t* u_codes,
                                  const int32_t* gain_q16,
                                  int p, int K, int M);
#endif

}  // namespace ps

#endif  // PENUMBRASIGHT_HLS_PROJECTION_ENGINE_HPP
