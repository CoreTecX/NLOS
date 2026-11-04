// ============================================================================
// PenumbraSight M4 定位核（候选投影引擎） · projection_engine.h
//
//   projection_engine.h  —— 接口 + psm_format 读取辅助（本文件）
//   projection_engine.cpp—— 可综合的核实现 + LANES 显式实例化
//   tb.cpp               —— 主机侧 testbench（g++ 自检，含朴素三重循环参照）
//
// 设计目标：同一份源码在两个环境下都能工作（照抄 src/hardware/recon.hpp 的约定）
//   g++       : 主机侧功能验证（不需要 Vivado），本目录综合专用 pragma 全部包在
//               #ifdef __SYNTHESIS__ 里，故主机编译对本文件零告警
//   Vitis HLS : 综合（工具自己定义 __SYNTHESIS__，pragma 照常生效）
//
// 依据
// ---------------------------------------------------------------------------
//   教师方案 §6.4 定位引擎：
//       s_p = Σ_c Σ_{k=1..K} ( Σ_m U_p[m,k] · d_c[m] )²
//       * 测量 d_R / d_G / d_B 常驻 BRAM；
//       * U 系数从 DDR 按 candidate → k → m 顺序流入；
//       * **每个系数同时乘三个颜色值**（系数复用，U 的 DDR 读带宽是每系数一次）；
//       * B 档默认 16 个 m 并行（A 档 8）；
//       * 维护最佳/次佳分数，margin = (s1 - s2) / (s1 + ε)；
//   教师方案 §6.3 定点格式：测量 Q2.16、U/V 系数 Q1.15（**每候选独立 scale**）、
//       MAC 累加 signed 48 bit、分数/残差 unsigned 48 或 64 bit。
//   configs/geometry_b.yaml：243 个候选（9 x 9 x 3）、loc_rank K = 8、
//       测量 95 x 96 = 9120、隐藏图 18 x 24 = 432。
//   src/software/include/psm_format.h：model.psm 的 rank-major 布局与访问器。
//
// 本层是「双精度参考层」
// ---------------------------------------------------------------------------
//   src/hardware/recon.hpp 是**双精度参考版**（其文首明写"用于主机侧与 Python
//   逐值对齐"）。本核与它同一层：系数、测量、分数全部用 double，用来
//     (a) 与朴素三重循环参考实现逐值对照；
//     (b) 给 §6.3 的定点层（同目录的 *_fixed.cpp / *.hpp）提供浮点侧的对齐基准。
//   定点层的判决（位宽、饱和、Q 格式）不在本文件；两者共用同一套访问器与布局。
//
// 与「既有定点层」共存时的命名约定
// ---------------------------------------------------------------------------
//   同目录下已存在 §6.3 的定点实现（projection_engine.hpp + projection_engine_fixed.cpp
//   + test_projection.cpp），其类型名 ProjResult / ProjStatus 已被占用。本层所有
//   可能跨层同名的类型统一加 Fp（floating-point）后缀，使两层头文件即使被同一个
//   翻译单元包含也不会重定义。函数名保持 projection_engine（参数类型不同即为
//   重载），与定点层一致，便于对照阅读。
//
// 布局（错了不会报错，只会整体错位）
// ---------------------------------------------------------------------------
//   U 为 **[K][M] rank-major**：元素 (k, m) 的字节偏移见 psm_format.h 的
//   ps_psm_read_U_km()。定位只读前 K 个 rank 时那 K*M 个 int16 在文件里连续。
//   meas3 为 **通道优先**：d_R = meas3[0..M)、d_G = meas3[M..2M)、d_B = meas3[2M..3M)。
//   score_ch 为 [P*3]，通道顺序同 meas3（R、G、B）。
//
// 绝对禁止
// ---------------------------------------------------------------------------
//   * 自己算 model.psm 的字节偏移。读 U 一律走 psm_format.h 的访问器
//     （ps_psm_read_U_km），候选记录起始一律走 ps_psm_candidate_offset()。
//   * 在核内把 int16 码字当实数用。码字必须乘候选头里的 u_scale（每候选不同）。
// ============================================================================

#ifndef PENUMBRASIGHT_HLS_PROJECTION_ENGINE_H
#define PENUMBRASIGHT_HLS_PROJECTION_ENGINE_H

#include <cstddef>
#include <cstdint>

#include "../../../software/include/psm_format.h"

namespace ps {

// ---------------------------------------------------------------- 编译期参数

// 每个周期的 m 并行度（§6.4）。B 档 16，A 档 8；32 留给冲刺档对照。
// 同一份源码在 8/16/32 下必须给出**逐位相同**的分数：内层按 m 升序把
// 单积依次加进累加器，尾块补零（加 0.0 是精确的），故与并行度无关。
#ifndef PS_PROJ_LANES_B
#define PS_PROJ_LANES_B 16
#endif
#ifndef PS_PROJ_LANES_A
#define PS_PROJ_LANES_A 8
#endif

// margin 的分母保护 ε（§6.4 写 s1 + ε）。取 1e-30：远小于任何真实分数，
// 又能挡住 s1 = 0 时的 0/0。
constexpr double kProjEpsFp = 1e-30;

// ---------------------------------------------------------------- 结论结构

enum ProjStatusFp {
    kProjOkFp      = 0,
    kProjBadArgsFp = 2
};

// §6.4 的输出契约：最佳/次佳 ID 与 margin。
struct ProjScoreFp {
    int    best_id;        // 最佳候选下标 = selected_id 的来源
    int    second_id;      // 次佳候选下标（并列时取小下标）
    double best_score;     // s1
    double second_score;   // s2
    double margin;         // (s1 - s2) / (s1 + ε)
};

// ---------------------------------------------------------------- psm 读取

// 读候选头（psm_format.h 声明的 20 字节 packed 结构，位于记录起始处）。
// 用 __builtin_memcpy 而不是指针强转：与 psm_format.h 内部读法定一致，
// 也避免未对齐解引用（记录 4096 对齐，但字段本身不保证对齐）。
inline void proj_cand_header_fp(const uint8_t* cand_base,
                                ps_psm_candidate_hdr_t& out) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE
#endif
    __builtin_memcpy(&out, cand_base, sizeof(out));
}

// 把某候选前 K 个 rank 的 U 还原成真实系数（码字 × 该候选的 u_scale）。
//   U_out[k*M + m] = ps_psm_read_U_km(cand_base, h, k, m) * h.u_scale
// 仅供主机侧参照/自检使用；核内部是**边流边算**，不物化这张表。
inline int proj_load_U_K_fp(const uint8_t* cand_base,
                            const ps_psm_header_t* h, int K, double* U_out) {
#ifdef __SYNTHESIS__
#pragma HLS INLINE off
#endif
    if (cand_base == 0 || h == 0 || U_out == 0 || K <= 0) return kProjBadArgsFp;
    if ((int)h->rank_max < K) return kProjBadArgsFp;
    ps_psm_candidate_hdr_t ch;
    proj_cand_header_fp(cand_base, ch);
    const uint64_t M = ps_psm_meas_len(h);
    const double usc = (double)ch.u_scale;
    for (int k = 0; k < K; ++k) {
        for (uint64_t m = 0; m < M; ++m) {
            U_out[(std::size_t)k * M + m] =
                (double)ps_psm_read_U_km(cand_base, h, (uint16_t)k, m) * usc;
        }
    }
    return kProjOkFp;
}

// ---------------------------------------------------------------- 核

// 候选投影引擎（§6.4）。
//
// 参数
//   meas3     [3*M]     归一化测量，通道优先：d_R | d_G | d_B
//   model_base          整份 model.psm 的起始字节（含 64 B 头部）
//   h                   头部（非空；形状与秩关系由调用方先过
//                       ps_psm_check_header，本核只做最小必要校验）
//   P                   候选数（B 档 243）
//   K                   参与打分的 rank 数（§5.1 定位 loc_rank = 8）
//   score_out [P]       每候选的总分数 s_p
//   score_ch  [P*3]     每候选每通道分数，可为 0（§6.2 的 243 x 3）
//   res                 最佳/次佳 ID 与 margin
//
// 返回 kProjOkFp / kProjBadArgsFp。
//
// 结构特征（三条必须保持）
//   P1 U 按 candidate → k → m 顺序流入，一个系数同乘三个颜色值（系数复用）。
//   P2 m 方向按 LANES 并行，尾块补零，整数式的「m 升序单积累加」保证
//      8/16/32 逐位相同。
//   P3 最佳/次佳用严格 `>` 比较，并列取小下标 —— 结果可复现。
template <int LANES>
int projection_engine(const double* meas3,
                      const uint8_t* model_base,
                      const ps_psm_header_t* h,
                      int P, int K,
                      double* score_out,
                      double* score_ch,
                      ProjScoreFp& res);

}  // namespace ps

#endif  // PENUMBRASIGHT_HLS_PROJECTION_ENGINE_H
