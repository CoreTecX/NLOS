// ============================================================================
// PenumbraSight M4 定位核（候选投影引擎） · projection_engine.cpp
//
// 全部定标约定与推导见 projection_engine.hpp 的文首；这里只写实现。
//
// 三个必须保持的结构特征
//   E1 RGB 共享 U 读流、累加器独立（方案 §6.4）。内层一次读一个 U 系数，同时乘
//      三个颜色通道，所以 U 的 DDR 读带宽是「每系数一次」而不是三次。
//   E2 MAC 阵列宽度由 LANES 决定，尾块补零。整数累加可结合，8/16/32 必须逐位相同。
//   E3 所有量化点走 ps_fixed 的 mul_shift_round / narrow_format（偶数舍入 + 饱和），
//      绝不出现 `(int)x` 截断。
// ============================================================================

#include "projection_engine.hpp"

// ---- 修法 C 的分块宽度 ----
// 一次 m0 遍历里复用 meas 的 rank 个数。含义与推导见
// sim/results/bw_cycle_model_and_fix_c.md；改并行度只需改这一个数。
//   越大 -> meas 的复用倍数越高，但片上累加器 acc[KS][3] 与一个 m0 迭代内的
//           MAC 次数也按 KS 增长（后者会推高 II）。
//   取 8 与 B 档定位的 rank 数（K = 8）相同，此时 meas 每个 m0 块恰好只读一遍。
#ifndef PS_PROJ_KS
#define PS_PROJ_KS 8
#endif

// ---- 修法 C 第二步的宽读倍数 ----
// 一次从 u_codes 取 PS_PROJ_UW 个 int16（默认 2，即与 32 bit 的 M_AXI 端口对齐）。
// 这样每个 m0 轮在 gmem0 上的读操作数从 kb×LANES 降到 kb×LANES/PS_PROJ_UW
// （见 sim/results/bw_cycle_model_and_fix_c.md §5：改后瓶颈正是这 kb×LANES 次逐字读）。
//   取 1 = 退回逐字读（对照用）；2 = 32 bit 读；4 = 64 bit 读（跨两个 32 bit 拍）。
// 前提：行内元素下标 (k·M + m0 + t) 相对 PS_PROJ_UW 对齐 ——
//   m0 是 LANES 的倍数、LANES 是 2 的幂且 >= 8，故 m0 必为 PS_PROJ_UW 的倍数；
//   只要 M 也是 PS_PROJ_UW 的倍数，则 k·M 对齐，整段读都落在 PS_PROJ_UW 边界上。
// M 不是 PS_PROJ_UW 的倍数时自动退回逐字读（数值完全相同，只是慢）。
#ifndef PS_PROJ_UW
#define PS_PROJ_UW 2
#endif

namespace ps {

namespace proj_detail {

// 宽读字类型：与 PS_PROJ_UW 一一对应。
#if PS_PROJ_UW >= 4
typedef int64_t proj_uword_t;
#elif PS_PROJ_UW == 2
typedef int32_t proj_uword_t;
#else
typedef int16_t proj_uword_t;
#endif

// 一个 k0 块里「meas 一次读入、kb 个 rank 复用」的 MAC 主体（修法 C 第一步 + 第二步）。
//   WIDE=false：u 逐 int16 读（通用路径，M 为任意值时都用它）
//   WIDE=true ：u 按 PS_PROJ_UW 个 int16 一次读（调用方保证 M % PS_PROJ_UW == 0）
// 两条路径的数值逐位相同：
//   * 累加顺序一致（m0 由小到大，块内 t 由小到大）；
//   * acc 是 int64 精确累加、每次累加不加饱和（sat_mac48 只在块末用一次），
//     故「先分组再累加」与「逐项累加」给出同一个整数，与分组方式无关。
//   —— 这正是 E2 不变式（LANES 8/16/32 逐位相同）在宽读下的延续，U6 在守它。
template <int LANES, bool WIDE>
static void proj_mac_kblock(const int16_t* Uc, int k0, int kb, int M,
                            const int32_t* meas,
                            int64_t (&acc)[PS_PROJ_KS][3]) {
    int m0 = 0;
    // ---- 完整块：块内不越界，去掉逐元素守卫，两个访存都可做宽读 ----
    for (; m0 + LANES <= M; m0 += LANES) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
        int32_t mblk[3][LANES];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=mblk complete dim=0
#endif
        for (int c = 0; c < 3; ++c) {
            for (int t = 0; t < LANES; ++t) {
                mblk[c][t] = meas[(std::size_t)c * (std::size_t)M + m0 + t];
            }
        }

        for (int kk = 0; kk < kb; ++kk) {
            const int16_t* urow =
                Uc + (std::size_t)(k0 + kk) * (std::size_t)M;
            if (WIDE) {
                const proj_uword_t* uw = (const proj_uword_t*)urow;
                for (int t = 0; t < LANES; t += PS_PROJ_UW) {
                    const uint64_t w = (uint64_t)uw[(m0 + t) / PS_PROJ_UW];
                    for (int s = 0; s < PS_PROJ_UW; ++s) {
                        const int16_t u = (int16_t)(w >> (16 * s));
                        acc[kk][0] += (int64_t)u * (int64_t)mblk[0][t + s];
                        acc[kk][1] += (int64_t)u * (int64_t)mblk[1][t + s];
                        acc[kk][2] += (int64_t)u * (int64_t)mblk[2][t + s];
                    }
                }
            } else {
                for (int t = 0; t < LANES; ++t) {
                    const int16_t u = urow[m0 + t];
                    acc[kk][0] += (int64_t)u * (int64_t)mblk[0][t];
                    acc[kk][1] += (int64_t)u * (int64_t)mblk[1][t];
                    acc[kk][2] += (int64_t)u * (int64_t)mblk[2][t];
                }
            }
        }
    }

    // ---- 尾块（不足一个 LANES，最多一个）：逐元素守卫、逐字读，两条路径共用 ----
    if (m0 < M) {
        int32_t mblk[3][LANES];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=mblk complete dim=0
#endif
        for (int c = 0; c < 3; ++c) {
            for (int t = 0; t < LANES; ++t) {
                const int m = m0 + t;
                mblk[c][t] =
                    (m < M) ? meas[(std::size_t)c * (std::size_t)M + m] : 0;
            }
        }
        for (int kk = 0; kk < kb; ++kk) {
            const int16_t* urow =
                Uc + (std::size_t)(k0 + kk) * (std::size_t)M;
            for (int t = 0; t < LANES; ++t) {
                const int m = m0 + t;
                if (m < M) {                  // 尾块补零，不加分支到 MAC 上
                    const int16_t u = urow[m];
                    acc[kk][0] += (int64_t)u * (int64_t)mblk[0][t];
                    acc[kk][1] += (int64_t)u * (int64_t)mblk[1][t];
                    acc[kk][2] += (int64_t)u * (int64_t)mblk[2][t];
                }
            }
        }
    }
}

}  // namespace proj_detail

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
                      FixFlags& flags) {
    res.best_id = -1;
    res.second_id = -1;
    res.margin_q16 = 0;
    res.boundary = 0;
    res.low_conf = 1;          // 参数不对时必须报「不可信」，不是报「可信但无解」
    res.overflow = 0;
    res.best_score = 0;
    res.second_score = 0;

    if (P <= 0 || K <= 0 || M <= 0) return kProjBadArgs;
    if (meas == 0 || u_codes == 0 || gain_q16 == 0 || score_out == 0) {
        return kProjBadArgs;
    }

    uint64_t best = 0;
    uint64_t second = 0;
    int best_id = -1;
    int second_id = -1;

    for (int p = 0; p < P; ++p) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        // 每候选一个 Q16.16 增益（= u_scale_p · d_scale）。越界意味着模型 scale
        // 异常，钳位并记账：钳位是**缩小**，排序仍单调，但结果已被标为不可信。
        const long long gain =
            gain_safe((long long)gain_q16[p], flags);

        const int16_t* Uc = u_codes
            + (std::size_t)p * (std::size_t)K * (std::size_t)M;

        uint64_t ch[3];
        ch[0] = 0;
        ch[1] = 0;
        ch[2] = 0;
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=ch complete dim=1
#endif

        // ---- 修法 C：meas 在 rank 方向复用 ----
        // 原结构是 `for k { for m0 { ... } }`，而 meas **与 k 无关** —— 同一段 meas
        // 被 K 个 rank 各读了一遍，这 K 倍重复就是定位核 532.4 ms 的全部来源。
        // 周期模型（与实测逐位吻合，见 sim/results/bw_cycle_model_and_fix_c.md）：
        //     P × K × (M / LANES) × (3 × LANES)
        // 改成 `for k0 { for m0 { 载入 meas 块; for kk } }` 之后，meas 每个 m0 块只
        // 读一次、由 KS 个 rank 复用，读次数降到 1/KS。
        //
        // 数值不变：每个 acc[kk][c] 内部的累加顺序与原来逐项一致（m0 由小到大、
        // 块内 t 由小到大），整数加法可结合，所以 LANES = 8/16/32 仍然逐位相同
        // —— 这是 E2 不变式，test_projection.cpp 的 L5/U6 段在守它。
        for (int k0 = 0; k0 < K; k0 += PS_PROJ_KS) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
            const int kb = (K - k0 < PS_PROJ_KS) ? (K - k0) : PS_PROJ_KS;

            // ---- 1. MAC：meas 一次读入，kb 个 rank 复用（E1 的加强版）----
            int64_t acc[PS_PROJ_KS][3];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=acc complete dim=0
#endif
            for (int kk = 0; kk < PS_PROJ_KS; ++kk) {
                acc[kk][0] = 0;
                acc[kk][1] = 0;
                acc[kk][2] = 0;
            }

            // ---- 1. MAC：meas 一次读入，kb 个 rank 复用；u 走宽读 ----
            // 宽读判据：M 是 PS_PROJ_UW 的倍数时才用宽读（对齐前提见文件头注释），
            // 否则退回逐字读。两条路径的数值逐位相同（见 proj_mac_kblock 的注释）。
            if ((PS_PROJ_UW > 1) && ((M % PS_PROJ_UW) == 0)) {
                proj_detail::proj_mac_kblock<LANES, true>(Uc, k0, kb, M, meas, acc);
            } else {
                proj_detail::proj_mac_kblock<LANES, false>(Uc, k0, kb, M, meas, acc);
            }

            // ---- 2/3/4. 增益 -> 平方 -> 分数累加（每个 rank 各一次）----
            for (int kk = 0; kk < kb; ++kk) {
                for (int c = 0; c < 3; ++c) {
                    const int64_t a = sat_mac48(acc[kk][c], flags);   // 48 bit 政策
                    const long long pz = mul_shift_round(a, gain, kGainFrac);
                    long long sq;
                    if (pz > (1LL << 31) || pz < -(1LL << 31)) {
                        // |p| > 2^31 时 p² > 2^62，会越过 64 bit 分数通路；
                        // 方案 §6.6 的处置是「标为不可信」，不是回绕。
                        sq = (long long)PS_PROJ_SQ_MAX;
                        flags.score_overflow++;
                        flags.sat_events++;
                    } else {
                        sq = pz * pz;
                    }
                    sq >>= PS_PROJ_ENERGY_SHIFT;          // 默认 0，单调变换
                    ch[c] = sat_add_u64(ch[c], (uint64_t)sq, flags);
                }
            }
        }

        if (score_ch != 0) {
            score_ch[(std::size_t)p * 3 + 0] = ch[0];
            score_ch[(std::size_t)p * 3 + 1] = ch[1];
            score_ch[(std::size_t)p * 3 + 2] = ch[2];
        }

        uint64_t tot = sat_add_u64(ch[0], ch[1], flags);
        tot = sat_add_u64(tot, ch[2], flags);
        score_out[p] = tot;

        // 最佳/次佳：`>` 严格比较，使并列时取小下标 —— 结果可复现（§6.4）
        if (best_id < 0 || tot > best) {
            second = best;
            second_id = best_id;
            best = tot;
            best_id = p;
        } else if (second_id < 0 || tot > second) {
            second = tot;
            second_id = p;
        }
    }

    if (best_id >= 0) {
        res.best_id = best_id;
        res.second_id = second_id;
        res.best_score = best;
        res.second_score = (second_id >= 0) ? second : 0ull;
        res.margin_q16 = proj_margin_q16(res.best_score, res.second_score);
        res.boundary = proj_on_boundary(best_id, nx, ny, nz);
        res.low_conf =
            (res.margin_q16 < margin_thresh_q16 || res.boundary != 0) ? 1 : 0;
    }
    res.overflow = flags.any() ? 1 : 0;
    return kProjOk;
}

// ---------------------------------------------------------------- 主机参考
#ifndef __SYNTHESIS__
double projection_score_reference(const int32_t* meas,
                                  const int16_t* u_codes,
                                  const int32_t* gain_q16,
                                  int p, int K, int M) {
    // 参考刻意用**同一个已取整的 gain_q16**，并且把投影系数也按同一个整数规则
    // 取整（pz 是整数）。于是核与参考的差只可能来自移位/平方/累加的实现错误，
    // 不含增益取整误差；两者在双精度里应当**逐候选相等**（所有中间量都是整数，
    // 且都远小于 2^53）。增益本身的取整误差由 testbench 的浮点参考单独量。
    if (p < 0 || K <= 0 || M <= 0) return 0.0;
    const double g = (double)gain_q16[p] / (double)(1 << kGainFrac);
    const int16_t* Uc = u_codes + (std::size_t)p * (std::size_t)K * (std::size_t)M;
    double tot = 0.0;
    for (int c = 0; c < 3; ++c) {
        const int32_t* dc = meas + (std::size_t)c * (std::size_t)M;
        for (int k = 0; k < K; ++k) {
            const int16_t* urow = Uc + (std::size_t)k * (std::size_t)M;
            double acc = 0.0;
            for (int m = 0; m < M; ++m) {
                acc += (double)urow[m] * (double)dc[m];
            }
            acc = std::floor(acc);            // 整数累加（双精度里精确）
            // 与 mul_shift_round 同一舍入语义；这里用 std::rint（半值取偶），
            // 与 ps_fixed::round_half_even 等价
            const long long pz = (long long)std::rint(acc * g);
            long long sq;
            if (pz > (1LL << 31) || pz < -(1LL << 31)) {
                sq = (long long)PS_PROJ_SQ_MAX;    // 与核同一处置
            } else {
                sq = pz * pz;
            }
            sq >>= PS_PROJ_ENERGY_SHIFT;           // 默认 0
            tot += (double)sq;
        }
    }
    return tot;
}
#endif  // __SYNTHESIS__

// ---------------------------------------------------------------- 实例化
// 方案 §6.4：B 档默认 16（48 个 DSP），A 档 8；32 留给冲刺档对照。
#define PS_INSTANTIATE_PROJECTION(L)                                          \
    template int projection_engine<L>(                                        \
        const int32_t*, const int16_t*, const int32_t*, int, int, int,        \
        int, int, int, int, uint64_t*, uint64_t*, ProjResult&, FixFlags&);

PS_INSTANTIATE_PROJECTION(8)
PS_INSTANTIATE_PROJECTION(16)
PS_INSTANTIATE_PROJECTION(32)

#undef PS_INSTANTIATE_PROJECTION

}  // namespace ps
