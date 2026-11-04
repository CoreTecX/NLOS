// ============================================================================
// PenumbraSight M4 定位核（候选投影引擎） · projection_engine.cpp
//
// 依据、布局、命名约定见 projection_engine.h 文首。这里只写实现，顺序严格照抄
// 方案 §6.4：
//
//   for p in 0..P-1:                       // 候选
//     for k in 0..K-1:                     // rank
//       for m0 in 0, LANES, 2*LANES, ...:  // m 方向 LANES 并行
//         u = U_code[p][k][m] * u_scale    // 一次读取
//         mac[t][0] += u * d_R[m]          // 同一个 u 同乘三个颜色值
//         mac[t][1] += u * d_G[m]          //   （系数复用，P1）
//         mac[t][2] += u * d_B[m]
//       acc[c] += Σ_t mac[t][c]
//       ch[c]  += acc[c] * acc[c]          // 平方后按通道累加
//     s_p = ch[0] + ch[1] + ch[2]
//
// 三个必须保持的结构特征
//   E1 系数复用：U 的 DDR 读带宽是「每系数一次」而不是三次（§6.4）。
//   E2 尾块补零，分支不落在乘法上；m 升序单积累加 -> 8/16/32 逐位相同。
//   E3 只读 model.psm 的访问器（ps_psm_read_U_km / ps_psm_candidate_offset），
//      不出现任何手算偏移。
// ============================================================================

#include "projection_engine.h"

namespace ps {

template <int LANES>
int projection_engine(const double* meas3,
                      const uint8_t* model_base,
                      const ps_psm_header_t* h,
                      int P, int K,
                      double* score_out,
                      double* score_ch,
                      ProjScoreFp& res) {
    // ---- 结论结构先置成「不可信」的初值：参数不对时报错而不是报可信 ----
    res.best_id = -1;
    res.second_id = -1;
    res.best_score = 0.0;
    res.second_score = 0.0;
    res.margin = 0.0;

    if (meas3 == 0 || model_base == 0 || h == 0 || score_out == 0) {
        return kProjBadArgsFp;
    }
    if (P <= 0 || K <= 0) return kProjBadArgsFp;
    if ((int)h->rank_max < K) return kProjBadArgsFp;

    const int M = (int)ps_psm_meas_len(h);
    const int Nhidden = (int)ps_psm_hidden_len(h);
    if (M <= 0 || Nhidden <= 0) return kProjBadArgsFp;
    (void)Nhidden;

    double best = 0.0;
    double second = 0.0;
    int best_id = -1;
    int second_id = -1;

    for (int p = 0; p < P; ++p) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        // 记录起始只走 psm_format.h 的访问器。
        const uint8_t* cand =
            model_base + ps_psm_candidate_offset(h, (uint32_t)p);

        ps_psm_candidate_hdr_t ch;
        proj_cand_header_fp(cand, ch);
        const double usc = (double)ch.u_scale;   // 每候选独立 scale（§6.3）

        double ch_acc[3];
        ch_acc[0] = 0.0;
        ch_acc[1] = 0.0;
        ch_acc[2] = 0.0;
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=ch_acc complete dim=1
#endif

        for (int k = 0; k < K; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
            double acc[3];
            acc[0] = 0.0;
            acc[1] = 0.0;
            acc[2] = 0.0;
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=acc complete dim=1
#endif

            for (int m0 = 0; m0 < M; m0 += LANES) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II = 1
#endif
                double mac[LANES][3];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=mac complete dim=0
#endif
                for (int t = 0; t < LANES; ++t) {
                    mac[t][0] = 0.0;
                    mac[t][1] = 0.0;
                    mac[t][2] = 0.0;
                }
                // 一次 U 读，同乘三个颜色值（P1 系数复用）
                for (int t = 0; t < LANES; ++t) {
                    const int m = m0 + t;
                    if (m < M) {
                        const double u =
                            (double)ps_psm_read_U_km(cand, h, (uint16_t)k,
                                                     (uint64_t)m) * usc;
                        mac[t][0] = u * meas3[m];
                        mac[t][1] = u * meas3[M + m];
                        mac[t][2] = u * meas3[2 * M + m];
                    }
                }
                // 尾块补零后按 m 升序合并（加 0.0 是精确的 -> 并行度无关）
                for (int t = 0; t < LANES; ++t) {
                    acc[0] += mac[t][0];
                    acc[1] += mac[t][1];
                    acc[2] += mac[t][2];
                }
            }

            // s_p 的 Σ_k (·)² 部分：按通道各自累加（§6.4 的 Σ_c Σ_k 顺序）
            ch_acc[0] += acc[0] * acc[0];
            ch_acc[1] += acc[1] * acc[1];
            ch_acc[2] += acc[2] * acc[2];
        }

        if (score_ch != 0) {
            score_ch[(std::size_t)p * 3 + 0] = ch_acc[0];
            score_ch[(std::size_t)p * 3 + 1] = ch_acc[1];
            score_ch[(std::size_t)p * 3 + 2] = ch_acc[2];
        }

        const double tot = ch_acc[0] + ch_acc[1] + ch_acc[2];
        score_out[p] = tot;

        // 严格 `>`：并列取小下标，结果可复现（P3）
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
        res.second_score = (second_id >= 0) ? second : 0.0;
        res.margin = (best > 0.0)
            ? (best - res.second_score) / (best + kProjEpsFp)
            : 0.0;
    }
    return kProjOkFp;
}

// ---------------------------------------------------------------- 实例化
// §6.4：B 档默认 16，A 档 8；32 留给冲刺档对照。
#define PS_INSTANTIATE_PROJECTION_FP(L)                                       \
    template int projection_engine<L>(const double*, const uint8_t*,          \
                                      const ps_psm_header_t*, int, int,       \
                                      double*, double*, ProjScoreFp&);

PS_INSTANTIATE_PROJECTION_FP(8)
PS_INSTANTIATE_PROJECTION_FP(16)
PS_INSTANTIATE_PROJECTION_FP(32)

#undef PS_INSTANTIATE_PROJECTION_FP

// ---------------------------------------------------------------- HLS 顶层包装
// Vitis HLS 的 top 必须是**具体函数**，不能是模板，故给一个非模板入口。
// LANES 固定为 PS_PROJ_LANES_B（B 档 16）；改并行度只改这一处。
int projection_engine_top(const double* meas3,
                          const uint8_t* model_base,
                          const ps_psm_header_t* h,
                          int P, int K,
                          double* score_out,
                          double* score_ch,
                          ProjScoreFp* res) {
    if (res == 0) return kProjBadArgsFp;
    return projection_engine<PS_PROJ_LANES_B>(meas3, model_base, h, P, K,
                                             score_out, score_ch, *res);
}

}  // namespace ps
