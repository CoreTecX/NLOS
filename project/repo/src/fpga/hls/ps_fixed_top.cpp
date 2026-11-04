// ============================================================================
// ps_fixed_top.cpp —— 定点层 HLS 顶层接口的实现（A6 的定点对偶）
//
// 这一层只做三件事：**平坦入参 -> 结构体 -> 转调定点核 -> 逐字段回写**。
// 数学一行都不重写，所以它**不可能引入数值差异** —— 这是刻意的：
// 接口层的 bug 只应该表现为「连错线」，不应该表现为「结果不对」。
//
// 被包装的四个核（原样调用，一字不改；签名抄自各自 .hpp）：
//     ps::projection_engine<L>(meas, u_codes, gain_q16, P, K, M,
//                              nx, ny, nz, margin_thresh_q16,
//                              score_out, score_ch, ProjResult&, FixFlags&)
//     ps::lr_forward<L>(const LrFactors&, const int32_t* f, int32_t* y,
//                       LrDiag&, LrWork*)
//     ps::lr_adjoint<L>(const LrFactors&, const int32_t* r, int32_t* g,
//                       LrDiag&, LrWork*)
//     ps::pdhg_core<L>(const LrFactors&, const int32_t* d3, const ReconShape&,
//                      const PdhgParams&, int32_t* f3, PdhgWork&, PdhgOut[3])
//
// 编译方式
// ----------------------------------------------------------------------------
// 主机（语法检查，pragma 不生效；**不定义** __SYNTHESIS__）：
//     g++ -std=c++17 -Wall -Wextra -Wno-unknown-pragmas
//         -Isrc/hardware -Isrc/software/include -Isrc/fpga/hls
//         src/fpga/hls/ps_fixed_top.cpp
//         <三个 *_fixed.cpp> <smoke.cpp> -o <exe>
// HLS（pragma 生效）：
//     由 build_hls.tcl 加 -D__SYNTHESIS__ 编译（工具自己也会定义，
//     见 ps_fixed.hpp 文首的说明）。
//
// depth 的口径
// ----------------------------------------------------------------------------
//   depth 是按被包装核的**名义 B 档尺寸**给的接口上界（不是运行期约束），
//   口径见 docs/hls_interface_spec.md §二：「给的是上界，给小了功能错」。
//   B 档（configs/geometry_b.yaml + src/software/include/psm_format.h）：
//     M = 9120（95×96 竖直差分测量）、N = 432（24×18 隐藏图）、
//     P = 243（候选 9×9×3）、定位秩 K_loc = 8、重建秩 R = 64、
//     PS_LR_MAX_RANK = 96（核允许的秩上界，见 lr_operator.hpp）
//
//   ⚠ 2026-09-28 更正：本层的 depth 原先**普遍偏小**（从 ps_axi_top.cpp 抄来的
//     一组占位值），而偏小的 depth 会让 Vitis 推断出错误的突发边界 ——
//     按接口规格的原文「给小了功能错」，这是**静默错数据**的风险，不是性能问题。
//     实测对照（左为更正前）：
//       meas      3M = 27,360        12288   -> 27360
//       u_codes   P·K·M = 17,729,280 1048576 -> 17729280
//       U         K·M = 583,680      8192    -> 583680
//       S         K ≤ 96             64      -> 96
//       V         K·N = 27,648       8192    -> 27648
//       d3        3M = 27,360        16384   -> 27360
//     其余端口（f/g/f3/score_*/gain_q16/wk_*）原值已 >= B 档，未动。
//
//   ⚠ 两层不再取同一组数：ps_axi_top.cpp 的 `model_base` 是「整个模型体」的基址
//     （depth 语义不同，需按 A6 规格单独核），本层的 `u_codes` 是**紧凑 rank-K**
//     数组。两者的正确 depth 本来就不同，原先「取同一组数便于对照」的做法是错的。
//
//   ⚠ u_codes 的布局是 `u_codes[p*K*M + k*M + m]`（紧凑，逐候选 stride = K·M），
//     而 model.psm 里每个候选记录的 U 是 `rank_max × meas_len`（= 64×9120，
//     见 psm_format.h 的 ps_psm_read_U）。**两者不一致** ——
//     PS 侧在把模型交给本 IP 之前必须先重排成 rank-8 紧凑数组
//     （243×8×9120×2 B ≈ 35.5 MB，一次性）。此项尚未在任何文档里落地，
//     已记入 sim/results/x19_hls_resources_and_timing.md 的待决表。
// ============================================================================

#include "ps_fixed_top.hpp"

#include "projection_engine/projection_engine.hpp"
#include "lr_operator/lr_operator.hpp"
#include "pdhg_core/pdhg_core.hpp"

// ============================================================================
// 1) 定位：候选打分与排序
//    包装 ps::projection_engine<PS_FIX_LANES>
// ============================================================================
extern "C" int ps_projection_fix(const int32_t* meas,
                                 const int16_t* u_codes,
                                 const int32_t* gain_q16,
                                 int P, int K, int M,
                                 int nx, int ny, int nz,
                                 int margin_thresh_q16,
                                 uint64_t* score_out,
                                 uint64_t* score_ch,
                                 int* best_id, int* second_id, int* margin_q16,
                                 int* boundary, int* low_conf, int* overflow,
                                 unsigned long long* best_score,
                                 unsigned long long* second_score,
                                 int* sat_events, int* mac_overflow,
                                 int* out_overflow, int* score_overflow,
                                 int* gain_clamped, long long* max_abs_acc) {
#ifdef __SYNTHESIS__
#pragma HLS INTERFACE m_axi port=meas      offset=slave bundle=gmem1_data  depth=27360
#pragma HLS INTERFACE m_axi port=u_codes   offset=slave bundle=gmem0_model depth=17729280
#pragma HLS INTERFACE m_axi port=gain_q16  offset=slave bundle=gmem0_model depth=1024
#pragma HLS INTERFACE m_axi port=score_out offset=slave bundle=gmem1_data  depth=1024
#pragma HLS INTERFACE m_axi port=score_ch  offset=slave bundle=gmem1_data  depth=3072
#pragma HLS INTERFACE s_axilite port=P bundle=control
#pragma HLS INTERFACE s_axilite port=K bundle=control
#pragma HLS INTERFACE s_axilite port=M bundle=control
#pragma HLS INTERFACE s_axilite port=nx bundle=control
#pragma HLS INTERFACE s_axilite port=ny bundle=control
#pragma HLS INTERFACE s_axilite port=nz bundle=control
#pragma HLS INTERFACE s_axilite port=margin_thresh_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=best_id bundle=control
#pragma HLS INTERFACE s_axilite port=second_id bundle=control
#pragma HLS INTERFACE s_axilite port=margin_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=boundary bundle=control
#pragma HLS INTERFACE s_axilite port=low_conf bundle=control
#pragma HLS INTERFACE s_axilite port=overflow bundle=control
#pragma HLS INTERFACE s_axilite port=best_score bundle=control
#pragma HLS INTERFACE s_axilite port=second_score bundle=control
#pragma HLS INTERFACE s_axilite port=sat_events bundle=control
#pragma HLS INTERFACE s_axilite port=mac_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=out_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=score_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=gain_clamped bundle=control
#pragma HLS INTERFACE s_axilite port=max_abs_acc bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#endif
    // ---------------------------------------------------------------- 标定档
    // 只在 PS_HLS_TOP_MODE=cal 时才定义 PS_FIX_CAL_B（见 build_hls.tcl 文首）。
    // 目的：生产档的 P/K/M/nx/ny/nz 是**运行期 s_axilite 寄存器参数**，HLS 无法
    // 静态定界循环，csynth 报的 Latency 是 2^28 次 maxbound 外推的噪声（X19 报告 §五）。
    // 这里把 B 档尺寸**赋成编译期常量**，循环就能定界，报告里的周期数才是真值。
    // 注意：**签名与 AXI 寄存器映射一个字都没动**，只是标定档下不再读那几个寄存器。
    // 数值出处：configs/geometry_b.yaml —— 候选 9x9x3 = 243、定位秩 K = 8、M = 9120。
    // 上了板的固件永远不定义这个宏，生产档行为不受影响。
#ifdef PS_FIX_CAL_B
    P = 243; K = 8; M = 9120; nx = 9; ny = 9; nz = 3;
#endif
    // ---- 入参检查（非法时返回被包装核的错误码）----
    if (!meas || !u_codes || !gain_q16 || !score_out) return ps::kProjBadArgs;
    if (!best_id || !second_id || !margin_q16 || !boundary || !low_conf
        || !overflow || !best_score || !second_score) return ps::kProjBadArgs;
    if (!sat_events || !mac_overflow || !out_overflow || !score_overflow
        || !gain_clamped || !max_abs_acc) return ps::kProjBadArgs;
    if (P <= 0 || K <= 0 || M <= 0) return ps::kProjBadArgs;
    if (K > PS_LR_MAX_RANK) return ps::kProjBadArgs;
    // nx / ny / nz <= 0 **不判非法**：核把 <=0 定义为「无网格信息」的合法模式
    // （proj_grid_index 返回 -1、boundary 记 0，见 projection_engine.hpp），
    // 顶层不额外拦截，以免与核语义不一致。

    ps::ProjResult res;
    ps::FixFlags flags;
    fix_flags_reset(flags);         // 核只累加 flags，初值由调用方给（不回绕）

    const int rc = ps::projection_engine<PS_FIX_LANES>(
        meas, u_codes, gain_q16, P, K, M, nx, ny, nz, margin_thresh_q16,
        score_out, score_ch, res, flags);

    // ---- ProjResult 逐字段回写 ----
    *best_id      = res.best_id;            // res.best_id      -> best_id
    *second_id    = res.second_id;          // res.second_id    -> second_id
    *margin_q16   = res.margin_q16;         // res.margin_q16   -> margin_q16
    *boundary     = res.boundary;           // res.boundary     -> boundary
    *low_conf     = res.low_conf;           // res.low_conf     -> low_conf
    *overflow     = res.overflow;           // res.overflow     -> overflow
    *best_score   = (unsigned long long)res.best_score;    // res.best_score
    *second_score = (unsigned long long)res.second_score;  // res.second_score

    // ---- FixFlags 逐字段回写（6 个字段全部回写）----
    *sat_events     = flags.sat_events;     // flags.sat_events     -> sat_events
    *mac_overflow   = flags.mac_overflow;   // flags.mac_overflow   -> mac_overflow
    *out_overflow   = flags.out_overflow;   // flags.out_overflow   -> out_overflow
    *score_overflow = flags.score_overflow; // flags.score_overflow -> score_overflow
    *gain_clamped   = flags.gain_clamped;   // flags.gain_clamped   -> gain_clamped
    *max_abs_acc    = flags.max_abs_acc;    // flags.max_abs_acc    -> max_abs_acc
    return rc;
}

// ============================================================================
// 2) 低秩前向  y = U · diag(S) · (V f)
//    包装 ps::lr_forward<PS_FIX_LANES>
// ============================================================================
extern "C" int ps_lr_forward_fix(const int16_t* U,
                                 const uint16_t* S,
                                 const int16_t* V,
                                 int K, int M, int N,
                                 int32_t u_gain_q16, int32_t s_gain_q16,
                                 int32_t v_gain_q16,
                                 int u_exp, int s_exp, int v_exp,
                                 const int32_t* f,
                                 int32_t* y,
                                 int32_t* wk_z,
                                 int32_t* wk_zs,
                                 long long* mac_count,
                                 int* sat_events, int* mac_overflow,
                                 int* out_overflow, int* score_overflow,
                                 int* gain_clamped, long long* max_abs_acc) {
#ifdef __SYNTHESIS__
#pragma HLS INTERFACE m_axi port=U    offset=slave bundle=gmem0_model   depth=583680
#pragma HLS INTERFACE m_axi port=S    offset=slave bundle=gmem0_model   depth=96
#pragma HLS INTERFACE m_axi port=V    offset=slave bundle=gmem0_model   depth=27648
#pragma HLS INTERFACE m_axi port=f    offset=slave bundle=gmem1_data    depth=1024
#pragma HLS INTERFACE m_axi port=y    offset=slave bundle=gmem1_data    depth=16384
#pragma HLS INTERFACE m_axi port=wk_z offset=slave bundle=gmem2_scratch depth=96
#pragma HLS INTERFACE m_axi port=wk_zs offset=slave bundle=gmem2_scratch depth=96
#pragma HLS INTERFACE s_axilite port=K bundle=control
#pragma HLS INTERFACE s_axilite port=M bundle=control
#pragma HLS INTERFACE s_axilite port=N bundle=control
#pragma HLS INTERFACE s_axilite port=u_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=s_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=v_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=u_exp bundle=control
#pragma HLS INTERFACE s_axilite port=s_exp bundle=control
#pragma HLS INTERFACE s_axilite port=v_exp bundle=control
#pragma HLS INTERFACE s_axilite port=mac_count bundle=control
#pragma HLS INTERFACE s_axilite port=sat_events bundle=control
#pragma HLS INTERFACE s_axilite port=mac_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=out_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=score_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=gain_clamped bundle=control
#pragma HLS INTERFACE s_axilite port=max_abs_acc bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#endif
    // 标定档（PS_FIX_CAL_B）：把 B 档尺寸钉成编译期常量，理由见 ps_projection_fix 处。
    // 重建口径：秩 R = 64（configs/geometry_b.yaml recon_rank_default）、N = 24x18 = 432。
#ifdef PS_FIX_CAL_B
    K = 64; M = 9120; N = 432;
#endif
    // ---- 入参检查 ----
    if (!U || !S || !V || !f || !y) return ps::kLrBadArgs;
    if (!mac_count || !sat_events || !mac_overflow || !out_overflow
        || !score_overflow || !gain_clamped || !max_abs_acc) return ps::kLrBadArgs;
    if (K <= 0 || M <= 0 || N <= 0) return ps::kLrBadArgs;
    if (K > PS_LR_MAX_RANK) return ps::kLrBadArgs;

    // ---- 拼装 LrFactors（逐字段，与 lr_operator.hpp 的字段顺序一致）----
    ps::LrFactors F;
    F.U = U;
    F.S = S;
    F.V = V;
    F.K = K;
    F.M = M;
    F.N = N;
    F.u_gain_q16 = u_gain_q16;
    F.s_gain_q16 = s_gain_q16;
    F.v_gain_q16 = v_gain_q16;
    F.u_exp = u_exp;
    F.s_exp = s_exp;
    F.v_exp = v_exp;

    // ---- 观测缓冲：wk_* 传 nullptr 即不观测 ----
    // 两个观测缓冲都为 nullptr 时按约定把 LrWork* 传 nullptr（板上去掉分支）；
    // 只给其中一个时也正确 —— 核按字段各自判空。
    //
    // ⚠ 综合视图里**整体去掉**：ps::LrWork 的成员是指针，本地建一个 LrWork、
    //    取地址交给核，就是「指针的指针」，HLS 在 Syn check 阶段直接拒绝
    //    （HLS 214-134，2026-09-28 实测）。该通道属于主机侧验证设施
    //    （test_fixed_top.cpp / test_lr_*.cpp 的中间量对照），板上无用途，
    //    因此综合时恒传 nullptr，等价于「不观测」。
#ifndef __SYNTHESIS__
    ps::LrWork W;
    W.z  = wk_z;
    W.zs = wk_zs;
    W.w  = 0;                       // 前向不使用 w / ws
    W.ws = 0;
    ps::LrWork* Wp = (wk_z != 0 || wk_zs != 0) ? &W : 0;
#else
    ps::LrWork* Wp = 0;
#endif

    ps::LrDiag dg;                 // 核内部会 fix_flags_reset(dg.flags)，无需预置
    const int rc = ps::lr_forward<PS_FIX_LANES>(F, f, y, dg, Wp);

    // ---- LrDiag / FixFlags 逐字段回写 ----
    *mac_count      = dg.mac_count;         // dg.mac_count          -> mac_count
    *sat_events     = dg.flags.sat_events;  // dg.flags.*            -> 6 个标量
    *mac_overflow   = dg.flags.mac_overflow;
    *out_overflow   = dg.flags.out_overflow;
    *score_overflow = dg.flags.score_overflow;
    *gain_clamped   = dg.flags.gain_clamped;
    *max_abs_acc    = dg.flags.max_abs_acc;
    return rc;
}

// ============================================================================
// 3) 低秩伴随  g = Vᵀ · diag(S) · (Uᵀ r)
//    包装 ps::lr_adjoint<PS_FIX_LANES>
// ============================================================================
extern "C" int ps_lr_adjoint_fix(const int16_t* U,
                                 const uint16_t* S,
                                 const int16_t* V,
                                 int K, int M, int N,
                                 int32_t u_gain_q16, int32_t s_gain_q16,
                                 int32_t v_gain_q16,
                                 int u_exp, int s_exp, int v_exp,
                                 const int32_t* r,
                                 int32_t* g,
                                 int32_t* wk_w,
                                 int32_t* wk_ws,
                                 long long* mac_count,
                                 int* sat_events, int* mac_overflow,
                                 int* out_overflow, int* score_overflow,
                                 int* gain_clamped, long long* max_abs_acc) {
#ifdef __SYNTHESIS__
#pragma HLS INTERFACE m_axi port=U     offset=slave bundle=gmem0_model   depth=583680
#pragma HLS INTERFACE m_axi port=S     offset=slave bundle=gmem0_model   depth=96
#pragma HLS INTERFACE m_axi port=V     offset=slave bundle=gmem0_model   depth=27648
#pragma HLS INTERFACE m_axi port=r     offset=slave bundle=gmem1_data    depth=16384
#pragma HLS INTERFACE m_axi port=g     offset=slave bundle=gmem1_data    depth=1024
#pragma HLS INTERFACE m_axi port=wk_w  offset=slave bundle=gmem2_scratch depth=96
#pragma HLS INTERFACE m_axi port=wk_ws offset=slave bundle=gmem2_scratch depth=96
#pragma HLS INTERFACE s_axilite port=K bundle=control
#pragma HLS INTERFACE s_axilite port=M bundle=control
#pragma HLS INTERFACE s_axilite port=N bundle=control
#pragma HLS INTERFACE s_axilite port=u_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=s_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=v_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=u_exp bundle=control
#pragma HLS INTERFACE s_axilite port=s_exp bundle=control
#pragma HLS INTERFACE s_axilite port=v_exp bundle=control
#pragma HLS INTERFACE s_axilite port=mac_count bundle=control
#pragma HLS INTERFACE s_axilite port=sat_events bundle=control
#pragma HLS INTERFACE s_axilite port=mac_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=out_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=score_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=gain_clamped bundle=control
#pragma HLS INTERFACE s_axilite port=max_abs_acc bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#endif
    // 标定档（PS_FIX_CAL_B）：与 ps_lr_forward_fix 同一组常量。
#ifdef PS_FIX_CAL_B
    K = 64; M = 9120; N = 432;
#endif
    if (!U || !S || !V || !r || !g) return ps::kLrBadArgs;
    if (!mac_count || !sat_events || !mac_overflow || !out_overflow
        || !score_overflow || !gain_clamped || !max_abs_acc) return ps::kLrBadArgs;
    if (K <= 0 || M <= 0 || N <= 0) return ps::kLrBadArgs;
    if (K > PS_LR_MAX_RANK) return ps::kLrBadArgs;

    ps::LrFactors F;
    F.U = U;
    F.S = S;
    F.V = V;
    F.K = K;
    F.M = M;
    F.N = N;
    F.u_gain_q16 = u_gain_q16;
    F.s_gain_q16 = s_gain_q16;
    F.v_gain_q16 = v_gain_q16;
    F.u_exp = u_exp;
    F.s_exp = s_exp;
    F.v_exp = v_exp;

    // 观测缓冲：同前向，综合视图里恒传 nullptr（HLS 214-134，见 ps_lr_forward_fix 处说明）。
#ifndef __SYNTHESIS__
    ps::LrWork W;
    W.z  = 0;                       // 伴随不使用 z / zs
    W.zs = 0;
    W.w  = wk_w;
    W.ws = wk_ws;
    ps::LrWork* Wp = (wk_w != 0 || wk_ws != 0) ? &W : 0;
#else
    ps::LrWork* Wp = 0;
#endif

    ps::LrDiag dg;
    const int rc = ps::lr_adjoint<PS_FIX_LANES>(F, r, g, dg, Wp);

    *mac_count      = dg.mac_count;
    *sat_events     = dg.flags.sat_events;
    *mac_overflow   = dg.flags.mac_overflow;
    *out_overflow   = dg.flags.out_overflow;
    *score_overflow = dg.flags.score_overflow;
    *gain_clamped   = dg.flags.gain_clamped;
    *max_abs_acc    = dg.flags.max_abs_acc;
    return rc;
}

// ============================================================================
// 4) 重建：三通道 PDHG
//    包装 ps::pdhg_core<PS_FIX_LANES>
// ============================================================================
extern "C" int ps_pdhg_fix(const int16_t* U,
                           const uint16_t* S,
                           const int16_t* V,
                           int K, int M, int N,
                           int32_t u_gain_q16, int32_t s_gain_q16,
                           int32_t v_gain_q16,
                           int u_exp, int s_exp, int v_exp,
                           const int32_t* d3,
                           int n_rows, int n_cols,
                           int n_iter, int min_iters, int residual_every,
                           int tv_mode,
                           int32_t tau_q16, int32_t sigma_q16,
                           int32_t lambda_q16, int32_t theta_q16,
                           int32_t rho_stop_q8,
                           int32_t* f3,
                           int32_t* wk_f_prev, int32_t* wk_fbar, int32_t* wk_q,
                           int32_t* wk_grad, int32_t* wk_divq, int32_t* wk_y,
                           int32_t* wk_r, int32_t* wk_btr, int32_t* wk_tmp,
                           int out_n_used[3], int out_early_stopped[3],
                           int out_overflow_ch[3],
                           unsigned int out_rho_q16[3],
                           unsigned long long out_res_sq[3],
                           unsigned long long out_d_sq[3],
                           long long out_mac_count[3],
                           int* sat_events, int* mac_overflow,
                           int* out_overflow, int* score_overflow,
                           int* gain_clamped, long long* max_abs_acc) {
#ifdef __SYNTHESIS__
#pragma HLS INTERFACE m_axi port=U      offset=slave bundle=gmem0_model   depth=583680
#pragma HLS INTERFACE m_axi port=S      offset=slave bundle=gmem0_model   depth=96
#pragma HLS INTERFACE m_axi port=V      offset=slave bundle=gmem0_model   depth=27648
#pragma HLS INTERFACE m_axi port=d3     offset=slave bundle=gmem1_data    depth=27360
#pragma HLS INTERFACE m_axi port=f3     offset=slave bundle=gmem1_data    depth=3072
// 9 个迭代暂存走独立 bundle，避免与模型抢 gmem0 的带宽。长度由 PdhgWork 语义钉死：
// f_prev/fbar/divq/btr/tmp = N；q/grad = 2N；y/r = M。
#pragma HLS INTERFACE m_axi port=wk_f_prev offset=slave bundle=gmem2_scratch depth=1024
#pragma HLS INTERFACE m_axi port=wk_fbar   offset=slave bundle=gmem2_scratch depth=1024
#pragma HLS INTERFACE m_axi port=wk_q      offset=slave bundle=gmem2_scratch depth=2048
#pragma HLS INTERFACE m_axi port=wk_grad   offset=slave bundle=gmem2_scratch depth=2048
#pragma HLS INTERFACE m_axi port=wk_divq   offset=slave bundle=gmem2_scratch depth=1024
#pragma HLS INTERFACE m_axi port=wk_y      offset=slave bundle=gmem2_scratch depth=16384
#pragma HLS INTERFACE m_axi port=wk_r      offset=slave bundle=gmem2_scratch depth=16384
#pragma HLS INTERFACE m_axi port=wk_btr    offset=slave bundle=gmem2_scratch depth=1024
#pragma HLS INTERFACE m_axi port=wk_tmp    offset=slave bundle=gmem2_scratch depth=1024
// 每通道结果数组（3 元）走 gmem1_data；标量走 control。
#pragma HLS INTERFACE m_axi port=out_n_used        offset=slave bundle=gmem1_data depth=3
#pragma HLS INTERFACE m_axi port=out_early_stopped offset=slave bundle=gmem1_data depth=3
#pragma HLS INTERFACE m_axi port=out_overflow_ch   offset=slave bundle=gmem1_data depth=3
#pragma HLS INTERFACE m_axi port=out_rho_q16       offset=slave bundle=gmem1_data depth=3
#pragma HLS INTERFACE m_axi port=out_res_sq        offset=slave bundle=gmem1_data depth=3
#pragma HLS INTERFACE m_axi port=out_d_sq          offset=slave bundle=gmem1_data depth=3
#pragma HLS INTERFACE m_axi port=out_mac_count     offset=slave bundle=gmem1_data depth=3
#pragma HLS INTERFACE s_axilite port=K bundle=control
#pragma HLS INTERFACE s_axilite port=M bundle=control
#pragma HLS INTERFACE s_axilite port=N bundle=control
#pragma HLS INTERFACE s_axilite port=u_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=s_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=v_gain_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=u_exp bundle=control
#pragma HLS INTERFACE s_axilite port=s_exp bundle=control
#pragma HLS INTERFACE s_axilite port=v_exp bundle=control
#pragma HLS INTERFACE s_axilite port=n_rows bundle=control
#pragma HLS INTERFACE s_axilite port=n_cols bundle=control
#pragma HLS INTERFACE s_axilite port=n_iter bundle=control
#pragma HLS INTERFACE s_axilite port=min_iters bundle=control
#pragma HLS INTERFACE s_axilite port=residual_every bundle=control
#pragma HLS INTERFACE s_axilite port=tv_mode bundle=control
#pragma HLS INTERFACE s_axilite port=tau_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=sigma_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=lambda_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=theta_q16 bundle=control
#pragma HLS INTERFACE s_axilite port=rho_stop_q8 bundle=control
#pragma HLS INTERFACE s_axilite port=sat_events bundle=control
#pragma HLS INTERFACE s_axilite port=mac_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=out_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=score_overflow bundle=control
#pragma HLS INTERFACE s_axilite port=gain_clamped bundle=control
#pragma HLS INTERFACE s_axilite port=max_abs_acc bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#endif
    // 标定档（PS_FIX_CAL_B）：把 B 档尺寸与迭代数一并钉成编译期常量 ——
    // n_iter 是**外层迭代循环的上界**，不钉住它就报不出 PDHG 的真实耗时。
    // 数值出处：configs/geometry_b.yaml ranks（recon_rank_default 64、
    // recon_iter_default 16）与 §5.1（N = 24x18，n_rows 18 / n_cols 24）；
    // min_iters 8、residual_every 4 见方案 §6.5。
#ifdef PS_FIX_CAL_B
    // ---- 标定分级：PS_FIX_CAL_LEVEL ----
    // 用途：把「Performance (step 3) 指令数暴涨导致预综合失败」这件事变成可控的二分。
    // 由 build_hls.tcl 从环境变量 PS_HLS_CAL_LEVEL 传进来，默认 3（与既有行为一致）。
    //   1 = 只钉尺寸三件 K/M/N；n_rows/n_cols 与迭代常量留给运行期
    //   2 = 只钉迭代常量；全部尺寸留给运行期
    //   3 = 全钉（默认，既有行为）
    //   4 = 只钉 N 与 n_rows/n_cols（把 K/M 留给运行期）
    //   5 = 只钉 K（其余留给运行期）
    // 判读方式：跑一轮看是否 `Pre-synthesis failed`，失败快（约 25 秒），
    //   所以这是一个便宜的实验，不必猜。
    // 对照事实（决定了本实验从哪一侧切）：`ps_lr_forward_fix` 的标定档**同样**用
    //   K = 64 / M = 9120 / N = 432，而它综合成功（6.37 ms）。所以这三个常量本身
    //   不是元凶；pdhg 独有的是 n_rows/n_cols（TV 的二维 shape）与早停路径。
#ifndef PS_FIX_CAL_LEVEL
#define PS_FIX_CAL_LEVEL 3
#endif
#if (PS_FIX_CAL_LEVEL == 1) || (PS_FIX_CAL_LEVEL == 3)
    K = 64; M = 9120; N = 432; n_rows = 18; n_cols = 24;
#elif PS_FIX_CAL_LEVEL == 4
    N = 432; n_rows = 18; n_cols = 24;
#elif PS_FIX_CAL_LEVEL == 5
    K = 64;
#elif PS_FIX_CAL_LEVEL == 6
    N = 432;
#elif PS_FIX_CAL_LEVEL == 7
    M = 9120;
#elif PS_FIX_CAL_LEVEL == 9
    n_rows = 18; n_cols = 24;
#elif PS_FIX_CAL_LEVEL == 10
    K = 64; N = 432;
#elif PS_FIX_CAL_LEVEL == 11
    // 11 = 钉 K/M/N 三件尺寸（**不钉 n_rows/n_cols**），用于 X19 的 pdhg 总延迟标定。
    // n_rows/n_cols 一旦钉住，TV 二维循环会让 Performance (step 3) 的指令数越过阈值
    // （见 sim/results/x19b_pdhg_cal_bisect.md §2），故这里保持运行期；它们的循环界
    // 改由 pdhg_core.hpp / pdhg_core_fixed.cpp 的 HLS LOOP_TRIPCOUNT 提供
    // （只进报告，不参与 RTL 生成）。
    K = 64; M = 9120; N = 432;
#endif
#if (PS_FIX_CAL_LEVEL == 2) || (PS_FIX_CAL_LEVEL == 3)
    // ★ 这两行踩过一条被证否的结论，必须留住更正记录：
    //   最初把 n_iter 钉成 16 时，Vitis 2025.2 的 HLS 前端崩溃
    //   （`clang frontend command failed due to signal`，崩在 Performance (step 3)，
    //    指令数 7,720 -> 22,964），于是把 n_iter 压到 1，并记成「取 1 就能跑」。
    //   **2026-09-29 复测：n_iter = 1 仍然失败**（`Pre-synthesis failed`，
    //   指令数 7,713 -> 22,922）。所以那个说法不成立，元凶不在 n_iter。
    //   那次失败的峰值内存只有 146.8 MB，与内存无关。
    //   此处保留 n_iter = 1，只为让分级实验能把迭代常量集团干净地排除掉。
    n_iter = 1; min_iters = 1; residual_every = 1;   // 早停判据在 1 次迭代下不会触发
#elif PS_FIX_CAL_LEVEL == 11
    // B 档真实迭代参数（configs/geometry_b.yaml recon_iter_default = 16；
    // 方案 §6.5：min_iters = 8、residual_every = 4）。n_iter 仍留运行期 ——
    // 钉住它会让外层迭代循环被展开，其界改用 LOOP_TRIPCOUNT 给（见 pdhg_core_fixed.cpp）。
    min_iters = 8; residual_every = 4;
#endif
#endif
    // ---- 入参检查 ----
    if (!U || !S || !V || !d3 || !f3) return ps::kPdhgBadArgs;
    if (!wk_f_prev || !wk_fbar || !wk_q || !wk_grad || !wk_divq
        || !wk_y || !wk_r || !wk_btr || !wk_tmp) return ps::kPdhgBadArgs;
    if (!out_n_used || !out_early_stopped || !out_overflow_ch || !out_rho_q16
        || !out_res_sq || !out_d_sq || !out_mac_count) return ps::kPdhgBadArgs;
    if (!sat_events || !mac_overflow || !out_overflow || !score_overflow
        || !gain_clamped || !max_abs_acc) return ps::kPdhgBadArgs;
    if (K <= 0 || M <= 0 || N <= 0 || n_rows <= 0 || n_cols <= 0) {
        return ps::kPdhgBadArgs;
    }
    if (K > PS_LR_MAX_RANK) return ps::kPdhgBadArgs;
    // n_rows*n_cols != N 由 ps::pdhg_solve_channel 内部校验（返回 kPdhgBadArgs），
    // 这里不重复拦截，保持与核同一份判据。

    // ---- 拼装 LrFactors（与两个 lr 顶层同一构造方式）----
    ps::LrFactors F;
    F.U = U;
    F.S = S;
    F.V = V;
    F.K = K;
    F.M = M;
    F.N = N;
    F.u_gain_q16 = u_gain_q16;
    F.s_gain_q16 = s_gain_q16;
    F.v_gain_q16 = v_gain_q16;
    F.u_exp = u_exp;
    F.s_exp = s_exp;
    F.v_exp = v_exp;

    ps::ReconShape shape;
    shape.n_rows = n_rows;
    shape.n_cols = n_cols;

    ps::PdhgParams P;
    P.n_iter         = n_iter;
    P.min_iters      = min_iters;
    P.residual_every = residual_every;
    P.tv_mode        = tv_mode;
    P.tau_q16        = tau_q16;
    P.sigma_q16      = sigma_q16;
    P.lambda_q16     = lambda_q16;
    P.theta_q16      = theta_q16;
    P.rho_stop_q8    = rho_stop_q8;

    ps::PdhgWork W;
    W.f_prev = wk_f_prev;
    W.fbar   = wk_fbar;
    W.q      = wk_q;
    W.grad   = wk_grad;
    W.divq   = wk_divq;
    W.y      = wk_y;
    W.r      = wk_r;
    W.btr    = wk_btr;
    W.tmp    = wk_tmp;

    ps::PdhgOut out3[3];
    const int rc = ps::pdhg_core<PS_FIX_LANES>(F, d3, shape, P, f3, W, out3);

    // ---- PdhgOut 逐通道、逐字段回写 ----
    for (int c = 0; c < 3; ++c) {
        out_n_used[c]        = out3[c].n_used;         // .n_used        -> out_n_used[3]
        out_early_stopped[c] = out3[c].early_stopped;  // .early_stopped -> out_early_stopped[3]
        out_overflow_ch[c]   = out3[c].overflow;       // .overflow      -> out_overflow_ch[3]
        out_rho_q16[c]       = (unsigned int)out3[c].rho_q16;        // .rho_q16
        out_res_sq[c]        = (unsigned long long)out3[c].res_sq;   // .res_sq
        out_d_sq[c]          = (unsigned long long)out3[c].d_sq;     // .d_sq
        out_mac_count[c]     = out3[c].mac_count;      // .mac_count     -> out_mac_count[3]
    }

    // ---- 顶层 FixFlags 逐字段回写 ----
    // 注意：pdhg_core 的签名**不输出内部 FixFlags**，它只经 PdhgOut.overflow 暴露
    // 「该通道是否发生过饱和」。因此这 6 个标量由顶层按值构造的 FixFlags 回写，
    // 本层不做数值加工，故恒为 0；每通道的溢出信息请读 out_overflow_ch[3]。
    // （PdhgOut.rho_q8 未在接口中导出，只导出上报口径的 rho_q16。）
    ps::FixFlags flags;
    fix_flags_reset(flags);
    *sat_events     = flags.sat_events;
    *mac_overflow   = flags.mac_overflow;
    *out_overflow   = flags.out_overflow;
    *score_overflow = flags.score_overflow;
    *gain_clamped   = flags.gain_clamped;
    *max_abs_acc    = flags.max_abs_acc;
    return rc;
}
