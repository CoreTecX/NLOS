// ============================================================================
// ps_axi_top.cpp —— HLS 顶层 AXI 接口层的实现（A6）
//
// 这一层只做一件事：**平坦入参 → 结构体 → 转调原顶层壳**。
// 数学一行都不重写，所以它不可能引入数值差异 —— 这是刻意的：
// 接口层的 bug 只应该表现为「连错线」，不应该表现为「结果不对」。
//
// 五个核的顶层壳（原样调用，不改）：
//     ps::projection_engine_top(meas3, model_base, h, P, K, score_out, score_ch, res)
//     ps::lr_forward_top(F, f, y)
//     ps::lr_adjoint_top(F, r, g)
//     ps::pdhg_core_top(F, d, shape, P, work, out, res_hist, res_hist_cap)
//
// 编译方式
// ----------------------------------------------------------------------------
// 主机（语法检查，pragma 不生效）：
//     g++ -std=c++17 -c ps_axi_top.cpp -Isrc/software/include -I.
// HLS（pragma 生效）：
//     由 src/fpga/hls/scripts/build_hls.tcl 加 -D__SYNTHESIS__ 编译
//     （HLS 工具自己会定义 __SYNTHESIS__，见 ps_fixed.hpp 文首的说明）
// ============================================================================

#include "ps_axi_top.h"

#include "../../software/include/psm_format.h"

#include "lr_operator/lr_operator.h"
#include "projection_engine/projection_engine.h"
#include "pdhg_core/pdhg_core.h"

// ============================================================================
// depth 的口径（2026-10-02 更正）
// ----------------------------------------------------------------------------
//  本层原先的 depth 是从更早的草稿抄来的一组占位值（U/V 8192、S 64、
//  meas 12288、model_base 1048576），与 B 档真实尺寸不符。而 depth 是
//  **接口上界**：给小了会让 Vitis 推断出错误的突发边界，属**静默错数据**，
//  不是性能问题（口径见 docs/hls_interface_spec.md §二）。
//
//  本次按 B 档真实尺寸逐项对齐，出处与 ps_fixed_top.cpp（其 depth 于
//  2026-09-28 更正）同源 —— psm_format.h:49-58 与 configs/geometry_b.yaml：
//      M(meas) = 95x96      = 9120      N(hidden) = 18x24 = 432
//      P(候选) = 9x9x3      = 243       K_loc = 8   R = 64
//      本层 K 上界 = PS_RANK_MAX = 64（见本文件 :"#define PS_RANK_MAX" 处）
//
//  逐项对照（元素数；左为更正前，右为更正后）：
//      U     K·M   = 583,680    8192      -> 583680
//      S     K     = 64         64        -> 64（本层 K 上界 64；定点层核允许
//                                            96，故那边写 96，两者各按自己的
//                                            K 上界取值，不强行取同一数 —— 同
//                                             ps_fixed_top.cpp:51-53 的原则）
//      V     K·N   = 27,648     8192      -> 27648
//      meas3 3·M   = 27,360     12288     -> 27360
//      model_base  = 300,589,120（整个 model.psm 体，按字节）  1048576 -> 300589120
//      h     = 64（sizeof(ps_psm_header_t)，psm_format.h:46）    16 -> 64
//      d     3·M   = 27,360     16384     -> 27360
//      f/y/r/g/score_*/暂存：原值已 >= B 档，未动（同 ps_fixed_top.cpp:49 的处理）
//
//  model_base 为什么是「整个模型体」而不是紧凑 rank-K 数组：
//    本层包的是双精度层 projection_engine_top，它按 psm 原生布局
//    `model_base + ps_psm_candidate_offset(h,p)` 逐候选读
//    （projection_engine.cpp:65-66），单条记录 = ps_psm_record_bytes(64,9120,432)
//    = 1,236,992 B（psm_format.h:158-163），243 条 + 64 B 头 = 300,589,120 B。
//    定点层的 u_codes = P·K·M = 17,729,280 是**紧凑 rank-8** 数组，语义不同，
//    两者不能取同一个数（同 ps_fixed_top.cpp:51-53 的更正）。
//
//  本层当前状态：`build_hls.tcl` 默认综合 shell（双精度层），块设计用的是
//  定点层四个 IP（penumbrasight_bd.tcl 的 CORE_MAP）。AXI 档**不是上板路径**
//  （DSP48E2 27x18 做不了双精度浮点，见 docs/hls_interface_spec.md §6.1
//  2026-09-29 定案），但 `PS_HLS_TOP_MODE=axi` 仍可构建，故 depth 仍须正确。
//  这一档的用途是「接口契约 + 可编译的包装」；若要彻底停用，见
//  docs/hls_interface_spec.md §6.1 的定案记录。
// ============================================================================

// 当前包的是双精度层。切到定点层时，这里换成 *_fixed.cpp 的入口，
// 并把 PS_AXI_SAMPLE_T / PS_AXI_COEF_T 改成 int32_t / int16_t。
typedef ps::LrFactorsFp   ps_axi_factors_t;
typedef ps::ProjScoreFp   ps_axi_proj_res_t;

// 秩上限。**必须与 src/software/include/psm_format.h:56 的
// PS_PSM_RANK_MAX 一致**（模型格式的硬约束，改了模型就读不出来）。
// 这里用本地名而不是直接引那个宏，是为了让接口层不强依赖 psm 格式头的
// 具体宏名；一致性由本行的数值 64 与 psm_format.h 同时锚定。
#define PS_RANK_MAX  64

// 四个顶层壳**只在各自的 .cpp 里定义，任何头文件都没有声明**
// （它们原本只给同目录的 tb.cpp 用，而 tb 与 .cpp 是一起编译的，
//   所以不需要声明）。本层是独立翻译单元，故在此显式声明。
// 签名逐字抄自 lr_operator.cpp:166/170、projection_engine.cpp:180、
// pdhg_core.cpp:210 —— **改动那三处时必须同步改这里**。
namespace ps {
int lr_forward_top(const LrFactorsFp& F, const double* f, double* y);
int lr_adjoint_top(const LrFactorsFp& F, const double* r, double* g);
int projection_engine_top(const double* meas3, const uint8_t* model_base,
                          const ps_psm_header_t* h, int P, int K,
                          double* score_out, double* score_ch, ProjScoreFp* res);
int pdhg_core_top(const LrFactorsFp& F, const double* d, const PdhgShapeFp& shape,
                  const PdhgParamsFp& P, double* f, PdhgWorkFp& W, PdhgOutFp& out,
                  double* res_hist, int res_hist_cap);
}  // namespace ps

// ============================================================================
// 1) 低秩前向  y = U · diag(S) · (V f)
// ============================================================================
extern "C" int ps_lr_forward_axi(const PS_AXI_COEF_T* U,
                                 const PS_AXI_COEF_T* S,
                                 const PS_AXI_COEF_T* V,
                                 int K, int M, int N,
                                 const PS_AXI_SAMPLE_T* f,
                                 PS_AXI_SAMPLE_T* y) {
#ifdef __SYNTHESIS__
#pragma HLS INTERFACE m_axi port=U offset=slave bundle=gmem0_model depth=583680
#pragma HLS INTERFACE m_axi port=S offset=slave bundle=gmem0_model depth=64
#pragma HLS INTERFACE m_axi port=V offset=slave bundle=gmem0_model depth=27648
#pragma HLS INTERFACE m_axi port=f offset=slave bundle=gmem1_data  depth=1024
#pragma HLS INTERFACE m_axi port=y offset=slave bundle=gmem1_data  depth=16384
#pragma HLS INTERFACE s_axilite port=K bundle=control
#pragma HLS INTERFACE s_axilite port=M bundle=control
#pragma HLS INTERFACE s_axilite port=N bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#endif
    if (!U || !S || !V || !f || !y) return PS_AXI_ERR_ARGS;
    if (K <= 0 || M <= 0 || N <= 0) return PS_AXI_ERR_ARGS;
    if (K > PS_RANK_MAX) return PS_AXI_ERR_DIM;

    ps_axi_factors_t F;
    F.U = U; F.S = S; F.V = V;
    F.K = K; F.M = M; F.N = N;
    return ps::lr_forward_top(F, f, y);
}

// ============================================================================
// 2) 低秩伴随  g = Vᵀ · diag(S) · (Uᵀ r)
// ============================================================================
extern "C" int ps_lr_adjoint_axi(const PS_AXI_COEF_T* U,
                                 const PS_AXI_COEF_T* S,
                                 const PS_AXI_COEF_T* V,
                                 int K, int M, int N,
                                 const PS_AXI_SAMPLE_T* r,
                                 PS_AXI_SAMPLE_T* g) {
#ifdef __SYNTHESIS__
#pragma HLS INTERFACE m_axi port=U offset=slave bundle=gmem0_model depth=583680
#pragma HLS INTERFACE m_axi port=S offset=slave bundle=gmem0_model depth=64
#pragma HLS INTERFACE m_axi port=V offset=slave bundle=gmem0_model depth=27648
#pragma HLS INTERFACE m_axi port=r offset=slave bundle=gmem1_data  depth=16384
#pragma HLS INTERFACE m_axi port=g offset=slave bundle=gmem1_data  depth=1024
#pragma HLS INTERFACE s_axilite port=K bundle=control
#pragma HLS INTERFACE s_axilite port=M bundle=control
#pragma HLS INTERFACE s_axilite port=N bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#endif
    if (!U || !S || !V || !r || !g) return PS_AXI_ERR_ARGS;
    if (K <= 0 || M <= 0 || N <= 0) return PS_AXI_ERR_ARGS;
    if (K > PS_RANK_MAX) return PS_AXI_ERR_DIM;

    ps_axi_factors_t F;
    F.U = U; F.S = S; F.V = V;
    F.K = K; F.M = M; F.N = N;
    return ps::lr_adjoint_top(F, r, g);
}

// ============================================================================
// 3) 定位：候选打分与排序
// ============================================================================
extern "C" int ps_projection_axi(const PS_AXI_SAMPLE_T* meas3,
                                 const uint8_t*         model_base,
                                 const void*            h,
                                 int P, int K,
                                 PS_AXI_SAMPLE_T* score_out,
                                 PS_AXI_SAMPLE_T* score_ch,
                                 int*   best_id,
                                 double* margin) {
#ifdef __SYNTHESIS__
#pragma HLS INTERFACE m_axi port=meas3      offset=slave bundle=gmem1_data  depth=27360
#pragma HLS INTERFACE m_axi port=model_base offset=slave bundle=gmem0_model depth=300589120
#pragma HLS INTERFACE m_axi port=h          offset=slave bundle=gmem0_model depth=64
#pragma HLS INTERFACE m_axi port=score_out  offset=slave bundle=gmem1_data  depth=1024
#pragma HLS INTERFACE m_axi port=score_ch   offset=slave bundle=gmem1_data  depth=3072
#pragma HLS INTERFACE m_axi port=best_id    offset=slave bundle=gmem1_data  depth=1
#pragma HLS INTERFACE m_axi port=margin     offset=slave bundle=gmem1_data  depth=1
#pragma HLS INTERFACE s_axilite port=P bundle=control
#pragma HLS INTERFACE s_axilite port=K bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#endif
    if (!meas3 || !model_base || !h || !score_out || !score_ch || !best_id || !margin)
        return PS_AXI_ERR_ARGS;
    if (P <= 0 || K <= 0) return PS_AXI_ERR_ARGS;
    if (K > PS_RANK_MAX) return PS_AXI_ERR_DIM;

    ps_axi_proj_res_t res;
    const ps_psm_header_t* hdr = static_cast<const ps_psm_header_t*>(h);
    int rc = ps::projection_engine_top(meas3, model_base, hdr, P, K,
                                       score_out, score_ch, &res);
    // 把结构体里的两个标量落到调用方缓冲，避免 bd 侧再去解 struct
    *best_id = res.best_id;
    *margin  = res.margin;
    return rc;
}

// ============================================================================
// 4) 重建：PDHG 单通道
// ============================================================================
extern "C" int ps_pdhg_axi(const PS_AXI_COEF_T* U,
                           const PS_AXI_COEF_T* S,
                           const PS_AXI_COEF_T* V,
                           int K, int M, int N,
                           const PS_AXI_SAMPLE_T* d,
                           int n_rows, int n_cols,
                           int n_iter, int min_iters, int residual_every,
                           double tau, double sigma, double lam,
                           double theta, double rho_stop,
                           PS_AXI_SAMPLE_T* f,
                           PS_AXI_SAMPLE_T* f_prev, PS_AXI_SAMPLE_T* fbar,
                           PS_AXI_SAMPLE_T* q, PS_AXI_SAMPLE_T* grad,
                           PS_AXI_SAMPLE_T* divq,
                           PS_AXI_SAMPLE_T* bf, PS_AXI_SAMPLE_T* r,
                           PS_AXI_SAMPLE_T* btr,
                           double* res_hist, int res_hist_cap,
                           int* n_used, int* early_stopped,
                           int* n_res, int* monotone,
                           double* rho, double* rho_first,
                           double* res_norm, double* d_norm) {
#ifdef __SYNTHESIS__
#pragma HLS INTERFACE m_axi port=U offset=slave bundle=gmem0_model depth=583680
#pragma HLS INTERFACE m_axi port=S offset=slave bundle=gmem0_model depth=64
#pragma HLS INTERFACE m_axi port=V offset=slave bundle=gmem0_model depth=27648
#pragma HLS INTERFACE m_axi port=d offset=slave bundle=gmem1_data  depth=27360
#pragma HLS INTERFACE m_axi port=f offset=slave bundle=gmem1_data  depth=1024
// 8 个迭代暂存走独立 bundle，避免与模型抢 gmem0 的带宽。
// **这是第一版的可工作取舍，不是最终形态** —— 见 docs/hls_interface_spec.md §六：
// 这些缓冲（合计约 6N+2M 个样点）最终应当搬到片上 BRAM，
// 否则每轮迭代都要走一次 DDR，带宽会先于算力成为瓶颈。
#pragma HLS INTERFACE m_axi port=f_prev offset=slave bundle=gmem2_scratch depth=1024
#pragma HLS INTERFACE m_axi port=fbar   offset=slave bundle=gmem2_scratch depth=1024
#pragma HLS INTERFACE m_axi port=q      offset=slave bundle=gmem2_scratch depth=2048
#pragma HLS INTERFACE m_axi port=grad   offset=slave bundle=gmem2_scratch depth=2048
#pragma HLS INTERFACE m_axi port=divq   offset=slave bundle=gmem2_scratch depth=1024
#pragma HLS INTERFACE m_axi port=bf     offset=slave bundle=gmem2_scratch depth=16384
#pragma HLS INTERFACE m_axi port=r      offset=slave bundle=gmem2_scratch depth=16384
#pragma HLS INTERFACE m_axi port=btr    offset=slave bundle=gmem2_scratch depth=1024
#pragma HLS INTERFACE m_axi port=res_hist offset=slave bundle=gmem1_data depth=32
#pragma HLS INTERFACE m_axi port=n_used        offset=slave bundle=gmem1_data depth=1
#pragma HLS INTERFACE m_axi port=early_stopped offset=slave bundle=gmem1_data depth=1
#pragma HLS INTERFACE m_axi port=n_res         offset=slave bundle=gmem1_data depth=1
#pragma HLS INTERFACE m_axi port=monotone      offset=slave bundle=gmem1_data depth=1
#pragma HLS INTERFACE m_axi port=rho           offset=slave bundle=gmem1_data depth=1
#pragma HLS INTERFACE m_axi port=rho_first     offset=slave bundle=gmem1_data depth=1
#pragma HLS INTERFACE m_axi port=res_norm      offset=slave bundle=gmem1_data depth=1
#pragma HLS INTERFACE m_axi port=d_norm        offset=slave bundle=gmem1_data depth=1
#pragma HLS INTERFACE s_axilite port=K bundle=control
#pragma HLS INTERFACE s_axilite port=M bundle=control
#pragma HLS INTERFACE s_axilite port=N bundle=control
#pragma HLS INTERFACE s_axilite port=n_rows bundle=control
#pragma HLS INTERFACE s_axilite port=n_cols bundle=control
#pragma HLS INTERFACE s_axilite port=n_iter bundle=control
#pragma HLS INTERFACE s_axilite port=min_iters bundle=control
#pragma HLS INTERFACE s_axilite port=residual_every bundle=control
#pragma HLS INTERFACE s_axilite port=tau bundle=control
#pragma HLS INTERFACE s_axilite port=sigma bundle=control
#pragma HLS INTERFACE s_axilite port=lam bundle=control
#pragma HLS INTERFACE s_axilite port=theta bundle=control
#pragma HLS INTERFACE s_axilite port=rho_stop bundle=control
#pragma HLS INTERFACE s_axilite port=res_hist_cap bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
#endif
    if (!U || !S || !V || !d || !f) return PS_AXI_ERR_ARGS;
    if (!f_prev || !fbar || !q || !grad || !divq || !bf || !r || !btr)
        return PS_AXI_ERR_ARGS;
    if (!n_used || !early_stopped || !n_res || !monotone)
        return PS_AXI_ERR_ARGS;
    if (!rho || !rho_first || !res_norm || !d_norm) return PS_AXI_ERR_ARGS;
    if (K <= 0 || M <= 0 || N <= 0) return PS_AXI_ERR_ARGS;
    if (K > PS_RANK_MAX) return PS_AXI_ERR_DIM;
    // 形状与 N 必须自洽：pdhg_core 内部会再校验一次，这里早退省一次访存
    if (n_rows <= 0 || n_cols <= 0 || n_rows * n_cols != N) return PS_AXI_ERR_SHAPE;

    ps_axi_factors_t F;
    F.U = U; F.S = S; F.V = V;
    F.K = K; F.M = M; F.N = N;

    ps::PdhgShapeFp shape;
    shape.n_rows = n_rows;
    shape.n_cols = n_cols;

    ps::PdhgParamsFp P;
    P.n_iter         = n_iter;
    P.min_iters      = min_iters;
    P.residual_every = residual_every;
    P.tau            = tau;
    P.sigma          = sigma;
    P.lam            = lam;
    P.theta          = theta;
    P.rho_stop       = rho_stop;

    ps::PdhgWorkFp W;
    W.f_prev = f_prev; W.fbar = fbar;
    W.q      = q;      W.grad = grad; W.divq = divq;
    W.bf     = bf;     W.r    = r;    W.btr  = btr;

    ps::PdhgOutFp out;
    int rc = ps::pdhg_core_top(F, d, shape, P, f, W, out, res_hist, res_hist_cap);

    *n_used        = out.n_used;
    *early_stopped = out.early_stopped;
    *n_res         = out.n_res;
    *monotone      = out.monotone;
    *rho           = out.rho;
    *rho_first     = out.rho_first;
    *res_norm      = out.res_norm;
    *d_norm        = out.d_norm;
    return rc;
}
