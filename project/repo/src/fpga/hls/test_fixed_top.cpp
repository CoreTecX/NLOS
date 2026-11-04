// ============================================================================
// fix_top_smoke.cpp —— ps_fixed_top 的透明度冒烟测试（**临时文件，不进仓库**）
//
// 对 4 个 ps_*_fix 顶层做「透明度断言」：同一组入参分别调用
//   · 顶层      ps_*_fix(...)
//   · 直接调用  ps::<core><PS_FIX_LANES>(...)
// 断言两边输出**逐值相等**（顶层只做拼装/转调/回写，不许有任何数值加工）。
//
// 维度取极小值：P=2, K=2, M=4, N=4, shape={2,2}；
// 数据用确定性伪随机 code = ((i*37 + 11) % 101) - 50；增益取小的正数 1<<12。
// ============================================================================

#include "ps_fixed_top.hpp"

#include "projection_engine/projection_engine.hpp"
#include "lr_operator/lr_operator.hpp"
#include "pdhg_core/pdhg_core.hpp"

#include <cstdint>
#include <cstdio>

// ---------------------------------------------------------------- 断言计数
static int g_pass = 0;
static int g_fail = 0;

static void ck(bool ok, const char* what) {
    if (ok) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("      FAIL: %s\n", what);
    }
}

// ---------------------------------------------------------------- 确定性数据
static int code(int i) { return ((i * 37 + 11) % 101) - 50; }              // [-50, 50]
static uint16_t scode(int i) { return (uint16_t)((i * 37 + 11) % 101); }   // [0, 100]

// ============================================================================
// 1) 定位核
// ============================================================================
static void test_projection() {
    std::printf("[1] ps_projection_fix vs ps::projection_engine<%d>\n",
                (int)PS_FIX_LANES);
    const int P = 2, K = 2, M = 4;
    const int nx = 2, ny = 1, nz = 1;
    const int thr = 1 << 15;

    int32_t meas[3 * M];
    for (int i = 0; i < 3 * M; ++i) meas[i] = code(i + 1);
    int16_t u_codes[P * K * M];
    for (int i = 0; i < P * K * M; ++i) u_codes[i] = (int16_t)code(i + 3);
    int32_t gain_q16[P];
    for (int p = 0; p < P; ++p) gain_q16[p] = 1 << 12;

    // ---- 顶层 ----
    uint64_t s_out_t[P], s_ch_t[P * 3];
    for (int i = 0; i < P; ++i) s_out_t[i] = 0;
    for (int i = 0; i < P * 3; ++i) s_ch_t[i] = 0;
    int t_best = -9, t_second = -9, t_margin = -9;
    int t_bound = -9, t_low = -9, t_ovf = -9;
    unsigned long long t_bests = 0, t_seconds = 0;
    int t_sat = -9, t_mac = -9, t_out = -9, t_score = -9, t_gain = -9;
    long long t_max = -9;
    int rc_t = ps_projection_fix(meas, u_codes, gain_q16, P, K, M, nx, ny, nz, thr,
                                 s_out_t, s_ch_t, &t_best, &t_second, &t_margin,
                                 &t_bound, &t_low, &t_ovf, &t_bests, &t_seconds,
                                 &t_sat, &t_mac, &t_out, &t_score, &t_gain, &t_max);

    // ---- 直接调用核 ----
    uint64_t s_out_d[P], s_ch_d[P * 3];
    for (int i = 0; i < P; ++i) s_out_d[i] = 0;
    for (int i = 0; i < P * 3; ++i) s_ch_d[i] = 0;
    ps::ProjResult res;
    ps::FixFlags fl;
    ps::fix_flags_reset(fl);
    int rc_d = ps::projection_engine<PS_FIX_LANES>(
        meas, u_codes, gain_q16, P, K, M, nx, ny, nz, thr,
        s_out_d, s_ch_d, res, fl);

    ck(rc_t == rc_d, "return code");
    ck(rc_t == ps::kProjOk, "rc == kProjOk");
    for (int p = 0; p < P; ++p) ck(s_out_t[p] == s_out_d[p], "score_out[p]");
    for (int i = 0; i < P * 3; ++i) ck(s_ch_t[i] == s_ch_d[i], "score_ch[i]");
    ck(t_best == res.best_id, "best_id");
    ck(t_second == res.second_id, "second_id");
    ck(t_margin == res.margin_q16, "margin_q16");
    ck(t_bound == res.boundary, "boundary");
    ck(t_low == res.low_conf, "low_conf");
    ck(t_ovf == res.overflow, "overflow");
    ck(t_bests == (unsigned long long)res.best_score, "best_score");
    ck(t_seconds == (unsigned long long)res.second_score, "second_score");
    ck(t_sat == fl.sat_events, "flags.sat_events");
    ck(t_mac == fl.mac_overflow, "flags.mac_overflow");
    ck(t_out == fl.out_overflow, "flags.out_overflow");
    ck(t_score == fl.score_overflow, "flags.score_overflow");
    ck(t_gain == fl.gain_clamped, "flags.gain_clamped");
    ck(t_max == fl.max_abs_acc, "flags.max_abs_acc");

    std::printf("      best_id=%d second_id=%d margin_q16=%d boundary=%d low_conf=%d"
                " overflow=%d\n", t_best, t_second, t_margin, t_bound, t_low, t_ovf);
    std::printf("      best_score=%llu second_score=%llu score_out=[%llu %llu]"
                " max_abs_acc=%lld\n",
                t_bests, t_seconds,
                (unsigned long long)s_out_t[0], (unsigned long long)s_out_t[1],
                t_max);
    std::printf("      flags: sat=%d mac=%d out=%d score=%d gain=%d\n",
                t_sat, t_mac, t_out, t_score, t_gain);
}

// ============================================================================
// 2) 低秩前向 / 伴随（含 wk_* 全 nullptr 的边界）
// ============================================================================
static void test_lr() {
    std::printf("[2] ps_lr_forward_fix / ps_lr_adjoint_fix vs ps::lr_*<%d>\n",
                (int)PS_FIX_LANES);
    const int K = 2, M = 4, N = 4;
    const int32_t gq = 1 << 12;             // 小的正增益（Q16.16）
    const int exp_invalid = ps::kGainExpInvalid;

    int16_t U[K * M];
    for (int i = 0; i < K * M; ++i) U[i] = (int16_t)code(i + 3);
    uint16_t S[K];
    for (int k = 0; k < K; ++k) S[k] = scode(k + 7);
    int16_t V[K * N];
    for (int i = 0; i < K * N; ++i) V[i] = (int16_t)code(i + 5);

    int32_t f[N];
    for (int i = 0; i < N; ++i) f[i] = code(i + 11);
    int32_t r[M];
    for (int i = 0; i < M; ++i) r[i] = code(i + 13);

    ps::LrFactors F;
    F.U = U; F.S = S; F.V = V;
    F.K = K; F.M = M; F.N = N;
    F.u_gain_q16 = gq; F.s_gain_q16 = gq; F.v_gain_q16 = gq;
    F.u_exp = exp_invalid; F.s_exp = exp_invalid; F.v_exp = exp_invalid;

    // ---------------- forward ----------------
    int32_t y_t[M], y_d[M];
    for (int i = 0; i < M; ++i) { y_t[i] = 0; y_d[i] = 0; }
    int32_t wz_t[K], wzs_t[K], wz_d[K], wzs_d[K];
    for (int i = 0; i < K; ++i) { wz_t[i] = 0; wzs_t[i] = 0; wz_d[i] = 0; wzs_d[i] = 0; }
    long long mc_t = -9;
    int f_sat = -9, f_mac = -9, f_out = -9, f_score = -9, f_gain = -9;
    long long f_max = -9;
    int rc_t = ps_lr_forward_fix(U, S, V, K, M, N, gq, gq, gq,
                                 exp_invalid, exp_invalid, exp_invalid,
                                 f, y_t, wz_t, wzs_t, &mc_t,
                                 &f_sat, &f_mac, &f_out, &f_score, &f_gain, &f_max);

    ps::LrDiag dg; ps::LrWork Wd;
    Wd.z = wz_d; Wd.zs = wzs_d; Wd.w = 0; Wd.ws = 0;
    int rc_d = ps::lr_forward<PS_FIX_LANES>(F, f, y_d, dg, &Wd);

    ck(rc_t == rc_d, "forward return code");
    ck(rc_t == ps::kLrOk, "forward rc == kLrOk");
    for (int i = 0; i < M; ++i) ck(y_t[i] == y_d[i], "forward y[i]");
    for (int k = 0; k < K; ++k) ck(wz_t[k] == wz_d[k], "forward wk_z[k]");
    for (int k = 0; k < K; ++k) ck(wzs_t[k] == wzs_d[k], "forward wk_zs[k]");
    ck(mc_t == dg.mac_count, "forward mac_count");
    ck(f_sat == dg.flags.sat_events, "forward flags.sat_events");
    ck(f_mac == dg.flags.mac_overflow, "forward flags.mac_overflow");
    ck(f_out == dg.flags.out_overflow, "forward flags.out_overflow");
    ck(f_score == dg.flags.score_overflow, "forward flags.score_overflow");
    ck(f_gain == dg.flags.gain_clamped, "forward flags.gain_clamped");
    ck(f_max == dg.flags.max_abs_acc, "forward flags.max_abs_acc");
    std::printf("      forward y=[%d %d %d %d] mac_count=%lld max_abs_acc=%lld\n",
                y_t[0], y_t[1], y_t[2], y_t[3], mc_t, f_max);

    // ---------------- adjoint ----------------
    int32_t g_t[N], g_d[N];
    for (int i = 0; i < N; ++i) { g_t[i] = 0; g_d[i] = 0; }
    int32_t ww_t[K], wws_t[K], ww_d[K], wws_d[K];
    for (int i = 0; i < K; ++i) { ww_t[i] = 0; wws_t[i] = 0; ww_d[i] = 0; wws_d[i] = 0; }
    long long ma_t = -9;
    int a_sat = -9, a_mac = -9, a_out = -9, a_score = -9, a_gain = -9;
    long long a_max = -9;
    int rc_at = ps_lr_adjoint_fix(U, S, V, K, M, N, gq, gq, gq,
                                  exp_invalid, exp_invalid, exp_invalid,
                                  r, g_t, ww_t, wws_t, &ma_t,
                                  &a_sat, &a_mac, &a_out, &a_score, &a_gain, &a_max);

    ps::LrDiag dga; ps::LrWork Wa;
    Wa.z = 0; Wa.zs = 0; Wa.w = ww_d; Wa.ws = wws_d;
    int rc_ad = ps::lr_adjoint<PS_FIX_LANES>(F, r, g_d, dga, &Wa);

    ck(rc_at == rc_ad, "adjoint return code");
    ck(rc_at == ps::kLrOk, "adjoint rc == kLrOk");
    for (int i = 0; i < N; ++i) ck(g_t[i] == g_d[i], "adjoint g[i]");
    for (int k = 0; k < K; ++k) ck(ww_t[k] == ww_d[k], "adjoint wk_w[k]");
    for (int k = 0; k < K; ++k) ck(wws_t[k] == wws_d[k], "adjoint wk_ws[k]");
    ck(ma_t == dga.mac_count, "adjoint mac_count");
    ck(a_sat == dga.flags.sat_events, "adjoint flags.sat_events");
    ck(a_mac == dga.flags.mac_overflow, "adjoint flags.mac_overflow");
    ck(a_out == dga.flags.out_overflow, "adjoint flags.out_overflow");
    ck(a_score == dga.flags.score_overflow, "adjoint flags.score_overflow");
    ck(a_gain == dga.flags.gain_clamped, "adjoint flags.gain_clamped");
    ck(a_max == dga.flags.max_abs_acc, "adjoint flags.max_abs_acc");
    std::printf("      adjoint g=[%d %d %d %d] mac_count=%lld max_abs_acc=%lld\n",
                g_t[0], g_t[1], g_t[2], g_t[3], ma_t, a_max);

    // ---------------- 边界：wk_* 全传 nullptr ----------------
    int32_t y_n[M];
    for (int i = 0; i < M; ++i) y_n[i] = 0;
    long long mc_n = -9;
    int n_sat = -9, n_mac = -9, n_out = -9, n_score = -9, n_gain = -9;
    long long n_max = -9;
    int rc_n = ps_lr_forward_fix(U, S, V, K, M, N, gq, gq, gq,
                                 exp_invalid, exp_invalid, exp_invalid,
                                 f, y_n, 0, 0, &mc_n,
                                 &n_sat, &n_mac, &n_out, &n_score, &n_gain, &n_max);

    int32_t y_dn[M];
    for (int i = 0; i < M; ++i) y_dn[i] = 0;
    ps::LrDiag dgn;
    int rc_dn = ps::lr_forward<PS_FIX_LANES>(F, f, y_dn, dgn, 0);

    ck(rc_n == rc_dn, "forward(wk=null) return code");
    ck(rc_n == ps::kLrOk, "forward(wk=null) rc == kLrOk");
    for (int i = 0; i < M; ++i) ck(y_n[i] == y_dn[i], "forward(wk=null) y[i]");
    for (int i = 0; i < M; ++i) ck(y_n[i] == y_t[i], "forward(wk=null) y == y(wk)");
    ck(mc_n == mc_t, "forward(wk=null) mac_count == mac_count(wk)");

    int32_t g_n[N];
    for (int i = 0; i < N; ++i) g_n[i] = 0;
    long long ma_n = -9;
    int m_sat = -9, m_mac = -9, m_out = -9, m_score = -9, m_gain = -9;
    long long m_max = -9;
    int rc_an = ps_lr_adjoint_fix(U, S, V, K, M, N, gq, gq, gq,
                                  exp_invalid, exp_invalid, exp_invalid,
                                  r, g_n, 0, 0, &ma_n,
                                  &m_sat, &m_mac, &m_out, &m_score, &m_gain, &m_max);
    int32_t g_dn[N];
    for (int i = 0; i < N; ++i) g_dn[i] = 0;
    ps::LrDiag dgan;
    int rc_adn = ps::lr_adjoint<PS_FIX_LANES>(F, r, g_dn, dgan, 0);

    ck(rc_an == rc_adn, "adjoint(wk=null) return code");
    ck(rc_an == ps::kLrOk, "adjoint(wk=null) rc == kLrOk");
    for (int i = 0; i < N; ++i) ck(g_n[i] == g_dn[i], "adjoint(wk=null) g[i]");
    for (int i = 0; i < N; ++i) ck(g_n[i] == g_t[i], "adjoint(wk=null) g == g(wk)");
    ck(ma_n == ma_t, "adjoint(wk=null) mac_count == mac_count(wk)");
    std::printf("      wk_* = nullptr 边界通过（y/g 与带观测版本逐值相同）\n");
}

// ============================================================================
// 3) 重建核
// ============================================================================
static void test_pdhg() {
    std::printf("[3] ps_pdhg_fix vs ps::pdhg_core<%d>\n", (int)PS_FIX_LANES);
    const int K = 2, M = 4, N = 4;
    const int n_rows = 2, n_cols = 2;
    const int32_t gq = 1 << 12;
    const int exp_invalid = ps::kGainExpInvalid;

    int16_t U[K * M];
    for (int i = 0; i < K * M; ++i) U[i] = (int16_t)code(i + 3);
    uint16_t S[K];
    for (int k = 0; k < K; ++k) S[k] = scode(k + 7);
    int16_t V[K * N];
    for (int i = 0; i < K * N; ++i) V[i] = (int16_t)code(i + 5);

    int32_t d3[3 * M];
    for (int i = 0; i < 3 * M; ++i) d3[i] = code(i + 2);

    // 参数：n_iter 取小值；residual_every>0 让残差路径也被走到；
    // rho_stop_q8 = 0 -> 不早停（核的规则：<=0 不早停）。
    // min_iters 核不校验，取 1（<= n_iter=2）为合法值。
    const int n_iter = 2, min_iters = 1, residual_every = 1, tv_mode = ps::kTvIsotropicL2;
    const int32_t tau = 1 << 12, sigma = 1 << 12, lam = 1 << 12;
    const int32_t theta = 1 << 16, rho_stop_q8 = 0;

    int32_t f3_t[3 * N], f3_d[3 * N];
    for (int i = 0; i < 3 * N; ++i) { f3_t[i] = 0; f3_d[i] = 0; }

    int32_t t_fprev[N], t_fbar[N], t_q[2 * N], t_grad[2 * N], t_divq[N];
    int32_t t_y[M], t_r[M], t_btr[N], t_tmp[N];
    int32_t d_fprev[N], d_fbar[N], d_q[2 * N], d_grad[2 * N], d_divq[N];
    int32_t d_y[M], d_r[M], d_btr[N], d_tmp[N];
    for (int i = 0; i < N; ++i) {
        t_fprev[i] = t_fbar[i] = t_divq[i] = t_btr[i] = t_tmp[i] = 0;
        d_fprev[i] = d_fbar[i] = d_divq[i] = d_btr[i] = d_tmp[i] = 0;
    }
    for (int i = 0; i < 2 * N; ++i) { t_q[i] = t_grad[i] = 0; d_q[i] = d_grad[i] = 0; }
    for (int i = 0; i < M; ++i) { t_y[i] = t_r[i] = 0; d_y[i] = d_r[i] = 0; }

    int o_nu_t[3], o_es_t[3], o_ov_t[3];
    unsigned int o_rho_t[3];
    unsigned long long o_rs_t[3], o_ds_t[3];
    long long o_mc_t[3];
    for (int c = 0; c < 3; ++c) { o_nu_t[c]=-9; o_es_t[c]=-9; o_ov_t[c]=-9; o_rho_t[c]=0; o_rs_t[c]=0; o_ds_t[c]=0; o_mc_t[c]=-9; }
    int p_sat = -9, p_mac = -9, p_out = -9, p_score = -9, p_gain = -9;
    long long p_max = -9;

    int rc_t = ps_pdhg_fix(U, S, V, K, M, N, gq, gq, gq,
                           exp_invalid, exp_invalid, exp_invalid,
                           d3, n_rows, n_cols,
                           n_iter, min_iters, residual_every, tv_mode,
                           tau, sigma, lam, theta, rho_stop_q8,
                           f3_t,
                           t_fprev, t_fbar, t_q, t_grad, t_divq, t_y, t_r, t_btr, t_tmp,
                           o_nu_t, o_es_t, o_ov_t, o_rho_t, o_rs_t, o_ds_t, o_mc_t,
                           &p_sat, &p_mac, &p_out, &p_score, &p_gain, &p_max);

    ps::LrFactors F;
    F.U = U; F.S = S; F.V = V;
    F.K = K; F.M = M; F.N = N;
    F.u_gain_q16 = gq; F.s_gain_q16 = gq; F.v_gain_q16 = gq;
    F.u_exp = exp_invalid; F.s_exp = exp_invalid; F.v_exp = exp_invalid;
    ps::ReconShape shape; shape.n_rows = n_rows; shape.n_cols = n_cols;
    ps::PdhgParams P;
    P.n_iter = n_iter; P.min_iters = min_iters; P.residual_every = residual_every;
    P.tv_mode = tv_mode;
    P.tau_q16 = tau; P.sigma_q16 = sigma; P.lambda_q16 = lam;
    P.theta_q16 = theta; P.rho_stop_q8 = rho_stop_q8;
    ps::PdhgWork W;
    W.f_prev = d_fprev; W.fbar = d_fbar; W.q = d_q; W.grad = d_grad; W.divq = d_divq;
    W.y = d_y; W.r = d_r; W.btr = d_btr; W.tmp = d_tmp;
    ps::PdhgOut out3[3];
    int rc_d = ps::pdhg_core<PS_FIX_LANES>(F, d3, shape, P, f3_d, W, out3);

    ck(rc_t == rc_d, "return code");
    ck(rc_t == ps::kPdhgOk, "rc == kPdhgOk");
    for (int i = 0; i < 3 * N; ++i) ck(f3_t[i] == f3_d[i], "f3[i]");
    for (int c = 0; c < 3; ++c) {
        ck(o_nu_t[c] == out3[c].n_used, "out_n_used[c]");
        ck(o_es_t[c] == out3[c].early_stopped, "out_early_stopped[c]");
        ck(o_ov_t[c] == out3[c].overflow, "out_overflow_ch[c]");
        ck(o_rho_t[c] == out3[c].rho_q16, "out_rho_q16[c]");
        ck(o_rs_t[c] == (unsigned long long)out3[c].res_sq, "out_res_sq[c]");
        ck(o_ds_t[c] == (unsigned long long)out3[c].d_sq, "out_d_sq[c]");
        ck(o_mc_t[c] == out3[c].mac_count, "out_mac_count[c]");
    }
    // pdhg_core 不输出内部 FixFlags：顶层这 6 个标量来自按值构造并清零的
    // FixFlags，故与「清零 FixFlags」逐字段相等（每通道溢出见 out_overflow_ch）。
    ps::FixFlags flzero;
    ps::fix_flags_reset(flzero);
    ck(p_sat == flzero.sat_events, "flags.sat_events (==0)");
    ck(p_mac == flzero.mac_overflow, "flags.mac_overflow (==0)");
    ck(p_out == flzero.out_overflow, "flags.out_overflow (==0)");
    ck(p_score == flzero.score_overflow, "flags.score_overflow (==0)");
    ck(p_gain == flzero.gain_clamped, "flags.gain_clamped (==0)");
    ck(p_max == flzero.max_abs_acc, "flags.max_abs_acc (==0)");

    std::printf("      f3=[%d %d %d %d | %d %d %d %d | %d %d %d %d]\n",
                f3_t[0], f3_t[1], f3_t[2], f3_t[3],
                f3_t[4], f3_t[5], f3_t[6], f3_t[7],
                f3_t[8], f3_t[9], f3_t[10], f3_t[11]);
    for (int c = 0; c < 3; ++c) {
        std::printf("      ch%d: n_used=%d early=%d ovf=%d rho_q16=%u"
                    " res_sq=%llu d_sq=%llu mac=%lld\n",
                    c, o_nu_t[c], o_es_t[c], o_ov_t[c], o_rho_t[c],
                    o_rs_t[c], o_ds_t[c], o_mc_t[c]);
    }
}

int main() {
    std::printf("=== ps_fixed_top 透明度冒烟测试 (PS_FIX_LANES=%d) ===\n",
                (int)PS_FIX_LANES);
    test_projection();
    test_lr();
    test_pdhg();
    std::printf("=== 断言: %d PASS / %d FAIL ===\n", g_pass, g_fail);
    std::printf(g_fail == 0 ? "RESULT: PASS\n" : "RESULT: FAIL\n");
    return g_fail == 0 ? 0 : 1;
}
