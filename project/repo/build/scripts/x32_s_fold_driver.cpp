// ============================================================================
// X32 —— 用 HLS 定点核验证「s_max 折进 s_gain_q16」这条下发路径
//
// 与 build/scripts/x20_fixed_vs_double.cpp 的唯一差别：
//   · 额外读一个命令行参数 s_gain_q16，写进 Ff.s_gain_q16（x20 恒为 0）；
//   · 由同一批码字还原双精度因子时，若 case 的 s_exp == 0x7FFFFFFF
//     （= ps::kGainExpInvalid，即核走 Q16.16 支路），则 S 的还原比例改用
//     s_gain_q16 / 2^16，而不是 ldexp(1, s_exp)。
// 这样同一条码字输入可以分别跑「幂次支路」（s_exp 有效，x20 原路径）与
// 「Q16.16 支路」（s_exp 无效、s_gain_q16 有效，即 s_max 折算后的路径），
// 两次都拿未量化的双精度因子当参考。
//
// 本文件不改任何 HLS 算法核源码，只 include 它们、按同一套签名喂入参。
// case 二进制格式与 x20 完全相同（magic "X20C"）：15 × int32 头 + 3 × int32
// 指数 + U/S/V 码字 + d_code + Ud/Sd/Vd/d_real（双精度参考）。
//
// 编译：见 build/scripts/x32_psm_e2e_smax.py
// 用法：x32_s_fold_driver.exe <case.bin> <result.bin> [s_gain_q16]
// ============================================================================

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "pdhg_core.hpp"   // 定点层
#include "pdhg_core.h"     // 双精度层

namespace {

constexpr int kGainExpInvalid = 0x7FFFFFFF;   // == ps::kGainExpInvalid

struct Case {
    int32_t K = 0, M = 0, N = 0, n_rows = 0, n_cols = 0;
    int32_t n_iter = 0, min_iters = 0, residual_every = 0, tv_mode = 0;
    int32_t tau_q16 = 0, sigma_q16 = 0, theta_q16 = 0, lambda_q16 = 0;
    int32_t rho_stop_q8 = 0;
    int32_t u_exp = 0, v_exp = 0, s_exp = 0;
    std::vector<int16_t> U;
    std::vector<uint16_t> S;
    std::vector<int16_t> V;
    std::vector<int32_t> d_code;
    std::vector<double> Ud, Sd, Vd, d_real;
};

template <typename T>
bool rd(std::FILE* f, std::vector<T>& v, std::size_t n) {
    v.resize(n);
    return std::fread(v.data(), sizeof(T), n, f) == n;
}

bool read_case(const char* path, Case& c) {
    std::FILE* f = std::fopen(path, "rb");
    if (!f) { std::printf("cannot open case %s\n", path); return false; }
    char magic[4];
    int32_t hdr[15], exps[3];
    bool ok = (std::fread(magic, 1, 4, f) == 4) &&
              (std::memcmp(magic, "X20C", 4) == 0) &&
              (std::fread(hdr, sizeof(int32_t), 15, f) == 15) &&
              (std::fread(exps, sizeof(int32_t), 3, f) == 3);
    if (!ok) { std::printf("bad header\n"); std::fclose(f); return false; }
    c.K = hdr[0]; c.M = hdr[1]; c.N = hdr[2]; c.n_rows = hdr[3]; c.n_cols = hdr[4];
    c.n_iter = hdr[5]; c.min_iters = hdr[6]; c.residual_every = hdr[7]; c.tv_mode = hdr[8];
    c.tau_q16 = hdr[9]; c.sigma_q16 = hdr[10]; c.theta_q16 = hdr[11];
    c.lambda_q16 = hdr[12]; c.rho_stop_q8 = hdr[13];
    c.u_exp = exps[0]; c.v_exp = exps[1]; c.s_exp = exps[2];
    ok = rd<int16_t>(f, c.U, (std::size_t)c.K * c.M) &&
         rd<uint16_t>(f, c.S, (std::size_t)c.K) &&
         rd<int16_t>(f, c.V, (std::size_t)c.K * c.N) &&
         rd<int32_t>(f, c.d_code, (std::size_t)c.M) &&
         rd<double>(f, c.Ud, (std::size_t)c.K * c.M) &&
         rd<double>(f, c.Sd, (std::size_t)c.K) &&
         rd<double>(f, c.Vd, (std::size_t)c.K * c.N) &&
         rd<double>(f, c.d_real, (std::size_t)c.M);
    std::fclose(f);
    if (!ok) { std::printf("short read\n"); return false; }
    return true;
}

double rel_l2(const std::vector<double>& a, const std::vector<double>& b) {
    double num = 0.0, den = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i];
        num += d * d; den += b[i] * b[i];
    }
    return (den > 0.0) ? std::sqrt(num / den) : 0.0;
}
double pearson(const std::vector<double>& a, const std::vector<double>& b) {
    const std::size_t n = a.size();
    double ma = 0.0, mb = 0.0;
    for (std::size_t i = 0; i < n; ++i) { ma += a[i]; mb += b[i]; }
    ma /= (double)n; mb /= (double)n;
    double va = 0.0, vb = 0.0, cov = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double x = a[i] - ma, y = b[i] - mb;
        va += x * x; vb += y * y; cov += x * y;
    }
    return (va > 0.0 && vb > 0.0) ? cov / std::sqrt(va * vb) : 0.0;
}
double psnr_db(const std::vector<double>& a, const std::vector<double>& b,
               double peak) {
    double mse = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double d = a[i] - b[i]; mse += d * d;
    }
    mse /= (double)a.size();
    return (mse > 0.0) ? 10.0 * std::log10(peak * peak / mse) : 1.0 / 0.0;
}

void run_double(const ps::LrFactorsFp& F, const std::vector<double>& d,
                const ps::PdhgShapeFp& sh, const Case& c, std::vector<double>& f_out,
                int& n_used, double& rho) {
    const int N = ps::pdhg_shape_n(sh), M = F.M;
    std::vector<double> f(N, 0.0), fp(N), fb(N), q(2 * N), g(2 * N), dv(N),
        bf(M), r(M), btr(N);
    ps::PdhgWorkFp W;
    W.f_prev = fp.data(); W.fbar = fb.data(); W.q = q.data(); W.grad = g.data();
    W.divq = dv.data(); W.bf = bf.data(); W.r = r.data(); W.btr = btr.data();
    ps::PdhgParamsFp P;
    P.n_iter = c.n_iter; P.min_iters = c.min_iters; P.residual_every = c.residual_every;
    P.tau = (double)c.tau_q16 / 65536.0; P.sigma = (double)c.sigma_q16 / 65536.0;
    P.lam = (double)c.lambda_q16 / 65536.0; P.theta = (double)c.theta_q16 / 65536.0;
    P.rho_stop = (c.rho_stop_q8 > 0) ? (double)c.rho_stop_q8 / 256.0 : 0.0;
    std::vector<double> hist((std::size_t)(c.n_iter + 2), 0.0);
    ps::PdhgOutFp o;
    ps::pdhg_core<16>(F, d.data(), sh, P, f.data(), W, o, hist.data(), (int)hist.size());
    f_out = f; n_used = o.n_used; rho = o.rho;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: %s <case.bin> <result.bin> [s_gain_q16]\n", argv[0]);
        return 2;
    }
    const int32_t s_gain_arg = (argc > 3) ? (int32_t)std::strtol(argv[3], 0, 0) : 0;
    Case c;
    if (!read_case(argv[1], c)) return 2;
    const int K = c.K, M = c.M, N = c.N;
    const bool gain_path = (c.s_exp == kGainExpInvalid);
    std::printf("=== X32 定点 vs 双精度（s_max 折算支路）===\n");
    std::printf("K=%d M=%d N=%d shape=(%d,%d) n_iter=%d lambda_q16=%d tv_mode=%d "
                "s_gain_q16=%d s_exp=%d(%s)\n",
                K, M, N, c.n_rows, c.n_cols, c.n_iter, c.lambda_q16, c.tv_mode,
                (int)s_gain_arg, (int)c.s_exp, gain_path ? "Q16.16 支路" : "幂次支路");

    // ---- 定点因子（码字 + 支路增益）----
    ps::LrFactors Ff;
    Ff.U = c.U.data(); Ff.S = c.S.data(); Ff.V = c.V.data();
    Ff.K = K; Ff.M = M; Ff.N = N;
    Ff.u_gain_q16 = 0; Ff.v_gain_q16 = 0;
    Ff.u_exp = c.u_exp; Ff.v_exp = c.v_exp;
    if (gain_path) {
        Ff.s_gain_q16 = s_gain_arg;      // 核在 s_exp 无效时采用这一支
        Ff.s_exp      = kGainExpInvalid;
    } else {
        Ff.s_gain_q16 = 0;               // 核在 s_exp 有效时忽略 gain
        Ff.s_exp      = c.s_exp;
    }

    // ---- 由同一批码字还原的双精度因子（隔离「定点算术」）----
    const double usc = std::ldexp(1.0, c.u_exp);
    const double vsc = std::ldexp(1.0, c.v_exp);
    const double ssc = gain_path ? ((double)s_gain_arg / 65536.0)
                                 : std::ldexp(1.0, c.s_exp);
    std::vector<double> Udq((std::size_t)K * M), Sdq((std::size_t)K),
        Vdq((std::size_t)K * N);
    for (std::size_t i = 0; i < Udq.size(); ++i) Udq[i] = (double)c.U[i] * usc;
    for (std::size_t i = 0; i < Sdq.size(); ++i) Sdq[i] = (double)c.S[i] * ssc;
    for (std::size_t i = 0; i < Vdq.size(); ++i) Vdq[i] = (double)c.V[i] * vsc;
    ps::LrFactorsFp Fq;
    Fq.U = Udq.data(); Fq.S = Sdq.data(); Fq.V = Vdq.data();
    Fq.K = K; Fq.M = M; Fq.N = N;

    // ---- 未量化的双精度参考因子（= psm 的绝对算子）----
    ps::LrFactorsFp Fr;
    Fr.U = c.Ud.data(); Fr.S = c.Sd.data(); Fr.V = c.Vd.data();
    Fr.K = K; Fr.M = M; Fr.N = N;

    const ps::ReconShape sh = {c.n_rows, c.n_cols};
    const ps::PdhgShapeFp shf = {c.n_rows, c.n_cols};

    // ---- 0) 算子透明度：定点 lr vs 同码字双精度 lr ----
    {
        std::vector<int32_t> f_code(N);
        for (int i = 0; i < N; ++i) {
            f_code[i] = (int32_t)(((long long)((i * 37) % 251)) * 1024);
        }
        std::vector<int32_t> y1(M);
        ps::LrDiag dg;
        ps::lr_forward<16>(Ff, f_code.data(), y1.data(), dg, 0);
        std::vector<double> fr(N), yr(M, 0.0);
        for (int i = 0; i < N; ++i) fr[i] = (double)f_code[i] / 262144.0;
        ps::lr_forward<16>(Fq, fr.data(), yr.data());
        long long num = 0, den = 0;
        for (int m = 0; m < M; ++m) {
            const double yref = yr[m] * 65536.0;
            const long long d = (long long)y1[m] - (long long)std::llround(yref);
            num += d * d; den += (long long)std::llround(yref) * (long long)std::llround(yref);
        }
        const double rel = (den > 0) ? std::sqrt((double)num / (double)den) : 0.0;
        std::printf("[check] lr_forward 定点 vs 同码字双精度：相对 L2 = %.4e"
                    "（定点饱和事件 %d）\n", rel, dg.flags.sat_events);
    }

    // ---- A) 定点 PDHG ----
    std::vector<int32_t> f_fix_code(N, 0);
    std::vector<int32_t> f_prev(N), fbar(N), q(2 * N), grad(2 * N), divq(N),
        y(M), r(M), btr(N), tmp(N);
    ps::PdhgWork W;
    W.f_prev = f_prev.data(); W.fbar = fbar.data(); W.q = q.data();
    W.grad = grad.data(); W.divq = divq.data(); W.y = y.data();
    W.r = r.data(); W.btr = btr.data(); W.tmp = tmp.data();
    ps::PdhgParams P;
    P.n_iter = c.n_iter; P.min_iters = c.min_iters; P.residual_every = c.residual_every;
    P.tv_mode = c.tv_mode; P.tau_q16 = c.tau_q16; P.sigma_q16 = c.sigma_q16;
    P.lambda_q16 = c.lambda_q16; P.theta_q16 = c.theta_q16; P.rho_stop_q8 = c.rho_stop_q8;
    ps::PdhgOut of;
    const int st_fix = ps::pdhg_solve_channel<16>(Ff, c.d_code.data(), sh, P,
                                                  f_fix_code.data(), W, of);

    // ---- B) 同码字 + 双精度算术 ----
    std::vector<double> d_q(M);
    for (int m = 0; m < M; ++m) d_q[m] = (double)c.d_code[m] / 65536.0;
    std::vector<double> f_b; int nu_b = 0; double rho_b = 0.0;
    run_double(Fq, d_q, shf, c, f_b, nu_b, rho_b);

    // ---- C) 未量化因子 + 双精度算术（参考）----
    std::vector<double> f_c; int nu_c = 0; double rho_c = 0.0;
    run_double(Fr, c.d_real, shf, c, f_c, nu_c, rho_c);
    std::printf("[A fixed] status=%d n_used=%d rho=%d/256 overflow=%d mac=%lld "
                "(samecode_double n_used=%d double-ref n_used=%d)\n",
                st_fix, of.n_used, (int)of.rho_q8, of.overflow,
                (long long)of.mac_count, nu_b, nu_c);

    std::vector<double> fx(N);
    for (int i = 0; i < N; ++i) fx[i] = (double)f_fix_code[i] / 262144.0;

    double pmx = 0.0;
    for (int i = 0; i < N; ++i) if (std::fabs(f_c[i]) > pmx) pmx = std::fabs(f_c[i]);
    const double peak = pmx > 0 ? 2.0 * pmx : 1.0;

    auto report = [&](const char* tag, const std::vector<double>& a,
                      const std::vector<double>& b) {
        double mx = 0.0;
        for (int i = 0; i < N; ++i) {
            const double d = std::fabs(a[i] - b[i]); if (d > mx) mx = d;
        }
        std::printf("[cmp] %-30s relL2=%.6e  r=%+.8f  max|d|=%.6e  PSNR=%.2f dB\n",
                    tag, rel_l2(a, b), pearson(a, b), mx, psnr_db(a, b, peak));
    };
    report("fixed vs double-ref", fx, f_c);
    report("fixed vs double-samecode", fx, f_b);

    std::FILE* g = std::fopen(argv[2], "wb");
    if (!g) { std::printf("cannot write %s\n", argv[2]); return 2; }
    std::fwrite("X32R", 1, 4, g);
    const int32_t hdr2[4] = {N, c.n_iter, of.n_used, of.overflow};
    std::fwrite(hdr2, sizeof(int32_t), 4, g);
    std::fwrite(fx.data(), sizeof(double), N, g);
    std::fwrite(f_b.data(), sizeof(double), N, g);
    std::fwrite(f_c.data(), sizeof(double), N, g);
    std::fclose(g);
    std::printf("[out] %s\n", argv[2]);
    return 0;
}
