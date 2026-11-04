// ============================================================================
// pdhg_core 主机侧 testbench · tb.cpp
//
// 编译并运行（在 src/fpga/hls/pdhg_core 目录下）：
//   g++ -std=c++17 -Wall -Wextra pdhg_core.cpp ../lr_operator/lr_operator.cpp tb.cpp -o tb.exe
//   ./tb.exe
//
// 自检内容（合成数据，真跑真比）：
//   C1 TV 三个原语：梯度逐值、散度恒等式 <∇f,q> = −<f,div q>、L2 球投影与半径界
//   C2 无噪合成数据上的 PDHG：残差历史、**单调下降**、收敛到 1e-3 以下、解 = f_true
//   C2b λ = 0 时 θ 不参与（§6.5 的固有性质：f̄ 只进对偶上升，而 λ=0 时 q ≡ 0）
//   C3 早停策略：每 4 次算残差、最小 8 次后方可早停（第 4 次不得早停）
//   C4 λ > 0（各向同性 TV）整条通路：f 非负、||q|| 逐像素不超 λ、θ 确实改变轨迹
//   C5 可选：定义 PS_PDHG_XCHECK_RECON 时，把本层 TV 原语与
//      src/hardware/recon.hpp 的 ps::tv_* 逐值相减（等价性证据）
//
// 合成问题（故意做成「唯一解已知」）
// ---------------------------------------------------------------------------
//   取 B = U·diag(S)·Vᵀ，其中
//     U：M x N（90 x 48）列正交；
//     V：N x N（48 x 48）列正交（正交矩阵，于是行也正交）；
//     S = [1.0, 1.0-0.5/47, ..., 0.5]（条件数 = 2，最慢模态 0.5^2 = 0.25）。
//   于是 B 满列秩、BᵀB 的特征值就是 S²，最小二乘解**唯一**。
//   再取 f_true ∈ [0.2, 1.0]^N（严格正，使非负约束在解附近不起作用），
//   d = B f_true（**无噪**）。此时无约束最小二乘解恰是 f_true，且它可行，
//   所以带非负约束的解也是 f_true，归一化残差应当**单调下降并趋零**。
//   τ、σ 按 §6.5 由幂迭代估出的 ||B||₂ 导出（pdhg_step_sizes）。
// ============================================================================

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>

#include "pdhg_core.h"

#ifdef PS_PDHG_XCHECK_RECON
// 仅用于 C5 的等价性核对；默认不编入（recon.hpp 用裸 pragma，-Wall 会报）。
#include "../../../hardware/recon.hpp"
#endif

namespace {

const int kH = 6;      // 隐藏图行数（z）
const int kW = 8;      // 隐藏图列数（x）
const int kN = kH * kW;
const int kM = 90;     // 测量数
const int kK = kN;     // 秩 R = N，使 B 满列秩、最小二乘解唯一

struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed ? seed : 1) {}
    uint64_t next() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return s;
    }
    double unit() { return (double)(next() >> 11) / 9007199254740992.0; }
};

// 对 A（rows x cols，行主序）的列做改进 Gram-Schmidt，使列正交。
void orthonormalize_columns(std::vector<double>& A, int rows, int cols) {
    for (int c = 0; c < cols; ++c) {
        for (int d = 0; d < c; ++d) {
            double dot = 0.0;
            for (int r = 0; r < rows; ++r) {
                dot += A[(std::size_t)r * cols + c] * A[(std::size_t)r * cols + d];
            }
            for (int r = 0; r < rows; ++r) {
                A[(std::size_t)r * cols + c] -= dot * A[(std::size_t)r * cols + d];
            }
        }
        double nrm = 0.0;
        for (int r = 0; r < rows; ++r) {
            nrm += A[(std::size_t)r * cols + c] * A[(std::size_t)r * cols + c];
        }
        nrm = std::sqrt(nrm);
        for (int r = 0; r < rows; ++r) A[(std::size_t)r * cols + c] /= nrm;
    }
}

double sqsum(const std::vector<double>& v) {
    double s = 0.0;
    for (std::size_t i = 0; i < v.size(); ++i) s += v[i] * v[i];
    return s;
}

double maxabsdiff(const std::vector<double>& a, const std::vector<double>& b) {
    double m = 0.0;
    for (std::size_t i = 0; i < a.size() && i < b.size(); ++i) {
        const double d = a[i] - b[i];
        if ((d < 0 ? -d : d) > m) m = (d < 0 ? -d : d);
    }
    return m;
}

}  // namespace

int main() {
    std::printf("=== pdhg_core tb（方案 §6.5 固定迭代原始-对偶 PDHG + TV）===\n\n");

    const ps::PdhgShapeFp shape = {kH, kW};
    std::vector<double> f_true((std::size_t)kN), d((std::size_t)kM);
    std::vector<double> Umat((std::size_t)kM * kK), Vmat((std::size_t)kN * kK);
    std::vector<double> Sval((std::size_t)kK);

    Rng rng(0xDEADBEEF12345678ull);
    for (int m = 0; m < kM; ++m)
        for (int k = 0; k < kK; ++k)
            Umat[(std::size_t)m * kK + k] = rng.unit() - 0.5;
    for (int n = 0; n < kN; ++n)
        for (int k = 0; k < kK; ++k)
            Vmat[(std::size_t)n * kK + k] = rng.unit() - 0.5;
    orthonormalize_columns(Umat, kM, kK);   // U 列正交
    orthonormalize_columns(Vmat, kN, kK);   // V 列正交（N x N 时即为正交矩阵）
    for (int k = 0; k < kK; ++k) Sval[(std::size_t)k] = 1.0 - 0.5 * k / (kK - 1);
    for (int n = 0; n < kN; ++n) f_true[(std::size_t)n] = 0.2 + 0.8 * rng.unit();

    // rank-major 因子：U[kM*k+m]、V[kN*k+n]；S 收进 U 侧以保持 rank-major 布局
    std::vector<double> Urm((std::size_t)kK * kM), Vrm((std::size_t)kK * kN),
        Sones((std::size_t)kK, 1.0);
    for (int k = 0; k < kK; ++k) {
        for (int m = 0; m < kM; ++m) {
            Urm[(std::size_t)k * kM + m] = Umat[(std::size_t)m * kK + k] * Sval[(std::size_t)k];
        }
        for (int n = 0; n < kN; ++n) {
            Vrm[(std::size_t)k * kN + n] = Vmat[(std::size_t)n * kK + k];
        }
    }
    ps::LrFactorsFp F;
    F.U = Urm.data();
    F.S = Sones.data();
    F.V = Vrm.data();
    F.K = kK;
    F.M = kM;
    F.N = kN;

    if (ps::lr_forward<16>(F, f_true.data(), d.data()) != ps::kLrOkFp) {
        std::printf("[FAIL] lr_forward 失败\n");
        return 1;
    }

    int fails = 0;

    // ------------------------------------------------------------ 幂迭代估 ||B||₂
    double lam_pow = 0.0;
    {
        std::vector<double> v((std::size_t)kN, 1.0 / std::sqrt((double)kN));
        std::vector<double> t((std::size_t)kM), w((std::size_t)kN);
        for (int it = 0; it < 300; ++it) {
            ps::lr_forward<16>(F, v.data(), t.data());
            ps::lr_adjoint<16>(F, t.data(), w.data());
            const double nw = std::sqrt(sqsum(w));
            if (!(nw > 0.0)) break;
            for (int n = 0; n < kN; ++n) v[(std::size_t)n] = w[(std::size_t)n] / nw;
            lam_pow = nw;
        }
    }
    const double norm_B = std::sqrt(lam_pow);
    double tau = 0.0, sigma = 0.0;
    ps::pdhg_step_sizes(norm_B, std::sqrt(8.0), 0.99, &tau, &sigma);
    std::printf("--- 问题规模与步长标定（§6.5 第 1 条）---\n");
    std::printf("  shape = %d x %d (N=%d)  M=%d  R=%d（满列秩，最小二乘解唯一）\n",
                kH, kW, kN, kM, kK);
    std::printf("  幂迭代 ||B||₂ = %.10f（解析上界 max S = %.2f）\n", norm_B,
                Sval[0]);
    std::printf("  τ = σ = sqrt(0.99)/sqrt(||B||²+||∇||²) = %.10f\n", tau);
    std::printf("  ||d|| = %.10f，  f_true ∈ [%.2f, %.2f]\n", std::sqrt(sqsum(d)),
                0.2, 1.0);

    // ------------------------------------------------------------ C1 TV 原语
    std::printf("\n--- C1 TV 三个原语（与 recon.hpp 同式）---\n");
    {
        std::vector<double> f((std::size_t)kN), g((std::size_t)(2 * kN));
        for (int n = 0; n < kN; ++n) f[(std::size_t)n] = rng.unit() - 0.5;
        ps::pdhg_gradient_fp(f.data(), shape, g.data());
        double gd = 0.0;
        for (int z = 0; z < kH; ++z) {
            for (int x = 0; x < kW; ++x) {
                const std::size_t i = (std::size_t)z * kW + x;
                const double rz = (z + 1 < kH)
                    ? f[(std::size_t)(z + 1) * kW + x] - f[i] : 0.0;
                const double rx = (x + 1 < kW) ? f[i + 1] - f[i] : 0.0;
                double dd = g[i] - rz;
                if ((dd < 0 ? -dd : dd) > gd) gd = (dd < 0 ? -dd : dd);
                dd = g[kN + i] - rx;
                if ((dd < 0 ? -dd : dd) > gd) gd = (dd < 0 ? -dd : dd);
            }
        }
        std::printf("  梯度 vs 朴素前向差分（Neumann 边界）：max|Δ| = %.3e\n", gd);

        std::vector<double> q((std::size_t)(2 * kN)), divq((std::size_t)kN);
        for (int i = 0; i < 2 * kN; ++i) q[(std::size_t)i] = rng.unit() - 0.5;
        ps::pdhg_divergence_fp(q.data(), shape, divq.data());
        double lhs = 0.0, scale = 0.0;
        for (int i = 0; i < kN; ++i) {
            const double t1 = g[i] * q[i] + g[kN + i] * q[kN + i];
            lhs += t1;
            scale += (t1 < 0 ? -t1 : t1);
        }
        double rhs = 0.0;
        for (int i = 0; i < kN; ++i) rhs += f[(std::size_t)i] * divq[(std::size_t)i];
        const double idiff = (lhs + rhs < 0) ? -(lhs + rhs) : (lhs + rhs);
        const double irel = (scale > 0.0) ? idiff / scale : idiff;
        std::printf("  <∇f, q> = %.12e   −<f, div q> = %.12e   相对差 = %.3e\n",
                    lhs, -rhs, irel);

        const double radius = 0.05;
        std::vector<double> q2 = q, qref = q;
        ps::pdhg_project_l2_ball_fp(q2.data(), shape, radius);
        double maxn = 0.0, pd = 0.0;
        for (int i = 0; i < kN; ++i) {
            const double a = qref[(std::size_t)i], b = qref[(std::size_t)(kN + i)];
            const double nrm = std::sqrt(a * a + b * b);
            double ra = a, rb = b;
            if (nrm > radius) {
                ra = a * radius / nrm;
                rb = b * radius / nrm;
            }
            double dd = q2[(std::size_t)i] - ra;
            if ((dd < 0 ? -dd : dd) > pd) pd = (dd < 0 ? -dd : dd);
            dd = q2[(std::size_t)(kN + i)] - rb;
            if ((dd < 0 ? -dd : dd) > pd) pd = (dd < 0 ? -dd : dd);
            const double nn = std::sqrt(q2[(std::size_t)i] * q2[(std::size_t)i]
                                      + q2[(std::size_t)(kN + i)] * q2[(std::size_t)(kN + i)]);
            if (nn > maxn) maxn = nn;
        }
        // 投影与朴素实现只差「乘除结合序」（核是 (radius/nrm)*q，参照是 (q*radius)/nrm），
        // 量级 ~1 ulp，故按 1e-15 判，不用严格 0。
        std::printf("  L2 球投影（radius=%.2f）vs 朴素逐像素：max|Δ| = %.3e（仅结合序，~1 ulp）\n",
                    radius, pd);
        std::printf("  投影后最大逐像素范数 = %.12f（须 <= radius）\n", maxn);
        if (gd == 0.0 && irel <= 1e-12 && pd <= 1e-15
            && maxn <= radius * (1.0 + 1e-15)) {
            std::printf("  [ok  ] 三个 TV 原语自洽（梯度逐值、散度恒等式、投影与半径界）\n");
        } else {
            std::printf("  [FAIL] TV 原语自检未通过\n");
            ++fails;
        }

#ifdef PS_PDHG_XCHECK_RECON
        ps::ReconShape rs;
        rs.n_rows = kH;
        rs.n_cols = kW;
        std::vector<double> g2((std::size_t)(2 * kN)), dv2((std::size_t)kN),
            q3 = qref;
        ps::tv_gradient(f.data(), rs, g2.data());
        ps::tv_divergence(qref.data(), rs, dv2.data());
        ps::tv_project_l2_ball(q3.data(), rs, radius);
        double d1 = 0.0, d2 = 0.0, d3 = 0.0;
        for (int i = 0; i < 2 * kN; ++i) {
            double t = g2[(std::size_t)i] - g[(std::size_t)i];
            if ((t < 0 ? -t : t) > d1) d1 = (t < 0 ? -t : t);
            t = q3[(std::size_t)i] - q2[(std::size_t)i];
            if ((t < 0 ? -t : t) > d3) d3 = (t < 0 ? -t : t);
        }
        for (int i = 0; i < kN; ++i) {
            const double t = dv2[(std::size_t)i] - divq[(std::size_t)i];
            if ((t < 0 ? -t : t) > d2) d2 = (t < 0 ? -t : t);
        }
        std::printf("  [C5] vs recon.hpp：gradient Δ=%.3e  divergence Δ=%.3e  project Δ=%.3e\n",
                    d1, d2, d3);
        if (d1 == 0.0 && d2 == 0.0 && d3 == 0.0) {
            std::printf("  [C5] [ok  ] 本层 TV 原语与 recon.hpp **逐位相同**\n");
        } else {
            std::printf("  [C5] [FAIL] 与 recon.hpp 不一致\n");
            ++fails;
        }
#endif
    }

    // ------------------------------------------------------------ 迭代暂存
    std::vector<double> f((std::size_t)kN, 0.0), f_prev((std::size_t)kN),
        fbar((std::size_t)kN), q((std::size_t)(2 * kN)),
        grad((std::size_t)(2 * kN)), divq((std::size_t)kN), bf((std::size_t)kM),
        r((std::size_t)kM), btr((std::size_t)kN);
    ps::PdhgWorkFp W;
    W.f_prev = f_prev.data();
    W.fbar = fbar.data();
    W.q = q.data();
    W.grad = grad.data();
    W.divq = divq.data();
    W.bf = bf.data();
    W.r = r.data();
    W.btr = btr.data();

    const int kIterMax = 400;
    const int kEvery = 4;
    std::vector<double> hist((std::size_t)(kIterMax / kEvery + 2), 0.0);
    std::vector<double> hist2((std::size_t)(kIterMax / kEvery + 2), 0.0);

    // ------------------------------------------------------------ C2 主判据
    {
        ps::PdhgParamsFp P;
        P.n_iter = kIterMax;
        P.min_iters = 8;
        P.residual_every = kEvery;
        P.tau = tau;
        P.sigma = sigma;
        P.lam = 0.0;            // 纯最小二乘（无 TV）
        P.theta = 0.0;          // 无惯性，投影梯度下降
        P.rho_stop = 1e-8;      // 早停门限

        ps::PdhgOutFp out;
        const int st = ps::pdhg_core<16>(F, d.data(), shape, P, f.data(), W, out,
                                         hist.data(), (int)hist.size());
        std::printf("\n--- C2 无噪合成数据 PDHG（λ=0，θ=0，纯投影梯度下降）---\n");
        std::printf("  返回 %d（0 = kPdhgOkFp）  实际迭代 %d  早停 %d\n", st,
                    out.n_used, out.early_stopped);
        std::printf("  ||d|| = %.8f   ρ_first = %.8e   ρ_last = %.8e\n", out.d_norm,
                    out.rho_first, out.rho);
        std::printf("  残差历史（每 %d 次一个点，共 %d 个）：\n", kEvery, out.n_res);
        for (int i = 0; i < out.n_res; ++i) {
            std::printf("    it=%3d  rho=%.8e\n", (i + 1) * kEvery,
                        hist[(std::size_t)i]);
        }
        double minf = 0.0;
        for (int n = 0; n < kN; ++n) {
            if (f[(std::size_t)n] < minf) minf = f[(std::size_t)n];
        }
        const double ferr = maxabsdiff(f, f_true);
        std::printf("  min f = %.3e（非负）   |f − f_true|max = %.3e\n", minf, ferr);
        if (st != ps::kPdhgOkFp) {
            std::printf("  [FAIL] 核返回错误码\n");
            ++fails;
        }
        if (out.monotone) {
            std::printf("  [ok  ] 归一化残差历史单调不增\n");
        } else {
            std::printf("  [FAIL] 残差历史非单调\n");
            ++fails;
        }
        if (out.rho < 1e-3) {
            std::printf("  [ok  ] 归一化残差下降到 %.3e（< 1e-3）\n", out.rho);
        } else {
            std::printf("  [FAIL] 归一化残差仅到 %.3e（未到 1e-3）\n", out.rho);
            ++fails;
        }
        if (minf >= 0.0) {
            std::printf("  [ok  ] f 满足非负约束\n");
        } else {
            std::printf("  [FAIL] f 出现负值\n");
            ++fails;
        }
        if (ferr <= 1e-4) {
            std::printf("  [ok  ] 解与 f_true 一致（满秩 + 无噪，唯一解）\n");
        } else {
            std::printf("  [FAIL] 解与 f_true 差 %.3e\n", ferr);
            ++fails;
        }
        if (out.early_stopped == 1 && out.n_used >= 8) {
            std::printf("  [ok  ] 在第 %d 次（>= 8）触发早停\n", out.n_used);
        } else {
            std::printf("  [FAIL] 早停未按预期触发\n");
            ++fails;
        }

        // C2b：λ=0 时 θ 不参与（f̄ 只进对偶上升，而 λ=0 -> q ≡ 0）
        std::vector<double> fq((std::size_t)kN, 0.0);
        ps::PdhgOutFp out2;
        ps::PdhgParamsFp P2 = P;
        P2.theta = 1.0;
        ps::pdhg_core<16>(F, d.data(), shape, P2, fq.data(), W, out2, hist2.data(),
                          (int)hist2.size());
        const double dh = maxabsdiff(hist, hist2);
        const double df = maxabsdiff(f, fq);
        std::printf("\n--- C2b λ = 0 时 θ 的作用面（§6.5 的固有性质）---\n");
        std::printf("  θ=0 与 θ=1：|Δ残差历史|max = %.3e   |Δf|max = %.3e\n", dh, df);
        if (dh == 0.0 && df == 0.0) {
            std::printf("  [ok  ] 逐位相同 —— 与「f̄ 只进对偶上升、λ=0 时 q≡0」一致\n");
        } else {
            std::printf("  [FAIL] λ=0 时 θ 竟改变了结果\n");
            ++fails;
        }
    }

    // ------------------------------------------------------------ C3 早停下限
    {
        ps::PdhgParamsFp P;
        P.n_iter = 4;
        P.min_iters = 4;          // 调用方故意调到 4
        P.residual_every = 4;
        P.tau = tau;
        P.sigma = sigma;
        P.lam = 0.0;
        P.theta = 1.0;
        P.rho_stop = 1.0;         // 门限放到 1.0：没有硬下限就会立刻早停
        ps::PdhgOutFp out;
        ps::pdhg_core<16>(F, d.data(), shape, P, f.data(), W, out, hist.data(),
                          (int)hist.size());
        std::printf("\n--- C3 早停下限（§6.5 最小 8 次）---\n");
        std::printf("  n_iter=4，min_iters=4，rho_stop=1.0 -> 早停 %d，迭代 %d，评估 %d 次\n",
                    out.early_stopped, out.n_used, out.n_res);
        if (out.early_stopped == 0 && out.n_used == 4) {
            std::printf("  [ok  ] 第 4 次不得早停（硬下限 8 生效）\n");
        } else {
            std::printf("  [FAIL] 早停下限失效\n");
            ++fails;
        }
    }

    // ------------------------------------------------------------ C4 TV 通路
    {
        const double lam_tv = 0.3;      // 加大 λ，使 TV 项与惯性外推都明显起作用
        double res_theta[2];
        double maxq[2];
        double fmax[2];
        std::vector<double> fsol[2];
        for (int t = 0; t < 2; ++t) {
            std::vector<double> ff((std::size_t)kN, 0.0);
            ps::PdhgParamsFp P;
            P.n_iter = 120;
            P.min_iters = 8;
            P.residual_every = kEvery;
            P.tau = tau;
            P.sigma = sigma;
            P.lam = lam_tv;
            P.theta = (t == 0) ? 0.0 : 1.0;
            P.rho_stop = 0.0;        // 不早停，跑满
            ps::PdhgOutFp out;
            ps::pdhg_core<16>(F, d.data(), shape, P, ff.data(), W, out, hist.data(),
                              (int)hist.size());
            res_theta[t] = out.rho;
            fsol[t] = ff;
            double mx = 0.0, qn = 0.0, mn = 0.0;
            for (int i = 0; i < kN; ++i) {
                const double tt = q[(std::size_t)i] * q[(std::size_t)i]
                                + q[(std::size_t)(kN + i)] * q[(std::size_t)(kN + i)];
                const double nn = std::sqrt(tt);
                if (nn > mx) mx = nn;
                qn += tt;
                if (ff[(std::size_t)i] < mn) mn = ff[(std::size_t)i];
            }
            maxq[t] = mx;
            fmax[t] = mn;
            if (t == 1) {
                std::printf("\n--- C4 λ > 0（各向同性 TV 通路）---\n");
                std::printf("  λ = %.3f，迭代 %d（跑满），ρ_last = %.6e\n", lam_tv,
                            out.n_used, out.rho);
                std::printf("  对偶场：max 逐像素范数 = %.12f（须 <= λ），||q||₂ = %.6e\n",
                            mx, std::sqrt(qn));
                std::printf("  min f = %.3e（非负）\n", mn);
            }
            (void)qn;
        }
        const double df_theta = maxabsdiff(fsol[0], fsol[1]);
        const double dres_theta = (res_theta[0] > 0.0)
            ? (std::fabs(res_theta[0] - res_theta[1]) / res_theta[0]) : 0.0;
        // θ 在 λ > 0 时确实改变轨迹（f̄ 只进对偶上升，λ>0 时对偶非平凡）
        std::printf("  θ=0 与 θ=1：|Δf|max = %.6e，ρ_last 相对差 = %.3e\n",
                    df_theta, dres_theta);
        if (maxq[1] <= lam_tv + 1e-12 && fmax[1] >= 0.0 && df_theta > 0.0) {
            std::printf("  [ok  ] 对偶投影生效（||q|| <= λ）、f 非负、且 θ 确实参与（惯性外推已接线）\n");
        } else {
            std::printf("  [FAIL] TV 通路自检未通过\n");
            ++fails;
        }
    }

    std::printf("\n=== 结论：%s（失败项 %d）===\n",
                fails == 0 ? "全部通过" : "有失败项", fails);
    return fails == 0 ? 0 : 1;
}
