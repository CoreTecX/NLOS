// ============================================================================
// X17 主机侧测试：M5 重建核算子层
//
// 编译（在 src/hardware 目录下）：
//   g++ -O2 -std=c++11 test_x17_recon.cpp -o t_x17
// 运行：
//   ./t_x17
// HWDATA_DIR 默认为 "./hwdata/"，故从 src/hardware 下运行无需额外定义。
//
// 判据
//   T0 几何装配守卫：m2_geometry 复现导出脚本给的矩形 —— 参数接错会让 T5~T7
//      全部静默失效，故先钉这一条（X8 已做过全量 T1，这里只是防接线错误）
//   T1 gradient      与 Python 逐值一致，相对 L2 ≤ 1e-12
//   T2 divergence    同上
//   T3 project_l2_ball 同上
//   T4 负伴随  <grad f, q> == -<f, div q>，C++ 内部自洽 ≤ 1e-12
//   T5 A_d @ f       相对 L2 ≤ 1e-4（X17 判据）
//   T6 A_dᵀ @ r      相对 L2 ≤ 1e-4
//   T7 A_dᵀ(A_d f - yd) 相对 L2 ≤ 1e-4（数据项梯度，复合算子）
//   T8 仅说明（不是检查项）：M5 求解器已由 X15g 定案，本文件不再守「未定案」，
//      数值验证改由 test_x15g_recon.cpp 承担
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "geometry.hpp"
#include "recon.hpp"
#include "transport.hpp"

#ifndef HWDATA_DIR
#define HWDATA_DIR "./hwdata/"
#endif

static int g_pass = 0;
static int g_fail = 0;

// ---------------------------------------------------------------- 读取辅助

static std::vector<uint8_t> slurp(const std::string& name) {
    const std::string p = std::string(HWDATA_DIR) + name;
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) {
        std::printf("  [ERR] 无法打开 %s\n", p.c_str());
        std::exit(2);
    }
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> b((size_t)n);
    if (n > 0 && std::fread(b.data(), 1, (size_t)n, f) != (size_t)n) {
        std::printf("  [ERR] 读取失败 %s\n", p.c_str());
        std::exit(2);
    }
    std::fclose(f);
    return b;
}

template <typename T>
static std::vector<T> load(const std::string& name) {
    std::vector<uint8_t> b = slurp(name);
    std::vector<T> v(b.size() / sizeof(T));
    if (!v.empty()) std::memcpy(v.data(), b.data(), v.size() * sizeof(T));
    return v;
}

static double param_of(const std::string& text, const char* key, double dflt) {
    const std::string k = std::string(key);
    size_t pos = 0;
    while (pos < text.size()) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string::npos) eol = text.size();
        std::string line = text.substr(pos, eol - pos);
        size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        size_t eq = line.find('=');
        if (eq != std::string::npos) {
            std::string lhs = line.substr(0, eq);
            while (!lhs.empty() && (lhs.back() == ' ' || lhs.back() == '\t')) lhs.pop_back();
            while (!lhs.empty() && (lhs.front() == ' ' || lhs.front() == '\t')) lhs.erase(lhs.begin());
            if (lhs == k) return std::atof(line.c_str() + eq + 1);
        }
        pos = eol + 1;
    }
    return dflt;
}

// ---------------------------------------------------------------- 判据辅助

static double rel_l2_d(const double* got, const double* ref, size_t n) {
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double d = got[i] - ref[i];
        num += d * d;
        den += ref[i] * ref[i];
    }
    if (den <= 0.0) return (num <= 0.0) ? 0.0 : 1e30;
    return std::sqrt(num / den);
}

static double max_abs_d(const double* got, const double* ref, size_t n) {
    double m = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double d = std::fabs(got[i] - ref[i]);
        if (d > m) m = d;
    }
    return m;
}

static void judge(const char* tag, double got, double tol, const char* unit) {
    const bool ok = got <= tol;
    std::printf("  [%s] %-34s %.3e %s（阈值 %.0e）\n",
                ok ? "ok  " : "FAIL", tag, got, unit, tol);
    if (ok) ++g_pass; else ++g_fail;
}

// ============================================================================
int main() {
    std::printf("X17 主机侧测试：M5 重建核算子层\n");

    std::vector<uint8_t> pb = slurp("x17_params.txt");
    const std::string txt(pb.begin(), pb.end());

    const int n     = (int)param_of(txt, "wall_n", 126);
    const int H     = (int)param_of(txt, "n_rows", 29);
    const int W     = (int)param_of(txt, "n_cols", 36);
    const int N     = (int)param_of(txt, "n_scene", 1044);
    const std::size_t M  = (std::size_t)n * n;
    const std::size_t Md = (std::size_t)(n - 1) * n;
    const double radius = param_of(txt, "proj_radius", 1.0);

    std::printf("  [info] wall_n=%d  shape=(%d,%d)  n_scene=%d  M=%zu  Md=%zu\n",
                n, H, W, N, M, Md);
    std::printf("  [info] proj_radius=%g  被裁像素 %d/%d\n",
                radius, (int)param_of(txt, "n_clipped", 0),
                (int)param_of(txt, "npix", 0));

    if (H * W != N) { std::printf("  [ERR] shape 与 n_scene 不符\n"); return 2; }
    {
        const std::size_t want = (std::size_t)param_of(txt, "M_wall_diff", 0);
        if (want != Md) { std::printf("  [ERR] M_wall_diff 与 (n-1)*n 不符\n"); return 2; }
    }

    // ---- 读向量 ----
    std::vector<float>  wall_x_f = load<float>("x17_wall_x.bin");
    std::vector<float>  wall_z_f = load<float>("x17_wall_z.bin");
    std::vector<float>  mon_x_f  = load<float>("x17_mon_x.bin");
    std::vector<float>  mon_z_f  = load<float>("x17_mon_z.bin");
    std::vector<int32_t> rect_i  = load<int32_t>("x17_rect.bin");
    std::vector<float>  f_ramp_f = load<float>("x17_f_ramp.bin");
    std::vector<float>  r_rand_f = load<float>("x17_r_rand.bin");
    std::vector<float>  q_rand_f = load<float>("x17_q_rand.bin");
    std::vector<float>  g_in_f   = load<float>("x17_g_in.bin");
    std::vector<double> ref_grad = load<double>("x17_ref_grad.bin");
    std::vector<double> ref_div  = load<double>("x17_ref_div.bin");
    std::vector<double> ref_proj = load<double>("x17_ref_proj.bin");
    std::vector<double> ref_Adf  = load<double>("x17_ref_Adf.bin");
    std::vector<double> ref_AdTr = load<double>("x17_ref_AdTr.bin");
    std::vector<double> ref_dgrd = load<double>("x17_ref_dgrad.bin");

    // 输入统一升为 double：C++ 读到的正是导出脚本算参考值时用的那组 float32 值
    std::vector<double> wall_x(wall_x_f.begin(), wall_x_f.end());
    std::vector<double> wall_z(wall_z_f.begin(), wall_z_f.end());
    std::vector<double> mon_x(mon_x_f.begin(), mon_x_f.end());
    std::vector<double> mon_z(mon_z_f.begin(), mon_z_f.end());
    std::vector<double> f_ramp(f_ramp_f.begin(), f_ramp_f.end());
    std::vector<double> r_rand(r_rand_f.begin(), r_rand_f.end());
    std::vector<double> q_rand(q_rand_f.begin(), q_rand_f.end());
    std::vector<double> g_in(g_in_f.begin(), g_in_f.end());

    ps::ReconShape shape;
    shape.n_rows = H;
    shape.n_cols = W;

    // ================================================================
    // T0 几何装配守卫
    // ================================================================
    std::printf("\n=== T0 几何装配守卫（%d 个隐藏点）===\n", N);
    {
        double ll[3] = {
            param_of(txt, "truth_ll", 0.0),
            0.0, 0.0
        };
        // truth_ll 是一行三个数，param_of 只取第一个；单独解析全量
        {
            const std::string key = "truth_ll";
            size_t p = txt.find(key);
            if (p != std::string::npos) {
                const char* c = txt.c_str() + p + key.size();
                const char* q = std::strchr(c, '=');
                if (q) std::sscanf(q + 1, "%lf %lf %lf", &ll[0], &ll[1], &ll[2]);
            }
        }

        ps::GeomParams g;
        g.wall_y      = param_of(txt, "wall_y", 0);
        g.wall_x_min  = param_of(txt, "wall_x_min", 0);
        g.wall_x_max  = param_of(txt, "wall_x_max", 0);
        g.wall_z_min  = param_of(txt, "wall_z_min", 0);
        g.wall_z_max  = param_of(txt, "wall_z_max", 0);
        g.occ_size_dx = param_of(txt, "occ_size_dx", 0);
        g.occ_size_dz = param_of(txt, "occ_size_dz", 0);
        g.wall_n      = n;

        std::printf("  [info] truth_ll = (%.6f, %.6f, %.6f)  t=%.6f\n",
                    ll[0], ll[1], ll[2], ll[1] / g.wall_y);

        std::vector<ps::OccRect> rect((size_t)N);
        ps::m2_geometry(ll, g, mon_x_f.data(), mon_z_f.data(), rect.data(), N);

        int mismatch = 0, first_i = -1;
        for (int i = 0; i < N; ++i) {
            const int32_t* rr = &rect_i[(size_t)i * 4];
            if (rect[i].x_lo != rr[0] || rect[i].x_hi != rr[1] ||
                rect[i].z_lo != rr[2] || rect[i].z_hi != rr[3]) {
                if (mismatch == 0) first_i = i;
                ++mismatch;
            }
        }
        const bool ok = (mismatch == 0);
        if (ok) {
            std::printf("  [ok  ] %-34s 全部一致（%d 个隐藏点）\n",
                        "矩形与导出脚本逐值相等", N);
            ++g_pass;
        } else {
            std::printf("  [FAIL] %-34s 不一致 %d / %d，首个在 i=%d\n",
                        "矩形与导出脚本逐值相等", mismatch, N, first_i);
            ++g_fail;
        }
    }

    // 续用导出脚本给的矩形（M2 已在 T0 与 X8 中验过）
    std::vector<ps::OccRect> rect((size_t)N);
    for (int i = 0; i < N; ++i) {
        const int32_t* rr = &rect_i[(size_t)i * 4];
        rect[i].x_lo = (int16_t)rr[0];
        rect[i].x_hi = (int16_t)rr[1];
        rect[i].z_lo = (int16_t)rr[2];
        rect[i].z_hi = (int16_t)rr[3];
    }

    // ================================================================
    // T1 / T2 / T3  TV 微分算子与投影
    // ================================================================
    std::printf("\n=== T1~T3 TV 微分算子与对偶投影 ===\n");
    {
        std::vector<double> got(2 * (size_t)N);
        ps::tv_gradient(f_ramp.data(), shape, got.data());
        const double e1 = rel_l2_d(got.data(), ref_grad.data(), 2 * (size_t)N);
        std::printf("        最大绝对差 %.3e\n",
                    max_abs_d(got.data(), ref_grad.data(), 2 * (size_t)N));
        judge("tv_gradient 相对 L2 误差", e1, 1e-12, "");

        ps::tv_divergence(g_in.data(), shape, got.data());
        const double e2 = rel_l2_d(got.data(), ref_div.data(), (size_t)N);
        std::printf("        最大绝对差 %.3e\n",
                    max_abs_d(got.data(), ref_div.data(), (size_t)N));
        judge("tv_divergence 相对 L2 误差", e2, 1e-12, "");

        std::vector<double> qc(q_rand);
        ps::tv_project_l2_ball(qc.data(), shape, radius);
        const double e3 = rel_l2_d(qc.data(), ref_proj.data(), 2 * (size_t)N);
        std::printf("        最大绝对差 %.3e\n",
                    max_abs_d(qc.data(), ref_proj.data(), 2 * (size_t)N));
        judge("tv_project_l2_ball 相对 L2 误差", e3, 1e-12, "");
    }

    // ================================================================
    // T4 负伴随 <grad f, q> == -<f, div q>
    // ================================================================
    std::printf("\n=== T4 负伴随 <grad f, q> == -<f, div q> ===\n");
    {
        std::vector<double> gq(2 * (size_t)N), divq((size_t)N);
        ps::tv_gradient(f_ramp.data(), shape, gq.data());
        ps::tv_divergence(q_rand.data(), shape, divq.data());

        double lhs = 0.0, rhs = 0.0;
        for (std::size_t i = 0; i < 2 * (size_t)N; ++i) lhs += gq[i] * q_rand[i];
        for (int i = 0; i < N; ++i) rhs -= f_ramp[i] * divq[i];

        const double sc = std::fmax(std::fabs(lhs), std::fabs(rhs));
        const double rel = (sc > 0.0) ? std::fabs(lhs - rhs) / sc : 0.0;
        std::printf("        <grad f, q>   = %.10e\n", lhs);
        std::printf("        -<f, div q>   = %.10e\n", rhs);
        const bool ok = (rel <= 1e-12);
        std::printf("  [%s] %-34s %.3e（阈值 1e-12）\n",
                    ok ? "ok  " : "FAIL", "相对偏差", rel);
        if (ok) ++g_pass; else ++g_fail;

        std::vector<double> gw(2 * (size_t)N);
        const double tv = ps::tv_isotropic_norm(f_ramp.data(), shape, gw.data());
        std::printf("  [info] TV(f_ramp) = %.10e\n", tv);
    }

    // ================================================================
    // T5 / T6 / T7  重建核算子
    // ================================================================
    std::printf("\n=== T5~T7 重建核算子（A_d 图像布局，%zu x %d）===\n", Md, N);
    {
        ps::TransportParams tp;
        ps::transport_configure(param_of(txt, "wall_y", 0),
                                param_of(txt, "monitor_y", 0),
                                param_of(txt, "monitor_block_area", 0), n, tp);

        ps::M5Geometry G;
        G.n_scene = N;
        G.rect    = rect.data();
        G.wall_x  = wall_x.data();
        G.wall_z  = wall_z.data();
        G.mon_x   = mon_x.data();
        G.mon_z   = mon_z.data();
        G.tp      = tp;

        std::vector<double> row_prev((size_t)n), row_cur((size_t)n);
        std::vector<double> col(Md), tmp(Md);
        ps::M5Scratch S;
        S.row_prev = row_prev.data();
        S.row_cur  = row_cur.data();
        S.col      = col.data();
        S.tmp      = tmp.data();

        // T5 y = A_d f
        std::vector<double> y(Md);
        ps::m5_apply_Ad(G, f_ramp.data(), y.data(), S);
        const double e5 = rel_l2_d(y.data(), ref_Adf.data(), Md);
        double ny = 0.0, nr = 0.0;
        for (std::size_t k = 0; k < Md; ++k) {
            ny += y[k] * y[k];
            nr += ref_Adf[k] * ref_Adf[k];
        }
        std::printf("        A_d@f   |y| = %.6e   |ref| = %.6e\n",
                    std::sqrt(ny), std::sqrt(nr));
        judge("A_d @ f 相对 L2 误差", e5, 1e-4, "");

        // T6 z = A_d^T r
        std::vector<double> z((size_t)N);
        ps::m5_apply_AdT(G, r_rand.data(), z.data(), S);
        const double e6 = rel_l2_d(z.data(), ref_AdTr.data(), (size_t)N);
        judge("A_d^T @ r 相对 L2 误差", e6, 1e-4, "");

        // T7 数据项梯度
        std::vector<double> dg((size_t)N);
        ps::m5_data_term_grad(G, f_ramp.data(), r_rand.data(), dg.data(), S);
        const double e7 = rel_l2_d(dg.data(), ref_dgrd.data(), (size_t)N);
        judge("A_d^T (A_d f - yd) 相对 L2 误差", e7, 1e-4, "");

        // 伴随一致性（C++ 内部，用同一 A_d 的两个方向）
        {
            double lhs = 0.0, rhs = 0.0;
            for (std::size_t k = 0; k < Md; ++k) lhs += y[k] * r_rand[k];
            for (int i = 0; i < N; ++i) rhs += f_ramp[i] * z[i];
            const double sc = std::sqrt(std::fabs(lhs) * std::fabs(rhs));
            const double rel = (sc > 0.0) ? std::fabs(lhs - rhs) / sc : 0.0;
            std::printf("        <A_d f, r> = %.10e   <f, A_d^T r> = %.10e\n", lhs, rhs);
            const bool ok = (rel <= 1e-12);
            std::printf("  [%s] %-34s %.3e（阈值 1e-12）\n",
                        ok ? "ok  " : "FAIL", "A_d 伴随一致性", rel);
            if (ok) ++g_pass; else ++g_fail;
        }

        // ============================================================
        // T8 已迁移（不是检查项，仅说明）
        // ============================================================
        // X15g 定案后，M5 的求解器边界由「吃 A 算子」改为「吃 Gram」（见 recon.hpp），
        // 本文件原先在此处的「求解器未定案守卫」已不适用。M5 的数值验证改由
        // test_x15g_recon.cpp 承担。此处保留一行说明，避免有人以为 M5 还没定案。
        std::printf("\n=== T8 M5 边界迁移说明 ===\n");
        std::printf("  [info] M5 求解器已由 X15g 定案：ADMM + 固定内层 CG（见 recon.hpp）\n");
        std::printf("  [info] M5 的数值验证请跑 test_x15g_recon.cpp\n");
    }

    std::printf("\n%s  PASS %d  FAIL %d\n",
                (g_fail == 0) ? "全部通过。" : "存在失败项。", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
