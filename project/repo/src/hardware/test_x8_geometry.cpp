// ============================================================================
// X8 主机侧测试：M2 几何算核与 M3 算子核
//
// 编译（在 src/hardware 目录下）：
//   g++ -O2 -std=c++11 -DHWDATA_DIR=\"hwdata/\" test_x8_geometry.cpp -o t_x8
// 运行：
//   ./t_x8
//
// 四项检查
//   T1 遮挡矩形：K 个候选 x 全部 1044 个隐藏点，与参考实现导出的矩形逐值比对
//      （空集只比「是否为空」，非空则四个索引必须逐值相等）
//   T2 A 的抽查列：相对 L2 误差 <= 1e-3（X8 判据）
//   T3 A_d 的抽查列：相对 L2 误差 <= 1e-3（X8 判据）
//   T4 伴随一致性：<A_d x, r> == <x, A_dᵀ r>，验证流式算子本身自洽
//      （X17 重建核依赖这一对算子，X8 阶段就要把它钉住）
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "geometry.hpp"
#include "transport.hpp"

#ifndef HWDATA_DIR
#define HWDATA_DIR "./hwdata/"
#endif

static int g_pass = 0;
static int g_fail = 0;

// ---------------------------------------------------------------- 读取辅助

static std::vector<uint8_t> slurp(const std::string& name) {
    std::string p = std::string(HWDATA_DIR) + name;
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

// 解析 x8_params.txt 里的 "key = value"，value 取第一个浮点数
static double param_of(const std::string& text, const char* key, double dflt) {
    const std::string k = std::string(key);
    size_t pos = 0;
    while (pos < text.size()) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string::npos) eol = text.size();
        std::string line = text.substr(pos, eol - pos);
        size_t eq = line.find('=');
        if (eq != std::string::npos) {
            std::string lhs = line.substr(0, eq);
            // 去掉首尾空白
            while (!lhs.empty() && (lhs.back() == ' ' || lhs.back() == '\t')) lhs.pop_back();
            while (!lhs.empty() && (lhs.front() == ' ' || lhs.front() == '\t')) lhs.erase(lhs.begin());
            if (lhs == k) return std::atof(line.c_str() + eq + 1);
        }
        pos = eol + 1;
    }
    return dflt;
}

// ---------------------------------------------------------------- 判据辅助

struct RelErr {
    double worst;      // 最坏相对误差
    int    worst_k;    // 出在最坏处的候选编号
    int    worst_p;    // 出在最坏处的抽查点编号
};

static void report(const char* tag, const RelErr& e, double tol, size_t items) {
    const bool ok = e.worst <= tol;
    std::printf("  [%s] %-30s 最坏相对误差 %.3e（阈值 %.0e，%zu 组）",
                ok ? "ok  " : "FAIL", tag, e.worst, tol, items);
    if (!ok) std::printf("  出在 候选 %d / 抽查点 %d", e.worst_k, e.worst_p);
    std::printf("\n");
    if (ok) ++g_pass; else ++g_fail;
}

static double rel_l2(const double* got, const float* ref, size_t n) {
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double d = got[i] - (double)ref[i];
        num += d * d;
        den += (double)ref[i] * (double)ref[i];
    }
    if (den <= 0.0) return (num <= 0.0) ? 0.0 : 1e30;
    return std::sqrt(num / den);
}

// ============================================================================
int main() {
    std::printf("X8 主机侧测试：M2 几何算核 + M3 算子核\n");

    // ---- 读参数 ----
    {
        std::vector<uint8_t> b = slurp("x8_params.txt");
        const std::string txt(b.begin(), b.end());
        const int n = (int)param_of(txt, "wall_n", 126);
        const int N = (int)param_of(txt, "n_scene", 1044);
        const int K = (int)param_of(txt, "n_cand", 3);
        const int P = (int)param_of(txt, "n_probe", 8);
        std::printf("  [info] wall_n=%d n_scene=%d n_cand=%d n_probe=%d\n", n, N, K, P);
        std::printf("  [info] wall_y=%g occ_size=(%g, %g) block_area=%g\n",
                    param_of(txt, "wall_y", 0), param_of(txt, "occ_size_dx", 0),
                    param_of(txt, "occ_size_dz", 0), param_of(txt, "monitor_block_area", 0));

        // ---- 读向量 ----
        std::vector<float>  wall_x_f = load<float>("x8_wall_x.bin");
        std::vector<float>  wall_z_f = load<float>("x8_wall_z.bin");
        std::vector<float>  mon_x_f  = load<float>("x8_mon_x.bin");
        std::vector<float>  mon_z_f  = load<float>("x8_mon_z.bin");
        std::vector<float>  cand_ll  = load<float>("x8_cand_ll.bin");
        std::vector<int32_t> probe   = load<int32_t>("x8_probe_idx.bin");
        std::vector<int32_t> ref_rect = load<int32_t>("x8_ref_rect.bin");
        std::vector<float>  ref_A    = load<float>("x8_ref_A.bin");
        std::vector<float>  ref_Ad   = load<float>("x8_ref_Ad.bin");

        std::vector<double> wall_x(wall_x_f.begin(), wall_x_f.end());
        std::vector<double> wall_z(wall_z_f.begin(), wall_z_f.end());
        std::vector<double> mon_x(mon_x_f.begin(), mon_x_f.end());
        std::vector<double> mon_z(mon_z_f.begin(), mon_z_f.end());

        const std::size_t M  = (std::size_t)n * n;
        const std::size_t Md = (std::size_t)(n - 1) * n;

        // ---- 参数装配 ----
        ps::GeomParams g;
        g.wall_y     = param_of(txt, "wall_y", 0);
        g.wall_x_min = param_of(txt, "wall_x_min", 0);
        g.wall_x_max = param_of(txt, "wall_x_max", 0);
        g.wall_z_min = param_of(txt, "wall_z_min", 0);
        g.wall_z_max = param_of(txt, "wall_z_max", 0);
        g.occ_size_dx = param_of(txt, "occ_size_dx", 0);
        g.occ_size_dz = param_of(txt, "occ_size_dz", 0);
        g.wall_n = n;

        ps::TransportParams tp;
        ps::transport_configure(param_of(txt, "wall_y", 0),
                                param_of(txt, "monitor_y", 0),
                                param_of(txt, "monitor_block_area", 0), n, tp);

        std::vector<ps::OccRect> rect((std::size_t)N);
        std::vector<double> col(M), col_d(Md);
        std::vector<double> row_prev(n), row_cur(n);

        // ================================================================
        // T1 遮挡矩形：K 个候选 x 全部 1044 个隐藏点
        // ================================================================
        std::printf("\n=== T1 遮挡矩形（%d 候选 x %d 隐藏点 = %d 组）===\n",
                    K, N, K * N);
        {
            int mismatch = 0, n_empty = 0, n_nonempty = 0;
            int first_bad_k = -1, first_bad_i = -1;
            for (int k = 0; k < K; ++k) {
                double ll[3] = { (double)cand_ll[k * 3 + 0],
                                 (double)cand_ll[k * 3 + 1],
                                 (double)cand_ll[k * 3 + 2] };
                ps::m2_geometry(ll, g, mon_x_f.data(), mon_z_f.data(),
                                rect.data(), N);
                for (int i = 0; i < N; ++i) {
                    const int32_t* rr = &ref_rect[((std::size_t)k * N + i) * 4];
                    const bool ref_empty = (rr[0] > rr[1]) || (rr[2] > rr[3]);
                    const bool got_empty = ps::rect_empty(rect[i]);
                    if (ref_empty || got_empty) {
                        if (ref_empty != got_empty) {
                            if (mismatch == 0) { first_bad_k = k; first_bad_i = i; }
                            ++mismatch;
                        }
                        if (ref_empty) ++n_empty; else ++n_nonempty;
                        continue;
                    }
                    ++n_nonempty;
                    if (rect[i].x_lo != rr[0] || rect[i].x_hi != rr[1] ||
                        rect[i].z_lo != rr[2] || rect[i].z_hi != rr[3]) {
                        if (mismatch == 0) { first_bad_k = k; first_bad_i = i; }
                        ++mismatch;
                        if (mismatch <= 5) {
                            std::printf("        k=%d i=%d got (%d,%d,%d,%d) ref (%d,%d,%d,%d)\n",
                                        k, i, (int)rect[i].x_lo, (int)rect[i].x_hi,
                                        (int)rect[i].z_lo, (int)rect[i].z_hi,
                                        rr[0], rr[1], rr[2], rr[3]);
                        }
                    }
                }
                // 逐候选统计空集比例，便于发现「t 越小空集越多」这一规律
                int e = 0;
                for (int i = 0; i < N; ++i) if (ps::rect_empty(rect[i])) ++e;
                const double t = (double)cand_ll[k * 3 + 1] / g.wall_y;
                std::printf("        候选 %d: y=%.4f t=%.6f  空集 %d/%d (%.1f%%)\n",
                            k, (double)cand_ll[k * 3 + 1], t, e, N,
                            100.0 * e / N);
            }
            if (mismatch == 0) {
                std::printf("  [ok  ] %-30s %d 组全部一致（空集 %d，非空 %d）\n",
                            "遮挡矩形逐值相等", K * N, n_empty, n_nonempty);
                ++g_pass;
            } else {
                std::printf("  [FAIL] %-30s %d 组不一致，首个在 候选 %d / 隐藏点 %d\n",
                            "遮挡矩形逐值相等", mismatch, first_bad_k, first_bad_i);
                ++g_fail;
            }
        }

        // ================================================================
        // T2 / T3 A 与 A_d 的抽查列
        // ================================================================
        std::printf("\n=== T2/T3 A 与 A_d 抽查列（%d 候选 x %d 抽查点）===\n", K, P);
        {
            RelErr eA{0.0, -1, -1}, eAd{0.0, -1, -1};
            for (int k = 0; k < K; ++k) {
                double ll[3] = { (double)cand_ll[k * 3 + 0],
                                 (double)cand_ll[k * 3 + 1],
                                 (double)cand_ll[k * 3 + 2] };
                ps::m2_geometry(ll, g, mon_x_f.data(), mon_z_f.data(),
                                rect.data(), N);
                for (int p = 0; p < P; ++p) {
                    const int i = probe[p];
                    ps::emit_A_column(i, rect[i], wall_x.data(), wall_z.data(),
                                      mon_x.data(), mon_z.data(), tp, col.data());
                    const double rA = rel_l2(col.data(),
                                             &ref_A[((std::size_t)k * P + p) * M], M);
                    if (rA > eA.worst) { eA.worst = rA; eA.worst_k = k; eA.worst_p = p; }

                    ps::emit_Ad_column(i, rect[i], wall_x.data(), wall_z.data(),
                                       mon_x.data(), mon_z.data(), tp, col_d.data(),
                                       row_prev.data(), row_cur.data());
                    const double rD = rel_l2(col_d.data(),
                                             &ref_Ad[((std::size_t)k * P + p) * Md], Md);
                    if (rD > eAd.worst) { eAd.worst = rD; eAd.worst_k = k; eAd.worst_p = p; }
                }
            }
            report("A 抽查列相对 L2 误差", eA, 1e-3, (size_t)K * P);
            report("A_d 抽查列相对 L2 误差", eAd, 1e-3, (size_t)K * P);
        }

        // ================================================================
        // T4 伴随一致性 <A_d x, r> == <x, A_dᵀ r>
        // 用真值候选，随机 x 与 r（固定种子，可复现）
        // ================================================================
        std::printf("\n=== T4 伴随一致性 <A_d x, r> == <x, A_d^T r> ===\n");
        {
            double ll[3] = { (double)cand_ll[0], (double)cand_ll[1],
                             (double)cand_ll[2] };
            ps::m2_geometry(ll, g, mon_x_f.data(), mon_z_f.data(), rect.data(), N);

            std::vector<double> x(N), r(Md), y(Md), z(N);
            uint32_t seed = 12345u;
            for (int i = 0; i < N; ++i) {
                seed = seed * 1664525u + 1013904223u;
                x[i] = ((double)(seed >> 8) / (double)(1u << 24)) - 0.5;
            }
            for (std::size_t k = 0; k < Md; ++k) {
                seed = seed * 1664525u + 1013904223u;
                r[k] = ((double)(seed >> 8) / (double)(1u << 24)) - 0.5;
            }

            ps::apply_Ad(N, rect.data(), wall_x.data(), wall_z.data(),
                         mon_x.data(), mon_z.data(), tp, x.data(), y.data(),
                         row_prev.data(), row_cur.data(), col_d.data());
            ps::apply_AdT(N, rect.data(), wall_x.data(), wall_z.data(),
                          mon_x.data(), mon_z.data(), tp, r.data(), z.data(),
                          row_prev.data(), row_cur.data(), col_d.data());

            double lhs = 0.0, rhs = 0.0, ny = 0.0, nz = 0.0;
            for (std::size_t k = 0; k < Md; ++k) { lhs += y[k] * r[k]; ny += y[k] * y[k]; }
            for (int i = 0; i < N; ++i) { rhs += x[i] * z[i]; nz += z[i] * z[i]; }

            const double scale = std::sqrt(ny) * std::sqrt(nz);
            const double rel = (scale > 0.0) ? std::fabs(lhs - rhs) / scale : 0.0;
            std::printf("        <A_d x, r> = %.10e\n", lhs);
            std::printf("        <x, A_d^T r> = %.10e\n", rhs);
            std::printf("        |A_d x| = %.6e   |A_d^T r| = %.6e\n",
                        std::sqrt(ny), std::sqrt(nz));
            const bool ok = rel <= 1e-12;
            std::printf("  [%s] %-30s 归一化偏差 %.3e（阈值 1e-12）\n",
                        ok ? "ok  " : "FAIL", "伴随一致性", rel);
            if (ok) ++g_pass; else ++g_fail;
        }
    }

    std::printf("\n%s  PASS %d  FAIL %d\n",
                (g_fail == 0) ? "全部通过。" : "存在失败项。", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
