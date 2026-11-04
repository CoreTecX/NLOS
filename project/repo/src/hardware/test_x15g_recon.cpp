// ============================================================================
// X15g 主机侧测试：M5 重建核的 ADMM 内核（recon.hpp）
//
// 编译（在 src/hardware 目录下）：
//   g++ -O2 -std=c++11 test_x15g_recon.cpp -o t_x15g
// 运行：
//   ./t_x15g
//
// 判据
//   T1 上三角偏移约定：tri_offset 与导出脚本的打包顺序逐格一致
//   T2 内核跑通：status == kReconOk，n_rec == n_outer
//   T3 与 Python 参考解一致：相对 L2 ≤ 1e-4（X17/X18 判据口径）
//   T4 结构指标对得上：与真值的标度不变误差 sm_err 落在 1e-3 量级
//      （X15g S2 实测 δ=0 时 2.49e-03；这一条防「数值接近但落在错误不动点」）
//   T5 入参守卫：空 Gram、rho <= 0 必须被拒
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "recon.hpp"

#ifndef HWDATA_DIR
#define HWDATA_DIR "./hwdata/"
#endif

static int g_pass = 0;
static int g_fail = 0;

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

static void judge(const char* tag, double got, double tol, bool le) {
    const bool ok = le ? (got <= tol) : (got >= tol);
    std::printf("  [%s] %-36s %.6e（阈值 %s %.0e）\n",
                ok ? "ok  " : "FAIL", tag, got, le ? "<=" : ">=", tol);
    if (ok) ++g_pass; else ++g_fail;
}

static double rel_l2(const double* a, const double* b, size_t n) {
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double d = a[i] - b[i];
        num += d * d;
        den += b[i] * b[i];
    }
    return (den > 0.0) ? std::sqrt(num / den) : 0.0;
}

// 标度不变误差：与 X15g 报告的 sm_err 同一口径。
static double sm_err(const double* u, const double* ref, size_t n) {
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; ++i) { num += u[i] * ref[i]; den += u[i] * u[i]; }
    if (den <= 0.0) return 1e30;
    const double a = num / den;
    double nr = 0.0, d2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double e = a * u[i] - ref[i];
        d2 += e * e;
        nr += ref[i] * ref[i];
    }
    return std::sqrt(d2 / nr);
}

int main(int argc, char** argv) {
    std::printf("X15g 主机侧测试：M5 重建核 ADMM 内核\n");

    std::vector<uint8_t> pb = slurp("x15g_params.txt");
    const std::string txt(pb.begin(), pb.end());

    const int n = (int)param_of(txt, "n_scene", 1044);
    const int H = (int)param_of(txt, "n_rows", 29);
    const int W = (int)param_of(txt, "n_cols", 36);
    const std::size_t tri_len = (std::size_t)param_of(txt, "tri_len", 0);
    const double lam = param_of(txt, "lam", 0);
    const double rho = param_of(txt, "rho", 0);
    const int n_outer = (int)param_of(txt, "n_outer", 400);
    const int n_cg = (int)param_of(txt, "n_cg", 15);

    std::printf("  [info] n=%d  shape=(%d,%d)  tri_len=%zu\n", n, H, W, tri_len);
    std::printf("  [info] λ=%g  ρ=%g  th=%g  外层=%d 内层CG=%d\n",
                lam, rho, lam / rho, n_outer, n_cg);

    if ((std::size_t)n * (n + 1) / 2 != tri_len) {
        std::printf("  [ERR] tri_len 与 n(n+1)/2 不符\n");
        return 2;
    }

    std::vector<double> tri = load<double>("x15g_ata_tri.bin");
    std::vector<double> aty = load<double>("x15g_aty.bin");
    std::vector<double> ref_u = load<double>("x15g_ref_u.bin");
    std::vector<double> u_true = load<double>("x15g_ref_utrue.bin");

    if (tri.size() != tri_len || aty.size() != (size_t)n || ref_u.size() != (size_t)n) {
        std::printf("  [ERR] 向量长度不符\n");
        return 2;
    }

    // ================================================================
    // T1 上三角偏移约定
    // ================================================================
    std::printf("\n=== T1 上三角偏移约定 ===\n");
    {
        bool ok = true;
        int bad_i = -1, bad_j = -1;
        std::size_t expect = 0;
        for (int i = 0; i < n && ok; ++i) {
            for (int j = i; j < n && ok; ++j) {
                if (ps::tri_offset(i, j, n) != expect) {
                    ok = false; bad_i = i; bad_j = j;
                }
                ++expect;
            }
        }
        // 对称性
        if (ok && ps::tri_offset(3, 900, n) != ps::tri_offset(900, 3, n)) ok = false;
        // 末格
        if (ok && ps::tri_offset(n - 1, n - 1, n) != tri_len - 1) ok = false;
        if (ok) {
            std::printf("  [ok  ] %-36s %zu 格全部对上，末格偏移 %zu\n",
                        "tri_offset 与行优先上三角一致", tri_len, tri_len - 1);
            ++g_pass;
        } else {
            std::printf("  [FAIL] %-36s 首个不符 (%d,%d)\n",
                        "tri_offset 与行优先上三角一致", bad_i, bad_j);
            ++g_fail;
        }
    }

    ps::ReconShape shape;
    shape.n_rows = H;
    shape.n_cols = W;
    if (ps::recon_n(shape) != n) {
        std::printf("  [ERR] shape 与 n_scene 不符\n");
        return 2;
    }

    // ================================================================
    // T2~T4 跑内核
    // ================================================================
    std::printf("\n=== T2~T4 ADMM 内核 ===\n");
    ps::M5Gram G;
    G.n_scene = n;
    G.ata_tri = tri.data();
    G.aty = aty.data();

    std::vector<double> u((size_t)n);
    std::vector<double> rhs((size_t)n), cgx((size_t)n), cgr((size_t)n);
    std::vector<double> cgp((size_t)n), cgap((size_t)n);
    std::vector<double> d(2 * (size_t)n), q(2 * (size_t)n);
    std::vector<double> g2(2 * (size_t)n), s2(2 * (size_t)n);
    ps::M5Work Wk;
    Wk.u = u.data(); Wk.rhs = rhs.data();
    Wk.cg_x = cgx.data(); Wk.cg_r = cgr.data();
    Wk.cg_p = cgp.data(); Wk.cg_ap = cgap.data();
    Wk.d = d.data(); Wk.q = q.data();
    Wk.g = g2.data(); Wk.s = s2.data();

    ps::ReconParams P;
    P.lam = lam; P.rho = rho; P.n_outer = n_outer; P.n_cg = n_cg;

    ps::ReconOut O;
    O.u = u.data(); O.n_rec = 0; O.status = -1;

    const int st = ps::m5_reconstruct(G, shape, P, Wk, O);
    const bool ok_status = (st == ps::kReconOk);
    std::printf("  [%s] %-36s status=%d n_rec=%d\n",
                ok_status ? "ok  " : "FAIL", "内核跑通", st, O.n_rec);
    if (ok_status) ++g_pass; else ++g_fail;

    if (ok_status) {
        judge("与 Python 参考解相对 L2", rel_l2(u.data(), ref_u.data(), n), 1e-4, true);

        const double e_cpp = sm_err(u.data(), u_true.data(), n);
        const double e_py = sm_err(ref_u.data(), u_true.data(), n);
        std::printf("        sm_err(C++) = %.6e   sm_err(Python) = %.6e\n", e_cpp, e_py);
        // 两者应同量级；用 Python 值的 10 倍作上界，防「数值接近但落到错误不动点」
        judge("sm_err 与 Python 同量级（<=10 倍）", e_cpp, 10.0 * e_py, true);
    }

    // ================================================================
    // T5 入参守卫
    // ================================================================
    std::printf("\n=== T5 入参守卫 ===\n");
    {
        ps::ReconOut O2; O2.u = u.data(); O2.n_rec = 0; O2.status = -1;
        ps::M5Gram bad = G; bad.ata_tri = NULL;
        const int st2 = ps::m5_reconstruct(bad, shape, P, Wk, O2);
        const bool ok2 = (st2 == ps::kReconBadArgs);
        std::printf("  [%s] %-36s status=%d\n", ok2 ? "ok  " : "FAIL",
                    "空 Gram 被拒", st2);
        if (ok2) ++g_pass; else ++g_fail;

        ps::ReconParams P3 = P; P3.rho = 0.0;
        ps::ReconOut O3; O3.u = u.data(); O3.n_rec = 0; O3.status = -1;
        const int st3 = ps::m5_reconstruct(G, shape, P3, Wk, O3);
        const bool ok3 = (st3 == ps::kReconBadArgs);
        std::printf("  [%s] %-36s status=%d\n", ok3 ? "ok  " : "FAIL",
                    "rho <= 0 被拒", st3);
        if (ok3) ++g_pass; else ++g_fail;

        const double rho_def = ps::m5_default_rho(G);
        std::printf("  [info] m5_default_rho = %.6e（导出脚本用的 ρ = %.6e）\n",
                    rho_def, rho);
        judge("默认 ρ 与定案值一致（相对差 <=1e-12）",
              std::fabs(rho_def - rho) / rho, 1e-12, true);
    }

    // 可选：把**内核实际输出** dump 成二进制，供 X13 出重建图。
    // 用相对路径（窄字符 fopen 走 ANSI 码页，绝对路径里的中文会乱码）。
    if (argc > 1 && ok_status) {
        FILE* fo = std::fopen(argv[1], "wb");
        if (fo) {
            std::fwrite(u.data(), sizeof(double), (size_t)n, fo);
            std::fclose(fo);
            std::printf("  [info] 已 dump 内核输出 %d 个 double → %s\n", n, argv[1]);
        }
    }

    std::printf("\n%s  PASS %d  FAIL %d\n",
                (g_fail == 0) ? "全部通过。" : "存在失败项。", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
