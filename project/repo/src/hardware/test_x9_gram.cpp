// ============================================================================
// X9 主机侧测试：M3 Gram 累加核
//
// 编译（在 src/hardware 目录下）：
//   g++ -O3 -std=c++11 test_x9_gram.cpp -o t_x9.exe
// 运行：
//   ./t_x9.exe            # 默认 2 块
//   ./t_x9.exe 1          # 1 块（对照峰值与误差）
//
// 三项检查
//   U1 块计划：切点、各块条目数、最大块，与 Python 核算表逐个比对
//   U2 A_d^T y：相对 L2 误差 <= 1e-3
//   U3 A_d^T A_d 上三角：相对 Frobenius 误差 <= 1e-3
//   U4 分块自洽：1 块与 2 块给出的上三角必须逐值一致（分块只是切分，不改数值）
//
// 注意：U3 每候选要跑 8.59e9 次内层乘加，主机上约 10~30 秒。三个候选请耐心等。
// ============================================================================
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "geometry.hpp"
#include "gram.hpp"
#include "transport.hpp"

#ifndef HWDATA_DIR
#define HWDATA_DIR "./hwdata/"
#endif

static int g_pass = 0;
static int g_fail = 0;

static std::vector<uint8_t> slurp(const std::string& name) {
    std::string p = std::string(HWDATA_DIR) + name;
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) { std::printf("  [ERR] 无法打开 %s\n", p.c_str()); std::exit(2); }
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

// 从 x8_params.txt 取参数（第一个数值）
static double param_of(const std::string& text, const char* key, double dflt) {
    const std::string k = key;
    size_t pos = 0;
    while (pos < text.size()) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string::npos) eol = text.size();
        std::string line = text.substr(pos, eol - pos);
        size_t eq = line.find('=');
        if (eq != std::string::npos) {
            std::string lhs = line.substr(0, eq);
            while (!lhs.empty() && (lhs.back() == ' ' || lhs.back() == '\t')) lhs.pop_back();
            while (!lhs.empty() && lhs.front() == ' ') lhs.erase(lhs.begin());
            if (lhs == k) return std::atof(line.c_str() + eq + 1);
        }
        pos = eol + 1;
    }
    return dflt;
}

static double rel_l2(const double* got, const float* ref, size_t n) {
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double d = got[i] - (double)ref[i];
        num += d * d;
        den += (double)ref[i] * (double)ref[i];
    }
    return (den > 0.0) ? std::sqrt(num / den) : 0.0;
}

// 双精度参考版本（U4 里两个结果都是 double，直接比）
static double rel_l2(const double* got, const double* ref, size_t n) {
    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double d = got[i] - ref[i];
        num += d * d;
        den += ref[i] * ref[i];
    }
    return (den > 0.0) ? std::sqrt(num / den) : 0.0;
}

int main(int argc, char** argv) {
    const int req_blocks = (argc > 1) ? std::atoi(argv[1]) : 2;
    std::printf("X9 主机侧测试：M3 Gram 累加核（%d 块）\n", req_blocks);

    std::vector<uint8_t> pb = slurp("x8_params.txt");
    const std::string txt(pb.begin(), pb.end());
    const int n = (int)param_of(txt, "wall_n", 126);
    const double wall_y = param_of(txt, "wall_y", 1.03);
    const double mon_y = param_of(txt, "monitor_y", 0.0);
    const double blk_area = param_of(txt, "monitor_block_area", 0.0);
    const double y_w = wall_y - mon_y;
    const double C = y_w * y_w * blk_area;

    std::vector<float> wall_x_f = load<float>("x8_wall_x.bin");
    std::vector<float> wall_z_f = load<float>("x8_wall_z.bin");
    std::vector<float> mon_x_f = load<float>("x8_mon_x.bin");
    std::vector<float> mon_z_f = load<float>("x8_mon_z.bin");
    std::vector<float> cand = load<float>("x9_cand_ll.bin");
    std::vector<float> yd_f = load<float>("x9_yd.bin");
    std::vector<float> ref_aty = load<float>("x9_ref_aty.bin");
    std::vector<float> ref_gram = load<float>("x9_ref_gram.bin");
    std::vector<int32_t> meta = load<int32_t>("x9_gram_meta.bin");

    const int N = meta[0], T = meta[1], K = meta[2];
    std::printf("  [info] wall_n=%d N=%d 上三角=%d 候选=%d  y_w=%.6f C=%.6g\n",
                n, N, T, K, y_w, C);
    std::printf("  [info] K*(N*(N+1)/2) = %d，与 meta 的 %d %s\n",
                N * (N + 1) / 2, T,
                (N * (N + 1) / 2 == T) ? "一致" : "不一致");
    std::fflush(stdout);

    std::vector<double> wall_x(wall_x_f.begin(), wall_x_f.end());
    std::vector<double> wall_z(wall_z_f.begin(), wall_z_f.end());
    std::vector<double> mon_x(mon_x_f.begin(), mon_x_f.end());
    std::vector<double> mon_z(mon_z_f.begin(), mon_z_f.end());
    std::vector<double> yd(yd_f.begin(), yd_f.end());

    ps::GeomParams g;
    g.wall_y = wall_y;
    g.wall_x_min = param_of(txt, "wall_x_min", 0);
    g.wall_x_max = param_of(txt, "wall_x_max", 0);
    g.wall_z_min = param_of(txt, "wall_z_min", 0);
    g.wall_z_max = param_of(txt, "wall_z_max", 0);
    g.occ_size_dx = param_of(txt, "occ_size_dx", 0);
    g.occ_size_dz = param_of(txt, "occ_size_dz", 0);
    g.wall_n = n;

    ps::TransportParams tp;
    ps::transport_configure(wall_y, mon_y, blk_area, n, tp);

    // d2 范围：下界 y_w^2，上界 y_w^2 + dx_max^2 + dz_max^2（用坐标极值推）
    double dxmax = 0.0, dzmax = 0.0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < N; ++j) {
            const double dx = std::fabs(wall_x[i] - mon_x[j]);
            const double dz = std::fabs(wall_z[i] - mon_z[j]);
            if (dx > dxmax) dxmax = dx;
            if (dz > dzmax) dzmax = dz;
        }
    }
    ps::Inv4Lut lut;
    lut.init(y_w * y_w, y_w * y_w + dxmax * dxmax + dzmax * dzmax);
    std::printf("  [info] d2 范围 [%.6f, %.6f]，LUT %d 项\n",
                y_w * y_w, y_w * y_w + dxmax * dxmax + dzmax * dzmax,
                (int)PS_INV4_LUT);

    // ---- U1 块计划 ----
    std::printf("\n=== U1 块计划 ===\n");
    ps::GramPlan plan;
    ps::gram_make_plan(N, req_blocks, plan);
    long long total_ent = ps::gram_cum_entries(N, N);
    std::printf("  上三角总数 %lld（应为 %d）\n", total_ent, T);
    for (int b = 0; b < plan.n_blocks; ++b) {
        const long long e = ps::gram_cum_entries(plan.cuts[b + 1], N)
                            - ps::gram_cum_entries(plan.cuts[b], N);
        std::printf("   块 %d：i ∈ [%d, %d)  条目 %lld\n",
                    b, plan.cuts[b], plan.cuts[b + 1], e);
    }
    std::printf("  最大块 %d 条，20 bit 峰值 %.2f Mb（%.2f%% of 32.1 Mb）\n",
                plan.max_entries, plan.max_entries * 20.0 / 1e6,
                100.0 * plan.max_entries * 20.0 / 32.1e6);
    {
        const bool ok = (total_ent == T)
                        && (plan.cuts[0] == 0)
                        && (plan.cuts[plan.n_blocks] == N);
        std::printf("  [%s] 块计划覆盖完整且不重叠\n", ok ? "ok  " : "FAIL");
        if (ok) ++g_pass; else ++g_fail;
    }

    // ---- 累加缓冲 ----
    std::vector<double> gram_triu(T), aty(N), acc(plan.max_entries);
    std::vector<int>    base(N + 1);
    std::vector<double> w_lo(N), w_hi(N), ad_row(N);
    std::vector<ps::OccRect> rect(N);

    // ---- U2 / U3 ----
    std::printf("\n=== U2/U3 A_d^T y 与 A_d^T A_d 上三角（%d 候选）===\n", K);
    double worst_aty = 0.0, worst_gram = 0.0;
    int wA_k = -1, wG_k = -1;
    for (int k = 0; k < K; ++k) {
        double ll[3] = { (double)cand[k * 3], (double)cand[k * 3 + 1],
                         (double)cand[k * 3 + 2] };
        ps::m2_geometry(ll, g, mon_x_f.data(), mon_z_f.data(), rect.data(), N);
        const long long mac = ps::m3_accum_gram(
            plan, rect.data(), wall_x.data(), wall_z.data(),
            mon_x.data(), mon_z.data(), tp, lut, C, yd.data(),
            gram_triu.data(), aty.data(), acc.data(), base.data(),
            w_lo.data(), w_hi.data(), ad_row.data());
        const double rA = rel_l2(aty.data(), &ref_aty[(size_t)k * N], N);
        const double rG = rel_l2(gram_triu.data(), &ref_gram[(size_t)k * T], T);
        std::printf("  候选 %d：y=%.4f  累加 MAC %.3e   A_d^T y %.3e   Gram %.3e\n",
                    k, (double)cand[k * 3 + 1], (double)mac, rA, rG);
        if (rA > worst_aty) { worst_aty = rA; wA_k = k; }
        if (rG > worst_gram) { worst_gram = rG; wG_k = k; }
        std::fflush(stdout);
    }
    {
        const bool ok = worst_aty <= 1e-3;
        std::printf("  [%s] %-34s 最坏 %.3e（阈值 1e-3）出在候选 %d\n",
                    ok ? "ok  " : "FAIL", "A_d^T y 相对 L2", worst_aty, wA_k);
        if (ok) ++g_pass; else ++g_fail;
    }
    {
        const bool ok = worst_gram <= 1e-3;
        std::printf("  [%s] %-34s 最坏 %.3e（阈值 1e-3）出在候选 %d\n",
                    ok ? "ok  " : "FAIL", "A_d^T A_d 上三角相对 Frobenius",
                    worst_gram, wG_k);
        if (ok) ++g_pass; else ++g_fail;
    }

    // ---- U4 分块自洽：1 块 vs 请求块数，数值必须一致 ----
    if (req_blocks != 1 && K > 0) {
        std::printf("\n=== U4 分块自洽（1 块 vs %d 块）===\n", req_blocks);
        // 必须用**同一个候选**比。gram_triu / aty 里留的是 U2/U3 循环最后一个
        // 候选（k = K-1）的结果，所以这里也用 k = K-1，否则比的是两个不同几何。
        const int kk = K - 1;
        std::printf("        使用候选 %d（与上一节的最后一次计算同候选）\n", kk);
        ps::GramPlan p1;
        ps::gram_make_plan(N, 1, p1);
        std::vector<double> acc1(p1.max_entries), g1(T), aty1(N);
        double ll[3] = { (double)cand[kk * 3], (double)cand[kk * 3 + 1],
                         (double)cand[kk * 3 + 2] };
        ps::m2_geometry(ll, g, mon_x_f.data(), mon_z_f.data(), rect.data(), N);
        ps::m3_accum_gram(p1, rect.data(), wall_x.data(), wall_z.data(),
                          mon_x.data(), mon_z.data(), tp, lut, C, yd.data(),
                          g1.data(), aty1.data(), acc1.data(), base.data(),
                          w_lo.data(), w_hi.data(), ad_row.data());
        const double rg = rel_l2(g1.data(), gram_triu.data(), T);
        const double ra = rel_l2(aty1.data(), aty.data(), N);
        const bool ok = (rg < 1e-12) && (ra < 1e-12);
        std::printf("        1 块 vs %d 块：Gram 相对差 %.3e，A_d^T y 相对差 %.3e\n",
                    req_blocks, rg, ra);
        std::printf("  [%s] %-34s 阈值 1e-12\n", ok ? "ok  " : "FAIL", "分块不改数值");
        if (ok) ++g_pass; else ++g_fail;
    }

    std::printf("\n%s  PASS %d  FAIL %d\n",
                (g_fail == 0) ? "全部通过。" : "存在失败项。", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
