// ============================================================================
// X10 定位核主机侧验证
//
// 把 M2 几何算核与 M3 Gram 累加核串起来，对官方第 1 轮的全部候选算
//   - G = A_d^T A_d 的上三角（545,490 个）
//   - z = A_d^T y
// 并按候选编号顺序写成一个二进制文件。特征分解留给 Python（numpy），
// 因为**这一步在真实系统里本来就是 PS 侧的活**（A6 的路线：
// PL 只出 Gram 与右端项，PS 做特征分解）。这样 X10 验的就是 PL 那一半。
//
// 编译（在 src/hardware 目录下）：
//   g++ -O3 -march=native -fopenmp -std=c++11 x10_localize.cpp -o t_x10.exe
// 运行（两种模式）：
//   ./t_x10.exe <out.bin> [k_start] [k_stop]
//       用官方第 1 轮网格，跑候选区间（默认全部 972 个）
//   ./t_x10.exe <out.bin> cand <cand.bin>
//       用**显式候选表**（float32[3*K]，每 3 个 = x, y, z）。X11 的一维扫描
//       与坐标下降各轮都走这个入口，这样调度实验不需要改几何代码。
//
// 输出布局（float32，小端）：
//   [0, T*K)        每个候选 T 个 G 上三角值（行优先，np.triu_indices 顺序）
//   [T*K, (T+N)*K)  每个候选 N 个 A_d^T y
//   共 K*(T+N) 个 float32。候选 k 的上三角起点 = k*T*K 处的第 k*T 个。
//
// 为什么按 fseek 定位写
//   并行下完成顺序不确定，若顺序写会错位。每个候选按自己的偏移 fseek 后再写，
//   顺序就无关紧要了。
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

static std::vector<uint8_t> slurp(const std::string& name) {
    // 名字里带盘符或路径分隔符时视为已限定路径，不再拼 HWDATA_DIR。
    // 主机工具经常被 Python 以绝对路径调用，无脑拼接会把路径搞坏。
    // 另注：Windows 下窄字符 fopen 走系统 ANSI 码页，**路径必须纯 ASCII**，
    // 否则中文目录名会乱码。调用方负责保证这一点。
    const bool qualified = (name.find(':') != std::string::npos)
                        || (name.find('\\') != std::string::npos)
                        || (name.find('/') != std::string::npos);
    const std::string p = qualified ? name : (std::string(HWDATA_DIR) + name);
    FILE* f = std::fopen(p.c_str(), "rb");
    if (!f) { std::printf("  [ERR] 无法打开 %s\n", p.c_str()); std::exit(2); }
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> b((size_t)n);
    if (n > 0 && std::fread(b.data(), 1, (size_t)n, f) != (size_t)n) {
        std::printf("  [ERR] 读取失败 %s\n", p.c_str()); std::exit(2);
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

int main(int argc, char** argv) {
    const char* out_path = (argc > 1) ? argv[1] : "x10_gram_z.bin";
    const bool cand_mode = (argc > 2) && (std::strcmp(argv[2], "cand") == 0);
    const char* cand_path = cand_mode ? argv[3] : "";
    int k_start = (!cand_mode && argc > 2) ? std::atoi(argv[2]) : 0;
    int k_stop = (!cand_mode && argc > 3) ? std::atoi(argv[3]) : -1;

    std::printf("X10 定位核主机侧验证：M2 + M3 -> Gram / A_d^T y\n");

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
    std::vector<float> yd_f = load<float>("x9_yd.bin");
    std::vector<float> grid = load<float>("x10_grid.bin");
    std::vector<int32_t> meta = load<int32_t>("x10_meta.bin");

    int nx = meta[0], ny = meta[1], nz = meta[2];
    int K = meta[3];
    const int N = (int)mon_x_f.size();
    const int T = N * (N + 1) / 2;

    // 显式候选表模式（X11 的扫描与各轮调度走这里）
    std::vector<double> cand_xyz;
    if (cand_mode) {
        std::vector<uint8_t> cb = slurp(cand_path);
        const size_t nf = cb.size() / sizeof(float);
        std::vector<float> cf(nf);
        if (nf) std::memcpy(cf.data(), cb.data(), nf * sizeof(float));
        cand_xyz.assign(cf.begin(), cf.end());
        K = (int)(cand_xyz.size() / 3);
        nx = ny = nz = 0;
    }

    if (!cand_mode && (k_stop < 0 || k_stop > K)) k_stop = K;
    if (cand_mode) { k_start = 0; k_stop = K; }
    if (cand_mode) {
        std::printf("  [info] 显式候选表 %s：%d 个候选\n", cand_path, K);
    } else {
        std::printf("  [info] 网格 %d x %d x %d = %d 候选；本次跑 [%d, %d)\n",
                    nx, ny, nz, K, k_start, k_stop);
    }
    std::printf("  [info] N=%d  T=%d  y_w=%.6f  C=%.6g\n", N, T, y_w, C);

    std::vector<double> wall_x(wall_x_f.begin(), wall_x_f.end());
    std::vector<double> wall_z(wall_z_f.begin(), wall_z_f.end());
    std::vector<double> mon_x(mon_x_f.begin(), mon_x_f.end());
    std::vector<double> mon_z(mon_z_f.begin(), mon_z_f.end());
    std::vector<double> yd(yd_f.begin(), yd_f.end());
    std::vector<double> xv, yv, zv;
    if (!cand_mode) {
        xv.assign(grid.begin(), grid.begin() + nx);
        yv.assign(grid.begin() + nx, grid.begin() + nx + ny);
        zv.assign(grid.begin() + nx + ny, grid.end());
    }

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

    // d2 范围（与 X9 主机测试同口径）
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
    std::printf("  [info] d2 范围 [%.6f, %.6f]  LUT %d 项\n",
                y_w * y_w, y_w * y_w + dxmax * dxmax + dzmax * dzmax,
                (int)PS_INV4_LUT);

    ps::GramPlan plan;
    ps::gram_make_plan(N, 2, plan);       // X9 定案：2 块 / 20 bit
    std::printf("  [info] Gram 分 %d 块，切点 %d / %d，最大块 %d 条"
                "（20 bit 峰值 %.2f%% BRAM）\n",
                plan.n_blocks, plan.cuts[1], plan.cuts[2], plan.max_entries,
                100.0 * plan.max_entries * 20.0 / 32.1e6);

    FILE* fo = std::fopen(out_path, "wb");
    if (!fo) { std::printf("  [ERR] 无法写 %s\n", out_path); return 2; }

    const size_t per_cand = (size_t)(T + N);
    std::vector<float> wbuf(per_cand);
    long long total_mac = 0;

    std::printf("\n  开始：%d 个候选，每个 8.591e9 MAC\n", k_stop - k_start);
    std::fflush(stdout);

#pragma omp parallel
    {
        std::vector<double> acc(plan.max_entries);
        std::vector<int> base(N + 1);
        std::vector<double> w_lo(N), w_hi(N), ad_row(N);
        std::vector<double> gram_triu(T), aty(N);
        std::vector<ps::OccRect> rect(N);
        long long local_mac = 0;

#pragma omp for schedule(dynamic, 1)
        for (int k = k_start; k < k_stop; ++k) {
            double ll[3];
            if (cand_mode) {
                ll[0] = cand_xyz[(size_t)k * 3 + 0];
                ll[1] = cand_xyz[(size_t)k * 3 + 1];
                ll[2] = cand_xyz[(size_t)k * 3 + 2];
            } else {
                const int ix = k / (ny * nz);
                const int rem = k % (ny * nz);
                const int iy = rem / nz;
                const int iz = rem % nz;
                ll[0] = xv[ix]; ll[1] = yv[iy]; ll[2] = zv[iz];
            }

            ps::m2_geometry(ll, g, mon_x_f.data(), mon_z_f.data(),
                            rect.data(), N);
            local_mac += ps::m3_accum_gram(
                plan, rect.data(), wall_x.data(), wall_z.data(),
                mon_x.data(), mon_z.data(), tp, lut, C, yd.data(),
                gram_triu.data(), aty.data(), acc.data(), base.data(),
                w_lo.data(), w_hi.data(), ad_row.data());

#pragma omp critical
            {
                for (int t = 0; t < T; ++t) wbuf[t] = (float)gram_triu[t];
                for (int i = 0; i < N; ++i) wbuf[T + i] = (float)aty[i];
                const long long off = (long long)k * (long long)per_cand
                                      * (long long)sizeof(float);
                std::fseek(fo, (long)off, SEEK_SET);
                std::fwrite(wbuf.data(), sizeof(float), per_cand, fo);
                const bool tick = cand_mode
                    ? ((k % 25 == 0) || k == k_stop - 1)
                    : ((k - k_start) % 50 == 0 || k == k_stop - 1);
                if (tick) {
                    std::printf("   候选 %4d/%d  ll=(%.5f, %.5f, %.5f)\n",
                                k, k_stop - 1, ll[0], ll[1], ll[2]);
                    std::fflush(stdout);
                }
            }
        }
#pragma omp critical
        total_mac += local_mac;
    }

    std::fclose(fo);
    std::printf("\n  完成。累计 MAC %.4e（%d 个候选 x 8.591e9）\n",
                (double)total_mac, k_stop - k_start);
    std::printf("  输出 %s（%.1f MB）\n", out_path,
                per_cand * (double)(k_stop - k_start) * 4.0 / 1e6);
    return 0;
}
