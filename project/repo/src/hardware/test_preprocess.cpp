// ============================================================================
// L1-1 主机侧测试：验证 HLS 风格预处理链与 S0-3 测试向量逐值相等
//
// 编译（在 hwdata 目录的同级）：
//   g++ -O2 -std=c++11 -DHWDATA_DIR=\".../hwdata/\" test_preprocess.cpp -o t
// 运行：
//   ./t
//
// 判据：全部为整数，逐值相等（S0-3 原则 P1）。任一处不等即计为失败。
// ============================================================================
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "preprocess.hpp"

#ifndef HWDATA_DIR
#define HWDATA_DIR "./hwdata/"
#endif

// 向量目录。默认用编译期 HWDATA_DIR；可由 main 的 argv[1] 覆盖。
// 用命令行参数而不是 -D 宏，是为了绕开 Windows cmd 吃掉
// `-DHWDATA_DIR="..."` 引号、宏展开成裸路径的老问题（见 l1_1_preprocess_hls.md §4）。
static std::string g_dir = HWDATA_DIR;

static int g_pass = 0;
static int g_fail = 0;

// ---------------------------------------------------------------- 读取辅助
static std::vector<uint8_t> slurp(const std::string& name) {
    std::string p = g_dir + name + ".bin";
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

// ---------------------------------------------------------------- 比对
template <typename Got, typename Exp>
static void cmp(const char* tag, const Got* got, const Exp* exp, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        if ((int64_t)got[i] != (int64_t)exp[i]) {
            std::printf("  [FAIL] %-34s 第 %zu 项: got %lld, exp %lld\n",
                        tag, i, (long long)got[i], (long long)exp[i]);
            ++g_fail;
            return;
        }
    }
    std::printf("  [ok]   %-34s (%zu 项)\n", tag, n);
    ++g_pass;
}

// ============================================================================
// 用例 1：v1——20x20 RAW 解交织四路 + 块和（block = 10）
// ============================================================================
static void case_v1(const std::string& who) {
    std::printf("[v1 / %s]\n", who.c_str());
    const int B = 10;
    // 导出文件的命名带 npz 的 stem 前缀
    const std::string name = "v1_block_small__" + who;
    auto raw = load<uint16_t>(name + "_raw");
    if (raw.size() != 400) { std::printf("  [ERR] RAW 尺寸异常\n"); ++g_fail; return; }

    ps::pix_t r[100], g1[100], g2[100], b[100];
    ps::bayer_extract(raw.data(), 20, 20, 0, 0, r);
    ps::bayer_extract(raw.data(), 20, 20, 1, 0, g1);
    ps::bayer_extract(raw.data(), 20, 20, 2, 0, g2);
    ps::bayer_extract(raw.data(), 20, 20, 3, 0, b);

    auto e_r = load<uint16_t>(name + "_r");
    auto e_g1 = load<uint16_t>(name + "_g1");
    auto e_g2 = load<uint16_t>(name + "_g2");
    auto e_b = load<uint16_t>(name + "_b");
    cmp((name + " 解交织 R").c_str(), r, e_r.data(), 100);
    cmp((name + " 解交织 G1").c_str(), g1, e_g1.data(), 100);
    cmp((name + " 解交织 G2").c_str(), g2, e_g2.data(), 100);
    cmp((name + " 解交织 B").c_str(), b, e_b.data(), 100);

    ps::gsum_t gs[100];
    ps::green_sum(g1, g2, 100, gs);

    ps::bs_t out[100];
    int bh = 0, bw = 0;
    ps::block_sum(r, 10, 10, B, out, &bh, &bw);
    auto e_sr = load<uint32_t>(name + "_sum_r");
    cmp((name + " 块和 R").c_str(), out, e_sr.data(), (size_t)(bh * bw));

    ps::block_sum(gs, 10, 10, B, out, &bh, &bw);
    auto e_sg = load<uint32_t>(name + "_sum_g1g2");
    cmp((name + " 块和 G1+G2").c_str(), out, e_sg.data(), (size_t)(bh * bw));

    ps::block_sum(b, 10, 10, B, out, &bh, &bw);
    auto e_sb = load<uint32_t>(name + "_sum_b");
    cmp((name + " 块和 B").c_str(), out, e_sb.data(), (size_t)(bh * bw));
    std::printf("\n");
}

// ============================================================================
// 用例 2：v4 跨行分块——输入是「已解交织的平面」，block = 5
// ============================================================================
static void case_crossrow() {
    std::printf("[v4 / crossrow]\n");
    auto plane = load<uint16_t>("v4_edge_cases__crossrow_plane");
    auto e_ok = load<uint32_t>("v4_edge_cases__crossrow_correct");
    auto e_bad = load<uint32_t>("v4_edge_cases__crossrow_wrong");

    ps::bs_t out[9];
    int bh = 0, bw = 0;
    ps::block_sum(plane.data(), 15, 15, 5, out, &bh, &bw);
    cmp("跨行分块（须为 correct）", out, e_ok.data(), 9);

    bool same_as_wrong = true;
    for (int i = 0; i < 9; ++i) {
        if ((uint64_t)out[i] != (uint64_t)e_bad[i]) { same_as_wrong = false; break; }
    }
    if (same_as_wrong) {
        std::printf("  [FAIL] 结果与 crossrow_wrong 相同——存在跨行分块 bug\n");
        ++g_fail;
    } else {
        std::printf("  [ok]   结果与 crossrow_wrong 不同\n");
        ++g_pass;
    }
    std::printf("\n");
}

// ============================================================================
// 用例 3：v4 暗电平——20x20 全 1000，减 1000 后应全 0；解交织后 block = 5
// ============================================================================
static void case_dark() {
    std::printf("[v4 / dark]\n");
    auto raw = load<uint16_t>("v4_edge_cases__dark_raw");
    auto off = load<int64_t>("v4_edge_cases__dark_offset");
    auto e = load<uint32_t>("v4_edge_cases__dark_exp");

    ps::pix_t buf[400];
    for (int i = 0; i < 400; ++i) buf[i] = raw[i];
    ps::subtract_dark(buf, 400, (ps::pix_t)off[0]);

    ps::pix_t r[100];
    ps::bayer_extract(buf, 20, 20, 0, 0, r);
    ps::bs_t out[4];
    int bh = 0, bw = 0;
    ps::block_sum(r, 10, 10, 5, out, &bh, &bw);
    cmp("减暗电平后块和（应为 0）", out, e.data(), 4);
    std::printf("\n");
}

// ============================================================================
// 用例 4：v3——32 帧 x 200x200，6 个累积档，block = 10
//   注意：帧累积与解交织、块和都是线性操作，可交换顺序。
//         这里按硬件做法「逐帧解交织 -> 块和 -> 累加」，与参考实现等价。
// ============================================================================
static void case_v3() {
    std::printf("[v3 / 帧累积]\n");
    const int H = 200, W = 200, B = 10;
    const int PH = H / 2, PW = W / 2;
    const int BH = PH / B, BW = PW / B;
    const int NBLK = BH * BW;

    auto frames = load<uint16_t>("v3_accum_frames__in_raw_frames");
    if (frames.size() != (size_t)32 * H * W) {
        std::printf("  [ERR] 帧数据尺寸异常 %zu\n", frames.size());
        ++g_fail;
        return;
    }

    const int marks[6] = {1, 2, 4, 8, 16, 32};
    const char* mark_name[6] = {"acc1", "acc2", "acc4", "acc8", "acc16", "acc32"};
    // 注意：向量的命名里 blocksum 用 g1g2、diff 用 g，两者不一致
    const char* bs_name[3] = {"r", "g1g2", "b"};
    const char* df_name[3] = {"r", "g", "b"};

    // 逐帧累加三个通道的块和
    std::vector<ps::acc_t> acc[3];
    for (int c = 0; c < 3; ++c) acc[c].assign(NBLK, 0);

    int mark_idx = 0;
    for (int f = 0; f < 32; ++f) {
        const uint16_t* fr = frames.data() + (size_t)f * H * W;
        // 解交织：r, b, 以及 g1/g2 之和
        // 放在静态存储区：五个 10000 元素的 64 位数组会超过默认栈空间
        static ps::pix_t rr[PH * PW], bb[PH * PW], gg[PH * PW];
        static ps::pix_t t1[PH * PW], t2[PH * PW];
        ps::bayer_extract(fr, H, W, 0, 0, rr);
        ps::bayer_extract(fr, H, W, 3, 0, bb);
        ps::bayer_extract(fr, H, W, 1, 0, t1);
        ps::bayer_extract(fr, H, W, 2, 0, t2);
        ps::green_sum(t1, t2, PH * PW, gg);

        const ps::pix_t* ch[3] = {rr, gg, bb};
        for (int c = 0; c < 3; ++c) {
            static ps::gsum_t tmp[PH * PW];
            for (int i = 0; i < PH * PW; ++i) tmp[i] = ch[c][i];
            ps::bs_t out[NBLK];
            int bh = 0, bw = 0;
            // 逐块求和（直接在这里展开，避免为三种容器写三次模板实例）
            for (int br = 0; br < BH; ++br) {
                for (int bc = 0; bc < BW; ++bc) {
                    ps::acc_t s = 0;
                    for (int dr = 0; dr < B; ++dr) {
                        for (int dc = 0; dc < B; ++dc) {
                            s += (ps::acc_t)tmp[(br * B + dr) * PW + (bc * B + dc)];
                        }
                    }
                    out[br * BW + bc] = (ps::bs_t)s;
                }
            }
            (void)bh;
            (void)bw;
            for (int i = 0; i < NBLK; ++i) acc[c][i] += (ps::acc_t)out[i];
        }

        if (mark_idx < 6 && (f + 1) == marks[mark_idx]) {
            char tag[96];
            for (int c = 0; c < 3; ++c) {
                std::snprintf(tag, sizeof(tag), "%s blocksum %s",
                              mark_name[mark_idx], bs_name[c]);
                auto e = load<uint32_t>(
                    std::string("v3_accum_frames__exp_") + mark_name[mark_idx] +
                    "_blocksum_" + bs_name[c]);
                cmp(tag, acc[c].data(), e.data(), (size_t)NBLK);

                // 差分（下一行减上一行）
                std::vector<ps::dif_t> df((size_t)(BH - 1) * BW);
                ps::row_diff(acc[c].data(), BH, BW, df.data());
                std::snprintf(tag, sizeof(tag), "%s diff %s",
                              mark_name[mark_idx], df_name[c]);
                auto ed = load<int32_t>(
                    std::string("v3_accum_frames__exp_") + mark_name[mark_idx] +
                    "_diff_" + df_name[c]);
                cmp(tag, df.data(), ed.data(), df.size());
            }
            ++mark_idx;
        }
    }
    std::printf("\n");
}

// ============================================================================
// 用例 5：v2——1920x1920 全图，block = 10，输出 96x96 + 95x96 差分
// ============================================================================
static void case_v2() {
    std::printf("[v2 / 全图]\n");
    const int H = 1920, W = 1920, B = 10;
    const int PH = H / 2, PW = W / 2;
    const int BH = PH / B, BW = PW / B;
    const int NBLK = BH * BW;

    auto raw = load<uint16_t>("v2_frame_full__in_raw");
    if (raw.size() != (size_t)H * W) {
        std::printf("  [ERR] 尺寸异常 %zu\n", raw.size());
        ++g_fail;
        return;
    }

    std::vector<ps::pix_t> r(PH * PW), g1(PH * PW), g2(PH * PW), b(PH * PW);
    ps::bayer_extract(raw.data(), H, W, 0, 0, r.data());
    ps::bayer_extract(raw.data(), H, W, 1, 0, g1.data());
    ps::bayer_extract(raw.data(), H, W, 2, 0, g2.data());
    ps::bayer_extract(raw.data(), H, W, 3, 0, b.data());
    std::vector<ps::gsum_t> gs(PH * PW);
    ps::green_sum(g1.data(), g2.data(), PH * PW, gs.data());

    const char* names[3] = {"g1g2", "r", "b"};
    std::vector<ps::pix_t>* ch[3];
    ch[0] = &gs, ch[1] = &r, ch[2] = &b;

    for (int c = 0; c < 3; ++c) {
        std::vector<ps::bs_t> out(NBLK);
        int bh = 0, bw = 0;
        ps::block_sum(ch[c]->data(), PH, PW, B, out.data(), &bh, &bw);
        char tag[64];
        std::snprintf(tag, sizeof(tag), "全图 blocksum %s", names[c]);
        auto e = load<uint32_t>(std::string("v2_frame_full__exp_blocksum_") + names[c]);
        cmp(tag, out.data(), e.data(), (size_t)NBLK);

        std::vector<ps::dif_t> df((size_t)(BH - 1) * BW);
        ps::row_diff(out.data(), BH, BW, df.data());
        std::snprintf(tag, sizeof(tag), "全图 diff %s", names[c]);
        const char* dn = (c == 0) ? "g" : names[c];
        auto ed = load<int32_t>(std::string("v2_frame_full__exp_diff_") + dn);
        cmp(tag, df.data(), ed.data(), df.size());
    }
    std::printf("\n");
}

// ============================================================================
static void print_widths() {
    ps::BitWidths w = ps::bit_widths();
    std::printf("[位宽自检] 当前宏配置 IN=%d BLOCK=%d FRAMES=%d COLOR=%d\n",
                PS_IN_BITS, PS_BLOCK, PS_FRAMES, PS_COLOR);
    std::printf("  像素 %d bit | G1+G2 %d bit | 块和 %d bit | 帧累积 %d bit | 差分 %d bit\n",
                w.in_bits, w.gsum_bits, w.bs_bits, w.acc_bits, w.dif_bits);
    std::printf("  （判据：块和须 >= 23，16 帧累积须 >= 27，见 S0-2 位宽表）\n");
    if (w.bs_bits >= 23 && w.acc_bits >= 27) {
        std::printf("  [ok]   位宽满足 S0-2\n");
        ++g_pass;
    } else {
        std::printf("  [FAIL] 位宽不足\n");
        ++g_fail;
    }
    std::printf("\n");
}

int main(int argc, char** argv) {
    if (argc > 1) {
        g_dir = argv[1];
        if (!g_dir.empty() && g_dir.back() != '/' && g_dir.back() != '\\') {
            g_dir += '/';
        }
    }
    std::printf("== L1-1 预处理链主机侧测试 ==\n");
    std::printf("   向量目录：%s\n\n", g_dir.c_str());
    print_widths();

    const char* names[4] = {"ramp", "sat", "zero", "official"};
    for (int i = 0; i < 4; ++i) case_v1(names[i]);
    case_crossrow();
    case_dark();
    case_v3();
    case_v2();

    std::printf("===========================================\n");
    std::printf("通过 %d 项，失败 %d 项\n", g_pass, g_fail);
    std::printf("===========================================\n");
    return g_fail == 0 ? 0 : 1;
}
