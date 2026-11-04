// ============================================================================
// projection_engine 主机侧 testbench · tb.cpp
//
// 编译并运行（在 src/fpga/hls/projection_engine 目录下）：
//   g++ -std=c++17 -Wall -Wextra projection_engine.cpp tb.cpp -o tb.exe
//   ./tb.exe
//
// 自检内容（全部用合成数据，真跑真比）：
//   A1 model.psm 头部自检（ps_psm_check_header）与布局量（records/payload）
//   A2 分数与**朴素三重循环**参考实现逐值一致（要求严格相等，见下）
//   A3 243 个分数的输出（全量打印）+ 最佳/次佳 ID 与 margin
//   A4 无噪合成测量下 top-1 命中构造出的真值候选
//   A5 LANES = 8/16/32 分数逐位相同（构造性不变式）
//
// 为什么 A2 能要求「严格相等」而不是给容差
// ---------------------------------------------------------------------------
//   朴素参考写成方案 §6.4 的原始求和顺序，即
//       total = Σ_c ( Σ_k ( Σ_m u·d )² )
//   每层都用「从 0 开始、按下标升序累加」的独立累加器。核内做的事完全相同：
//   m 方向按 LANES 分块、尾块补零（加 0.0 是精确的），块内按 m 升序把单积
//   依次加进 acc[c]；k 方向升序累进 ch_acc[c]；最后 ch_acc[0]+ch_acc[1]+ch_acc[2]。
//   两边的浮点加法顺序逐条相同，故结果是逐位相等的，可以写 `diff == 0.0`。
//   —— 这是刻意设计出来的（若把最优/次佳比较写进乘法链、或改成分块归约，
//      这个不变式立刻失效，所以核内注释把 P2 标成必须保持）。
// ============================================================================

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>

#include "projection_engine.h"

namespace {

// ---------------------------------------------------------------- 合成数据规模
// 真实 B 档是 M = 95x96 = 9120、N = 18x24 = 432、P = 243、K = 8。
// 本 tb 把测量与隐藏图缩小（保持 P = 243 与 K = 8 不动）以便秒级跑完并全量打印，
// 布局、访问器、并行结构与被测核完全一致。
const int kMeasH = 9;
const int kMeasW = 10;
const int kHiddenH = 6;
const int kHiddenW = 8;
const int kCandidates = 243;
const int kRankMax = 8;
const int kK = 8;
const uint16_t kLocRank = 8;

const int kTrueCand = 120;      // 构造出的「真值」候选取一个内侧编号

// ---------------------------------------------------------------- 确定性随机
struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed ? seed : 1) {}
    uint64_t next() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return s;
    }
    int16_t code() { return (int16_t)((int)((next() >> 17) % 6001u) - 3000); }
    uint16_t ucode() { return (uint16_t)((next() >> 17) % 8001u + 800u); }
};

// ---------------------------------------------------------------- 字节写入
void wr_i16(std::vector<uint8_t>& b, uint64_t off, int16_t v) {
    std::memcpy(&b[(std::size_t)off], &v, sizeof(v));
}
void wr_u16(std::vector<uint8_t>& b, uint64_t off, uint16_t v) {
    std::memcpy(&b[(std::size_t)off], &v, sizeof(v));
}

// scale 刻意取 2 的幂（精确可表示），使 code * scale 在 double 里无取整误差 ——
// 这样「核 vs 朴素参考」的差异只可能来自求和结构，而不含 scale 取整。
const double kUScale = 0.0009765625;    // 2^-10
const double kSScale = 0.00006103515625;  // 2^-14
const double kVScale = 0.0009765625;      // 2^-10

}  // namespace

int main() {
    std::printf("=== projection_engine tb（方案 §6.4 定位引擎）===\n\n");

    // ------------------------------------------------------------ 造 model.psm
    ps_psm_header_t h;
    std::memset(&h, 0, sizeof(h));
    std::memcpy(h.magic, PS_PSM_MAGIC, 4);
    h.version = PS_PSM_VERSION;
    h.header_bytes = (uint16_t)PS_PSM_HEADER_BYTES;
    h.meas_h = (uint16_t)kMeasH;
    h.meas_w = (uint16_t)kMeasW;
    h.hidden_h = (uint16_t)kHiddenH;
    h.hidden_w = (uint16_t)kHiddenW;
    h.candidates = (uint16_t)kCandidates;
    h.rank_max = (uint16_t)kRankMax;
    h.rank_default = (uint16_t)kRankMax;
    h.loc_rank = kLocRank;
    h.geometry_hash = 0;                 // tb 里跳过几何哈希
    h.payload_crc32 = 0;

    const uint64_t M = ps_psm_meas_len(&h);
    const uint64_t N = ps_psm_hidden_len(&h);
    const uint64_t rec = ps_psm_record_bytes_hdr(&h);
    h.payload_bytes = (uint64_t)kCandidates * rec;

    std::vector<uint8_t> file((std::size_t)(PS_PSM_HEADER_BYTES + h.payload_bytes), 0);
    std::memcpy(file.data(), &h, sizeof(h));

    // 填充每个候选的 U / S / V 码字与三个 scale（全部走 psm_format.h 的偏移函数）
    Rng rng(0x9E3779B97F4A7C15ull);
    for (int p = 0; p < kCandidates; ++p) {
        const uint64_t base = ps_psm_candidate_offset(&h, (uint32_t)p);
        ps_psm_candidate_hdr_t ch;
        std::memset(&ch, 0, sizeof(ch));
        ch.x_mm = (int16_t)((p % 9 - 4) * 10);
        ch.wall_distance_mm = (int16_t)(((p / 9) % 9 - 4) * 10);
        ch.z_mm = (int16_t)(((p / 81) % 3 - 1) * 20);
        ch.u_scale = (float)kUScale;
        ch.s_scale = (float)kSScale;
        ch.v_scale = (float)kVScale;
        std::memcpy(&file[(std::size_t)base], &ch, sizeof(ch));

        const uint64_t u_off = base + ps_psm_u_offset(h.rank_max, M);
        const uint64_t s_off = base + ps_psm_s_offset(h.rank_max, M);
        const uint64_t v_off = base + ps_psm_v_offset(h.rank_max, M);
        for (int k = 0; k < kRankMax; ++k) {
            for (uint64_t m = 0; m < M; ++m) {
                wr_i16(file, u_off + ((uint64_t)k * M + m) * 2u, rng.code());
            }
            wr_u16(file, s_off + (uint64_t)k * 2u, rng.ucode());
            for (uint64_t n = 0; n < N; ++n) {
                wr_i16(file, v_off + ((uint64_t)k * N + n) * 2u, rng.code());
            }
        }
    }

    // ------------------------------------------------------------ A1 头部自检
    std::printf("--- A1 model.psm 头部与布局（psm_format.h）---\n");
    int chk = ps_psm_check_header(&h, 0, (uint32_t)kMeasH, (uint32_t)kMeasW,
                                 (uint32_t)kHiddenH, (uint32_t)kHiddenW);
    std::printf("  shape: M(meas)=%llu  N(hidden)=%llu  P=%d  K=%d  rank_max=%u\n",
                (unsigned long long)M, (unsigned long long)N, kCandidates, kK,
                (unsigned)h.rank_max);
    std::printf("  record_bytes=%llu  payload_bytes=%llu  file_bytes=%zu\n",
                (unsigned long long)rec, (unsigned long long)h.payload_bytes,
                file.size());
    std::printf("  数组偏移: U=%llu S=%llu V=%llu mask=%llu（均须 4096 对齐）\n",
                (unsigned long long)ps_psm_u_offset(h.rank_max, M),
                (unsigned long long)ps_psm_s_offset(h.rank_max, M),
                (unsigned long long)ps_psm_v_offset(h.rank_max, M),
                (unsigned long long)ps_psm_mask_offset(h.rank_max, M, N));
    std::printf("  头部自检 ps_psm_check_header -> %d（0 = PS_PSM_OK）\n", chk);
    if (chk != PS_PSM_OK) {
        std::printf("[FAIL] 合成 model.psm 头部不合法\n");
        return 1;
    }

    // ------------------------------------------------------------ 造测量
    // 用真值候选 p* 的第 0 个 rank 造三通道测量（各通道给不同权重），
    // 使 p* 在无噪条件下应当拿 top-1。
    std::vector<double> meas3((std::size_t)(3 * M), 0.0);
    {
        const uint8_t* cand =
            file.data() + ps_psm_candidate_offset(&h, (uint32_t)kTrueCand);
        for (int m = 0; m < (int)M; ++m) {
            const double u0 = (double)ps_psm_read_U_km(cand, &h, 0, (uint64_t)m)
                              * kUScale;
            meas3[(std::size_t)m] = 1.0 * u0;                 // R
            meas3[(std::size_t)(M + m)] = 1.2 * u0;           // G
            meas3[(std::size_t)(2 * M + m)] = 1.4 * u0;       // B
        }
    }

    // ------------------------------------------------------------ A2/A3/A4 跑核
    std::printf("\n--- A2/A3/A4 核 vs 朴素三重循环参照 ---\n");
    std::vector<double> score16((std::size_t)kCandidates, 0.0);
    std::vector<double> score16_ch((std::size_t)kCandidates * 3, 0.0);
    ps::ProjScoreFp res;
    const int st = ps::projection_engine<16>(meas3.data(), file.data(), &h,
                                            kCandidates, kK, score16.data(),
                                            score16_ch.data(), res);
    if (st != ps::kProjOkFp) {
        std::printf("[FAIL] 核返回 %d\n", st);
        return 1;
    }

    // 朴素三重循环参考（方案 §6.4 的原始求和顺序，独立实现）
    double max_abs_diff = 0.0;
    double max_rel_diff = 0.0;
    int ref_best = -1;
    double ref_best_score = 0.0;
    std::vector<double> ref((std::size_t)kCandidates, 0.0);
    for (int p = 0; p < kCandidates; ++p) {
        const uint8_t* cand =
            file.data() + ps_psm_candidate_offset(&h, (uint32_t)p);
        ps_psm_candidate_hdr_t ch;
        std::memcpy(&ch, cand, sizeof(ch));
        double total = 0.0;
        for (int c = 0; c < 3; ++c) {
            double inner = 0.0;
            for (int k = 0; k < kK; ++k) {
                double acc = 0.0;
                for (int m = 0; m < (int)M; ++m) {
                    const double u =
                        (double)ps_psm_read_U_km(cand, &h, (uint16_t)k,
                                                 (uint64_t)m) * (double)ch.u_scale;
                    acc += u * meas3[(std::size_t)c * M + m];
                }
                inner += acc * acc;
            }
            total += inner;
        }
        ref[(std::size_t)p] = total;
        const double ad = (score16[(std::size_t)p] > total)
                              ? (score16[(std::size_t)p] - total)
                              : (total - score16[(std::size_t)p]);
        if (ad > max_abs_diff) max_abs_diff = ad;
        const double rd = (total != 0.0) ? ad / (total < 0 ? -total : total) : ad;
        if (rd > max_rel_diff) max_rel_diff = rd;
        if (ref_best < 0 || total > ref_best_score) {
            ref_best = p;
            ref_best_score = total;
        }
    }

    int n_print = 0;
    std::printf("  243 个候选分数（核，LANES=16，每行 3 个）：\n");
    for (int p = 0; p < kCandidates; ++p) {
        std::printf("  s[%3d]=%14.6e", p, score16[(std::size_t)p]);
        if (++n_print % 3 == 0) std::printf("\n");
    }
    if (n_print % 3 != 0) std::printf("\n");

    std::printf("\n  best_id=%d  second_id=%d\n", res.best_id, res.second_id);
    std::printf("  s1=%.6e  s2=%.6e  margin=(s1-s2)/(s1+eps)=%.10f\n",
                res.best_score, res.second_score, res.margin);
    std::printf("  朴素参考 best_id=%d  s1=%.6e\n", ref_best, ref_best_score);
    std::printf("  核 vs 参考：max|Δ| = %.3e   max|Δ|/s = %.3e\n",
                max_abs_diff, max_rel_diff);

    int fails = 0;
    if (max_abs_diff == 0.0) {
        std::printf("  [ok  ] 分数与朴素三重循环参考**逐值一致**（Δ 恒为 0）\n");
    } else {
        std::printf("  [FAIL] 分数与参考不一致（max|Δ| = %.3e）\n", max_abs_diff);
        ++fails;
    }
    if (res.best_id == ref_best) {
        std::printf("  [ok  ] 排序与参考一致（best_id 相同）\n");
    } else {
        std::printf("  [FAIL] best_id 不一致：核 %d vs 参考 %d\n",
                    res.best_id, ref_best);
        ++fails;
    }
    if (res.best_id == kTrueCand) {
        std::printf("  [ok  ] 无噪合成测量 top-1 命中真值候选 %d\n", kTrueCand);
    } else {
        std::printf("  [FAIL] top-1 = %d，真值候选 = %d\n", res.best_id, kTrueCand);
        ++fails;
    }
    if (res.margin > 0.0 && res.margin < 1.0) {
        std::printf("  [ok  ] margin 落在 (0,1)：最佳与次佳可区分\n");
    } else {
        std::printf("  [FAIL] margin = %.10f 超出 (0,1)\n", res.margin);
        ++fails;
    }

    // 三通道分数之和必须等于总分（§6.2 的 243 x 3）
    double worst_sum = 0.0;
    for (int p = 0; p < kCandidates; ++p) {
        const double s3 = score16_ch[(std::size_t)p * 3 + 0]
                        + score16_ch[(std::size_t)p * 3 + 1]
                        + score16_ch[(std::size_t)p * 3 + 2];
        const double d = s3 - score16[(std::size_t)p];
        if (d > worst_sum) worst_sum = d;
    }
    if (worst_sum == 0.0) {
        std::printf("  [ok  ] 3 通道分数之和 == 总分（243 x 3 通道输出自洽）\n");
    } else {
        std::printf("  [FAIL] 通道和与总分之差 = %.3e\n", worst_sum);
        ++fails;
    }

    // ------------------------------------------------------------ A5 LANES 不变性
    std::printf("\n--- A5 LANES = 8 / 16 / 32 逐位一致性 ---\n");
    std::vector<double> s8((std::size_t)kCandidates, 0.0);
    std::vector<double> s32((std::size_t)kCandidates, 0.0);
    ps::ProjScoreFp r8, r32;
    ps::projection_engine<8>(meas3.data(), file.data(), &h, kCandidates, kK,
                             s8.data(), 0, r8);
    ps::projection_engine<32>(meas3.data(), file.data(), &h, kCandidates, kK,
                              s32.data(), 0, r32);
    double d8 = 0.0, d32 = 0.0;
    for (int p = 0; p < kCandidates; ++p) {
        if (s8[(std::size_t)p] != score16[(std::size_t)p]) d8 += 1.0;
        if (s32[(std::size_t)p] != score16[(std::size_t)p]) d32 += 1.0;
    }
    std::printf("  与 LANES=16 不等的元素数：LANES=8 -> %.0f，LANES=32 -> %.0f\n",
                d8, d32);
    std::printf("  best_id：8 -> %d，16 -> %d，32 -> %d\n",
                r8.best_id, res.best_id, r32.best_id);
    if (d8 == 0.0 && d32 == 0.0 && r8.best_id == res.best_id
        && r32.best_id == res.best_id) {
        std::printf("  [ok  ] 8/16/32 分数逐位相同，排序一致（m 升序单积累加的不变式）\n");
    } else {
        std::printf("  [FAIL] LANES 改变了结果\n");
        ++fails;
    }

    std::printf("\n=== 结论：%s（失败项 %d）===\n", fails == 0 ? "全部通过" : "有失败项", fails);
    return fails == 0 ? 0 : 1;
}
