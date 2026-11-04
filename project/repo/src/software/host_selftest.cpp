// ============================================================================
// host_selftest.cpp —— PS 侧五个模块的主机自检（make host-test 运行的就是它）
//
// 自检原则（与仓库其它测试一致）
// ---------------------------------------------------------------------------
//   * 只验**能验的东西**：寄存器协议、CRC、偏移、量纲、状态机、CSV 列。
//     凡是要 PL 或相机才能得到的「性能/精度」结论，一律不下结论。
//   * 不编造：模型是 **Python 侧用 src/model/penumbrasight_model/psm.py 生成**的
//     （见 tools/make_host_test_model.py），C 侧只做断言；ExpectedCrC 之类的
//     数字全部来自 Python 写出的 .expected.txt，不许在 C 里写死。
//   * 反例与正例同等重要：篡改魔数/几何哈希/秩/payload/文件长度都必须被**对应的**
//     错误码拒掉，而不是笼统报「模型坏了」。
//
// 覆盖
// ---------------------------------------------------------------------------
//   A 寄存器      §8.2 全表偏移、位定义、4 位全组合往返、Q0.16、64 bit 地址对
//   B 模型        正例 + 7 类反例 + §6.6 出图门 + 与 Python/Zlib 的 CRC 一致性
//   C 取流        RAW Bayer 约束（含反例）、synthetic 与 ps_sources.hpp 逐值一致、
//                 file 与 synthetic 逐值一致
//   D FPGA        影子寄存器默认值、允许集合、错误码、64 bit 地址、超时复位、
//                 真实后端如实返回 Unavailable
//   E 遥测        CSV 表头/列数/确定性、落盘回读、汇总
//   F UI          指令解析、非法参数拒绝、四区/性能页渲染、状态<->配置往返
//
// 用法：
//   host_selftest [模型路径] [期望表路径] [工作目录]
// 默认分别为 ps_tmp/host_test_model.psm / ...expected.txt / ps_tmp
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// ---- 被测模块（C 口径，extern "C"）----
#include "camera_service.h"
#include "fpga_control.h"
#include "model_loader.h"
#include "ps_regs.h"
#include "telemetry.h"
#include "ui_server.h"

// ---- 仓库既有的共享契约：用于「C 模块 vs 冻结 C++ 契约」的交叉比对 ----
#include "ps_contract.hpp"
#include "ps_ipc.hpp"
#include "ps_platform.hpp"
#include "ps_sources.hpp"

namespace {

// ============================================================ 小工具

// Python 侧写出的 key=value 期望表。
struct Expect {
    std::vector<std::string> keys;
    std::vector<uint64_t>    vals;

    bool load(const std::string& path) {
        std::FILE* f = std::fopen(path.c_str(), "rb");
        if (!f) return false;
        char line[256];
        while (std::fgets(line, sizeof(line), f)) {
            char* eq = std::strchr(line, '=');
            if (!eq) continue;
            *eq = '\0';
            const char* k = line;
            const char* v = eq + 1;
            keys.push_back(std::string(k));
            vals.push_back(std::strtoull(v, nullptr, 0));
        }
        std::fclose(f);
        return !keys.empty();
    }
    bool get(const char* key, uint64_t* out) const {
        for (size_t i = 0; i < keys.size(); ++i) {
            if (keys[i] == key) {
                if (out) *out = vals[i];
                return true;
            }
        }
        return false;
    }
};

bool patch_bytes(const std::string& path, long off, const void* src, size_t n) {
    std::FILE* f = std::fopen(path.c_str(), "r+b");
    if (!f) return false;
    if (std::fseek(f, off, SEEK_SET) != 0) { std::fclose(f); return false; }
    const size_t w = std::fwrite(src, 1, n, f);
    std::fclose(f);
    return w == n;
}
bool patch_u16(const std::string& p, long off, uint16_t v) {
    const uint8_t b[2] = {static_cast<uint8_t>(v & 0xFFu),
                          static_cast<uint8_t>((v >> 8) & 0xFFu)};
    return patch_bytes(p, off, b, 2);
}
bool patch_u64(const std::string& p, long off, uint64_t v) {
    uint8_t b[8];
    for (int i = 0; i < 8; ++i) b[i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFFu);
    return patch_bytes(p, off, b, 8);
}

// 「文件被截断」用写一个更短的前缀副本表达，不用 ftruncate/_chsize
//（平台相关，会让主机与板端行为分叉）。
bool copy_prefix(const std::string& src, const std::string& dst, uint64_t bytes) {
    std::FILE* in = std::fopen(src.c_str(), "rb");
    if (!in) return false;
    std::FILE* out = std::fopen(dst.c_str(), "wb");
    if (!out) { std::fclose(in); return false; }
    uint8_t buf[4096];
    uint64_t left = bytes;
    bool ok = true;
    while (left > 0u) {
        const size_t want = static_cast<size_t>(left < sizeof(buf) ? left : sizeof(buf));
        const size_t got = std::fread(buf, 1, want, in);
        if (got == 0u) { ok = false; break; }
        if (std::fwrite(buf, 1, got, out) != got) { ok = false; break; }
        left -= got;
    }
    std::fclose(in);
    std::fclose(out);
    return ok;
}

std::vector<std::string> read_lines(const std::string& path) {
    std::vector<std::string> lines;
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return lines;
    char buf[1024];
    while (std::fgets(buf, sizeof(buf), f)) {
        std::string s(buf);
        while (!s.empty() && (s[s.size() - 1] == '\n' || s[s.size() - 1] == '\r')) {
            s.erase(s.size() - 1);
        }
        lines.push_back(s);
    }
    std::fclose(f);
    return lines;
}

void print_rc(const char* tag, int got, int want) {
    std::printf("        [%s] rc=%d（%s），期望 %d（%s）\n", tag, got,
                ps_model_status_name(got), want, ps_model_status_name(want));
}

// ps_camera_format_t -> ps::RawStreamSpec（只为与冻结契约做比对，不参与生产路径）
ps::RawStreamSpec to_spec(const ps_camera_format_t& f) {
    ps::RawStreamSpec s;
    std::memset(&s, 0, sizeof(s));
    s.width = static_cast<int>(f.width);
    s.height = static_cast<int>(f.height);
    s.container_bits = static_cast<int>(f.container_bits);
    s.color = 1;                       // RAW Bayer 始终走解交织口径
    s.bayer_phase = static_cast<int>(f.bayer_phase);
    s.crop = static_cast<int>(f.crop);
    s.block = static_cast<int>(f.block);
    s.frames = static_cast<int>(f.frames);
    s.dark_level = f.dark_level;
    return s;
}

// ============================================================ A 寄存器

void test_regs(ps::PsTest& t) {
    // §8.2 的 16 个字必须逐字递进，窗口 64 字节。RTL 的地址译码就是照这个写的。
    const uint32_t offs[PS_REG_WORDS] = {
        PS_REG_CONTROL,        PS_REG_STATUS,          PS_REG_FRAME_COUNT,
        PS_REG_ITER_COUNT,     PS_REG_RANK,            PS_REG_CANDIDATE_COUNT,
        PS_REG_SELECTED_ID,    PS_REG_CONFIDENCE,      PS_REG_LATENCY_CYCLES,
        PS_REG_ERROR_CODE,     PS_REG_RAW_ADDR_LO,     PS_REG_RAW_ADDR_HI,
        PS_REG_MODEL_ADDR_LO,  PS_REG_MODEL_ADDR_HI,   PS_REG_OUT_ADDR_LO,
        PS_REG_OUT_ADDR_HI};
    bool consecutive = true;
    for (unsigned i = 0; i < PS_REG_WORDS; ++i) {
        if (offs[i] != i * 4u) consecutive = false;
    }
    t.check(PS_REG_BYTES == 64u && PS_REG_WORDS == 16u, "寄存器窗口 = 16 字 = 64 字节");
    t.check(consecutive, "16 个偏移逐字递进 0x00..0x3C（无空洞）");
    t.check(PS_CTRL_BIT_MASK == 0xFu, "CONTROL 位定义 = bit0..bit3（start/reset/selfloc/rgb）");
    t.check(PS_STS_BIT_MASK == 0xFu, "STATUS 位定义 = bit0..bit3（busy/done/error/low_conf）");
    t.check(PS_REG_DEFAULT_FRAME_COUNT == 4u && PS_REG_DEFAULT_ITER_COUNT == 16u &&
                PS_REG_DEFAULT_RANK == 64u && PS_REG_DEFAULT_CANDIDATE_COUNT == 243u,
            "默认值 = §8.2 表格（4 / 16 / 64 / 243）");

    // 位域往返：4 位全 16 种组合，拼装 -> 解析 -> 逐位比对。
    bool ctrl_rt = true, sts_rt = true, mask_ok = true;
    for (unsigned m = 0u; m < 16u; ++m) {
        ps_reg_control_bits_t cb;
        cb.start   = (m & 1u) ? 1u : 0u;
        cb.reset   = (m & 2u) ? 1u : 0u;
        cb.selfloc = (m & 4u) ? 1u : 0u;
        cb.rgb     = (m & 8u) ? 1u : 0u;
        const uint32_t v = ps_reg_pack_control(&cb);
        uint32_t exp = 0u;
        for (unsigned i = 0u; i < PS_CTRL_BIT_COUNT; ++i) {
            if ((m >> i) & 1u) exp |= ps_reg_control_bit(i);
        }
        if (v != exp) ctrl_rt = false;
        // 未知高位不许被回读成 1
        ps_reg_control_bits_t back;
        ps_reg_unpack_control(v | 0xFFFFFFF0u, &back);
        if (back.start != cb.start || back.reset != cb.reset ||
            back.selfloc != cb.selfloc || back.rgb != cb.rgb) {
            ctrl_rt = false;
        }

        ps_reg_status_bits_t sb;
        sb.busy     = (m & 1u) ? 1u : 0u;
        sb.done     = (m & 2u) ? 1u : 0u;
        sb.error    = (m & 4u) ? 1u : 0u;
        sb.low_conf = (m & 8u) ? 1u : 0u;
        const uint32_t sv = ps_reg_pack_status(&sb);
        uint32_t sexp = 0u;
        for (unsigned i = 0u; i < PS_STS_BIT_COUNT; ++i) {
            if ((m >> i) & 1u) sexp |= ps_reg_status_bit(i);
        }
        if (sv != sexp) sts_rt = false;
        ps_reg_status_bits_t sback;
        ps_reg_unpack_status(sv | 0xFFFFFFF0u, &sback);
        if (sback.busy != sb.busy || sback.done != sb.done ||
            sback.error != sb.error || sback.low_conf != sb.low_conf) {
            sts_rt = false;
        }
        // 拼装结果必须始终落在掩码内（多出任何一位都说明拼装函数写错了）
        if ((v & ~PS_CTRL_BIT_MASK) != 0u || (sv & ~PS_STS_BIT_MASK) != 0u) {
            mask_ok = false;
        }
    }
    t.check(ctrl_rt, "CONTROL 位域拼装/解析往返（4 位全 16 组合）");
    t.check(sts_rt, "STATUS 位域拼装/解析往返（4 位全 16 组合）");
    t.check(mask_ok, "拼装结果不越出 4 位掩码");

    // 允许集合（§5.1/§8.2）：合法值必须过，非法值必须被拒。
    t.check(ps_reg_frame_count_valid(1u) && ps_reg_frame_count_valid(4u) &&
                ps_reg_frame_count_valid(8u) && !ps_reg_frame_count_valid(2u),
            "FRAME_COUNT 允许集合 {1,4,8}");
    t.check(ps_reg_iter_count_valid(8u) && ps_reg_iter_count_valid(12u) &&
                ps_reg_iter_count_valid(16u) && ps_reg_iter_count_valid(32u) &&
                !ps_reg_iter_count_valid(24u),
            "ITER_COUNT 允许集合 {8,12,16,32}");
    t.check(ps_reg_rank_valid(32u) && ps_reg_rank_valid(48u) &&
                ps_reg_rank_valid(64u) && ps_reg_rank_valid(96u) &&
                !ps_reg_rank_valid(40u),
            "RANK 允许集合 {32,48,64,96}");

    // Q0.16：满量程 65536，且不能出现「1.0 差一点点」。
    t.check(ps_reg_q16_to_double(0x8000u) == 0.5, "Q0.16：0x8000 -> 0.5");
    t.check(ps_reg_double_to_q16(0.5) == 0x8000u, "Q0.16 反向：0.5 -> 0x8000");
    t.check(ps_reg_double_to_q16(1.5) == 65535u && ps_reg_double_to_q16(-1.0) == 0u,
            "Q0.16 越界钳位（1.5 -> 0xFFFF，-1 -> 0，不回绕）");
    bool q_rt = true;
    for (uint16_t q = 0u; q < 65535u; q = static_cast<uint16_t>(q + 257u)) {
        if (ps_reg_double_to_q16(ps_reg_q16_to_double(q)) != q) q_rt = false;
    }
    t.check(q_rt, "Q0.16 整数往返（步长 257 扫全量程）");
    bool q_parity = true;
    const uint16_t kQ[6] = {0u, 1u, 0x1234u, 0x8000u, 0xFFFEu, 0xFFFFu};
    for (unsigned i = 0u; i < 6u; ++i) {
        if (ps_reg_q16_to_double(kQ[i]) != ps::ps_q16_to_double(kQ[i])) {
            q_parity = false;
        }
    }
    t.check(q_parity, "Q0.16 换算与共享契约 ps_ipc.hpp 逐值相同");
    t.check(ps_reg_confidence_is_low(0x7FFFu) &&
                !ps_reg_confidence_is_low(0x8000u),
            "低可信度门限比较与门限常量一致");

    // 64 bit 地址对：三对寄存器都是 LO/HI 拆分。
    bool addr_rt = true;
    const uint64_t kAddr[4] = {0ull, 0x123456789ABCDEFull, 0x100000000ull,
                               0xFFFFFFFFFFFFFFFFull};
    for (unsigned i = 0u; i < 4u; ++i) {
        const uint64_t a = kAddr[i];
        if (ps_reg_addr_join(ps_reg_addr_lo(a), ps_reg_addr_hi(a)) != a) addr_rt = false;
    }
    t.check(addr_rt, "64 bit 地址 LO/HI 拆装往返（含跨 4 GB 与全 1）");

    // 短串解码
    char buf[64];
    ps_reg_control_bits_t cb;
    cb.start = 1u; cb.reset = 0u; cb.selfloc = 1u; cb.rgb = 0u;
    const int n1 = ps_reg_decode_control(ps_reg_pack_control(&cb), buf, sizeof(buf));
    t.check(n1 > 0 && std::strstr(buf, "start") != nullptr &&
                std::strstr(buf, "selfloc") != nullptr &&
                std::strstr(buf, "rgb") == nullptr,
            "CONTROL 解码：只列置位的位（start selfloc）");
    const int n2 = ps_reg_decode_status(PS_STS_DONE | PS_STS_LOW_CONF, buf, sizeof(buf));
    t.check(n2 > 0 && std::strstr(buf, "done") != nullptr &&
                std::strstr(buf, "low_conf") != nullptr,
            "STATUS 解码：done low_conf");

    // 周期 <-> 微秒（150 MHz）：100 ms 的定位预算 = 15000000 周期
    t.check(ps_reg_cycles_to_us(15000000ull) == 100000ull,
            "周期->微秒换算（150 MHz：15000000 周期 = 100 ms）");
    t.check(ps_reg_error_name(PS_REG_ERR_NONE) != nullptr &&
                std::strcmp(ps_reg_error_name(PS_REG_ERR_NONE), "无错误") == 0,
            "ERROR_CODE=0 必须是「无错误」");
}

// ============================================================ B 模型

void test_model(ps::PsTest& t, const std::string& model_path,
                const std::string& expected_path, const std::string& workdir) {
    Expect exp;
    const bool have_exp = exp.load(expected_path);
    t.check(have_exp, "读到 Python 侧写出的期望表（.expected.txt）");
    if (!have_exp) return;

    uint64_t e_crc = 0u, e_payload = 0u, e_rec = 0u, e_file = 0u, e_geom = 0u;
    (void)exp.get("payload_crc32", &e_crc);
    (void)exp.get("payload_bytes", &e_payload);
    (void)exp.get("record_bytes", &e_rec);
    (void)exp.get("file_bytes", &e_file);
    (void)exp.get("geometry_hash", &e_geom);
    std::printf("        [py] crc=%08llX payload=%llu record=%llu geom=%08llX\n",
                (unsigned long long)e_crc, (unsigned long long)e_payload,
                (unsigned long long)e_rec, (unsigned long long)e_geom);

    t.check(ps_crc32_selftest() == 0, "psm_crc32.c 自检（标准向量 + 分块累加）");

    const ps_model_shape_t shape = ps_model_shape_make(4u, 3u, 2u, 2u);

    // ---- 正例 ----
    ps_model_t m;
    std::memset(&m, 0, sizeof(m));
    int rc = ps_model_load(model_path.c_str(), static_cast<uint32_t>(e_geom), &shape, &m);
    print_rc("正例", rc, PS_MODEL_OK);
    t.check(rc == PS_MODEL_OK, "Python 生成的小模型装载成功");
    t.check(m.payload_bytes == e_payload, "payload 长度与 Python 声明一致");
    t.check(m.file_bytes == e_file && e_file == PS_PSM_HEADER_BYTES + e_payload,
            "文件长度 = 64 + payload_bytes");
    t.check(m.crc_computed == static_cast<uint32_t>(e_crc),
            "CRC32 实算 == Python zlib.crc32（跨语言一致）");
    t.check(m.crc_computed == m.crc_expected, "CRC 实算 == 头部声明");
    t.check(ps_psm_record_bytes_hdr(&m.header) == e_rec,
            "单候选记录长度与 psm.py 的 array_offsets 推导一致（4096 对齐）");
    t.check(m.header.meas_h == 4u && m.header.meas_w == 3u &&
                m.header.hidden_h == 2u && m.header.hidden_w == 2u &&
                m.header.candidates == 3u && m.header.rank_max == 4u,
            "头部尺寸组合与 Python 一致");
    t.check(m.payload_capacity == ps_psm_align_up(e_payload, PS_PSM_ARRAY_ALIGN),
            "装载区按 4 KB 向上对齐");
    t.check(m.payload != nullptr &&
                (reinterpret_cast<uintptr_t>(m.payload) % PS_PSM_ARRAY_ALIGN) == 0,
            "装载区首址 4 KB 对齐（DMA 突发的前提）");
    t.check(m.is_stub == 1 && m.payload_phys == 0u,
            "主机桩件物理地址显式为 0（不伪装成假地址）");
    t.check(m.geometry_verified == 1, "几何哈希比对过且一致");
    t.check(ps_model_render_allowed(&m) == 1, "§6.6 出图门：允许出图");
    ps_model_print(&m);
    ps_model_unload(&m);
    t.check(m.payload == nullptr && m.loaded_payload == 0, "卸载后装载区指针归零");

    // ---- 只读头部路径（板端启动路径）----
    {
        ps_psm_header_t h;
        uint64_t fb = 0u;
        const int hrc = ps_model_read_header(model_path.c_str(),
                                             static_cast<uint32_t>(e_geom), &shape,
                                             &h, &fb);
        t.check(hrc == PS_MODEL_OK && fb == e_file, "只读 64 字节头部路径通过");
    }

    // ---- 反例 1：几何哈希不符（§6.6 必须拒，且必须只读头部就拒）----
    {
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        const int got = ps_model_load(model_path.c_str(), 0x11111111u, &shape, &h);
        print_rc("几何哈希不符", got, PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_GEOM_HASH);
        t.check(got == PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_GEOM_HASH,
                "几何哈希不符被拒（错误码精确到 GEOM_HASH）");
        t.check(h.payload == nullptr && h.loaded_payload == 0,
                "哈希不符时不申请装载区、不搬 payload");
        t.check(ps_model_render_allowed(&h) == 0, "§6.6 出图门：拒绝出图");
    }

    // ---- 反例 2：魔数被改 ----
    {
        const std::string p = workdir + "/bad_magic.psm";
        t.check(copy_prefix(model_path, p, e_file), "（准备）拷贝模型");
        t.check(patch_bytes(p, static_cast<long>(PS_PSM_OFF_MAGIC), "PSM2", 4),
                "（准备）篡改魔数");
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        const int got = ps_model_load(p.c_str(), static_cast<uint32_t>(e_geom), &shape, &h);
        print_rc("魔数错", got, PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_MAGIC);
        t.check(got == PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_MAGIC, "魔数错被拒");
    }

    // ---- 反例 3：秩关系被破坏（loc_rank = 200 > rank_max = 4）----
    {
        const std::string p = workdir + "/bad_rank.psm";
        copy_prefix(model_path, p, e_file);
        t.check(patch_u16(p, 22, 200u), "（准备）篡改 loc_rank = 200");
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        const int got = ps_model_load(p.c_str(), static_cast<uint32_t>(e_geom), &shape, &h);
        print_rc("秩关系破坏", got, PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_RANK);
        t.check(got == PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_RANK, "秩关系被破坏被拒");
    }

    // ---- 反例 4：payload 被改一个字节 -> CRC 必须抓到，且装载区要还回去 ----
    {
        const std::string p = workdir + "/bad_crc.psm";
        copy_prefix(model_path, p, e_file);
        const long off = static_cast<long>(PS_PSM_HEADER_BYTES + 4096u);
        const uint8_t bad = 0x5Au;
        t.check(patch_bytes(p, off, &bad, 1), "（准备）翻转 payload 一个字节");
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        const int got = ps_model_load(p.c_str(), static_cast<uint32_t>(e_geom), &shape, &h);
        print_rc("payload 篡改", got, PS_MODEL_ERR_CRC);
        t.check(got == PS_MODEL_ERR_CRC, "payload 篡改被 CRC32 抓到");
        t.check(h.payload == nullptr, "CRC 失败后装载区已释放（286 MB 不泄漏）");
    }

    // ---- 反例 5：payload_bytes 与候选数不符（头部自检抓）----
    {
        const std::string p = workdir + "/bad_len.psm";
        copy_prefix(model_path, p, e_file);
        t.check(patch_u64(p, static_cast<long>(PS_PSM_OFF_PAYLOAD_BYTES), e_payload + 1u),
                "（准备）把 payload_bytes 改大 1");
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        const int got = ps_model_load(p.c_str(), static_cast<uint32_t>(e_geom), &shape, &h);
        print_rc("payload_bytes 不符", got, PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_SIZE);
        t.check(got == PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_SIZE,
                "payload 长度自检失败被拒（头部自洽性）");
    }

    // ---- 反例 6：文件被物理截短 1 字节（文件长度检查抓）----
    {
        const std::string cut = workdir + "/bad_cut.psm";
        t.check(copy_prefix(model_path, cut, e_file - 1u), "（准备）写少 1 字节的前缀副本");
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        const int got = ps_model_load(cut.c_str(), static_cast<uint32_t>(e_geom), &shape, &h);
        print_rc("文件截短", got, PS_MODEL_ERR_SIZE);
        t.check(got == PS_MODEL_ERR_SIZE, "文件被截短被拒（文件长度检查，不是 CRC）");
    }

    // ---- 反例 7：几何哈希「未检查」不等于「通过」----
    {
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        const int got = ps_model_load(model_path.c_str(), 0u, &shape, &h);
        t.check(got == PS_MODEL_OK, "expect_hash=0 时装载通过（仅调试口径）");
        t.check(ps_model_render_allowed(&h) == 0,
                "但 §6.6 出图门仍然拒绝（「没验过」≠「验过且一致」）");
        ps_model_unload(&h);
    }

    // ---- 反例 8：形状不符（拿 B 档 95x96->18x24 去装 4x3->2x2）----
    {
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        const ps_model_shape_t b = ps_model_shape_b();
        const int got = ps_model_load(model_path.c_str(), static_cast<uint32_t>(e_geom),
                                      &b, &h);
        print_rc("形状不符", got, PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_SHAPE);
        t.check(got == PS_MODEL_ERR_HEADER_BASE + PS_PSM_ERR_SHAPE,
                "形状不符被拒（防止「读得进来但语义整体错位」）");
    }

    // ---- 反例 9：打不开的文件 / 非 ASCII 路径 ----
    {
        ps_model_t h;
        std::memset(&h, 0, sizeof(h));
        t.check(ps_model_load((workdir + "/no_such.psm").c_str(), 0u, &shape, &h) ==
                    PS_MODEL_ERR_OPEN,
                "文件不存在被拒");
        // 用转义字节写 UTF-8 的「中文/模型.psm」，避免源码编码影响用例本身
        const std::string cn = "\xE4\xB8\xAD\xE6\x96\x87/\xE6\xA8\xA1\xE5\x9E\x8B.psm";
        t.check(ps_model_load(cn.c_str(), 0u, &shape, &h) == PS_MODEL_ERR_OPEN,
                "非 ASCII 路径被显式拒绝（不乱码、不静默）");
    }
}

// ============================================================ C 取流

void test_camera(ps::PsTest& t, const std::string& workdir) {
    // B 档：1920 裁剪 / block 10 -> 960/10 = 96（与 ps_contract.hpp 同口径）
    ps_camera_format_t b;
    std::memset(&b, 0, sizeof(b));
    b.width = 2016u; b.height = 2016u; b.container_bits = 12u; b.bayer_phase = 1u;
    b.bytes_per_pixel = 2u; b.interleaved_rgb = 0u;
    b.crop = 1920u; b.block = 10u; b.frames = 4u; b.dark_level = 16u;

    // 官方档：2016 裁剪 / block 8 -> 1008/8 = 126（两档 crop 不同，不得混用）
    ps_camera_format_t official = b;
    official.crop = 2016u; official.block = 8u;

    char reason[192];
    reason[0] = '\0';
    t.check(ps_camera_format_check(&b, reason, sizeof(reason)) == PS_CAM_OK,
            "RAW12 Bayer 格式通过检查");
    t.check(ps_camera_wall_side(&b) == 96u, "B 档推出的墙网格边长 = 96");
    t.check(ps_camera_wall_side(&official) == 126u, "官方档推出的墙网格边长 = 126");

    // 与冻结契约 ps_contract.hpp 的交叉比对：同规格必须推出同一个数，
    // 且校验结论也一致（我这里只实现了一份，错了必须当场发现）。
    const ps::RawStreamSpec sb = to_spec(b);
    const ps::RawStreamSpec so = to_spec(official);
    t.check(ps::ps_wall_side(sb) == static_cast<int>(ps_camera_wall_side(&b)) &&
                ps::ps_wall_side(so) == static_cast<int>(ps_camera_wall_side(&official)),
            "墙网格边长与 ps_contract.hpp 的 ps_wall_side 一致");
    t.check(ps::ps_validate_spec(sb, 96, 16) == ps::kPsOk &&
                ps::ps_validate_spec(so, 126, 16) == ps::kPsOk,
            "同规格在 ps_validate_spec 下也判定合法");
    t.check(ps::ps_validate_spec(sb, 126, 16) == ps::kPsSideMismatch &&
                ps_camera_geometry_check(&b, 126u, reason, sizeof(reason)) ==
                    PS_CAM_ERR_FORMAT,
            "B 档规格配 126 墙网格：两边都判为「边长不符」");

    // ---- RAW Bayer 红线的反例 ----
    {
        ps_camera_format_t bad = b;
        bad.interleaved_rgb = 1u;
        t.check(ps_camera_format_check(&bad, reason, sizeof(reason)) == PS_CAM_ERR_FORMAT &&
                    std::strstr(reason, "8bit RGB") != nullptr,
                "已解交织的 RGB 格式被拒（SDK 不得先做 8bit RGB 转换）");
        bad = b; bad.bytes_per_pixel = 1u;
        t.check(ps_camera_format_check(&bad, reason, sizeof(reason)) == PS_CAM_ERR_FORMAT,
                "每像素 1 字节（8bit 截断）被拒");
        bad = b; bad.bytes_per_pixel = 3u;
        t.check(ps_camera_format_check(&bad, reason, sizeof(reason)) == PS_CAM_ERR_FORMAT,
                "每像素 3 字节（RGB8 解马赛克）被拒");
        bad = b; bad.container_bits = 10u;
        t.check(ps_camera_format_check(&bad, reason, sizeof(reason)) == PS_CAM_ERR_FORMAT,
                "容器位深 10（不在 {8,12,16}）被拒");
        bad = b; bad.crop = 1919u;
        t.check(ps_camera_format_check(&bad, reason, sizeof(reason)) == PS_CAM_ERR_FORMAT,
                "奇数裁剪边长被拒（解交织要求偶数）");
        bad = b; bad.crop = 1918u;
        t.check(ps_camera_format_check(&bad, reason, sizeof(reason)) == PS_CAM_ERR_FORMAT,
                "裁剪边长不能被 2*block 整除被拒（M1 会静默取整）");
        bad = b; bad.block = 0u;
        t.check(ps_camera_format_check(&bad, reason, sizeof(reason)) == PS_CAM_ERR_FORMAT,
                "分块边长 0 被拒");
    }

    // ---- synthetic 源与 ps_sources.hpp 逐值一致 ----
    ps_camera_format_t small;
    std::memset(&small, 0, sizeof(small));
    small.width = 64u; small.height = 64u; small.container_bits = 12u;
    small.bayer_phase = 0u; small.bytes_per_pixel = 2u; small.interleaved_rgb = 0u;
    small.crop = 64u; small.block = 8u; small.frames = 2u; small.dark_level = 16u;

    ps_camera_capture_t cap;
    std::memset(&cap, 0, sizeof(cap));
    const int orc = ps_camera_open(&cap, PS_CAM_IMPL_SYNTHETIC, &small, nullptr);
    t.check(orc == PS_CAM_OK && cap.confirmed_raw_bayer == 1,
            "synthetic 源打开（RAW 口径已确认）");

    const uint64_t per_frame = ps_camera_frame_pixels(&small);
    const uint64_t total = per_frame * small.frames;
    std::vector<uint16_t> buf_syn(static_cast<size_t>(total), 0u);
    const long long got_syn = ps_camera_pump(&cap, static_cast<unsigned>(small.frames),
                                             buf_syn.data());
    t.check(got_syn == static_cast<long long>(total) &&
                cap.frames_served == small.frames && cap.dropped_frames == 0u,
            "synthetic 源灌满 2 帧（无丢帧）");

    {
        const ps::RawStreamSpec ss = to_spec(small);
        bool same = true;
        for (uint64_t k = 0u; k < total; ++k) {
            const uint16_t want = ps::ps_sim_pixel(ss, cap.seed, static_cast<long long>(k));
            if (buf_syn[static_cast<size_t>(k)] != want) { same = false; break; }
        }
        t.check(same, "synthetic 像素与 ps_sources.hpp 的 ps_sim_pixel 逐值相同");
    }

    // ---- file 源与 synthetic 逐值一致 ----
    const std::string raw_path = workdir + "/host_test_raw.bin";
    t.check(ps_camera_write_raw_frames(raw_path.c_str(), buf_syn.data(), total) == PS_CAM_OK,
            "裸帧文件写出（无头、uint16 小端、帧主序）");

    ps_camera_capture_t fcap;
    std::memset(&fcap, 0, sizeof(fcap));
    const int frc = ps_camera_open(&fcap, PS_CAM_IMPL_FILE, &small, raw_path.c_str());
    t.check(frc == PS_CAM_OK && fcap.total_pixels == total,
            "file 源打开并从文件长度推断出整数帧");
    if (frc == PS_CAM_OK) {
        std::vector<uint16_t> buf_file(static_cast<size_t>(total), 0u);
        const long long got_file =
            ps_camera_pump(&fcap, static_cast<unsigned>(small.frames), buf_file.data());
        bool same = (got_file == static_cast<long long>(total));
        if (same) {
            for (uint64_t k = 0u; k < total; ++k) {
                if (buf_file[static_cast<size_t>(k)] != buf_syn[static_cast<size_t>(k)]) {
                    same = false; break;
                }
            }
        }
        t.check(same, "file 与 synthetic 两条路逐值相同（同一接口的实质）");

        // rewind 后重取必须与第一次相同（位置无关哈希的直接推论）
        t.check(ps_camera_rewind(&fcap) == PS_CAM_OK, "file 源 rewind");
        std::vector<uint16_t> buf_again(static_cast<size_t>(total), 0u);
        const long long got_again =
            ps_camera_pump(&fcap, static_cast<unsigned>(small.frames), buf_again.data());
        bool again_same = (got_again == static_cast<long long>(total));
        if (again_same) {
            for (uint64_t k = 0u; k < total; ++k) {
                if (buf_again[static_cast<size_t>(k)] != buf_syn[static_cast<size_t>(k)]) {
                    again_same = false; break;
                }
            }
        }
        t.check(again_same, "rewind 后重取逐值相同");
    }

    // 曝光记账（主机没有真实曝光，只允许「记账」，不许假装是实测）
    t.check(cap.set_exposure_us != nullptr &&
                cap.set_exposure_us(cap.ctx, 1250) == PS_CAM_OK &&
                cap.get_exposure_us != nullptr &&
                cap.get_exposure_us(cap.ctx) == 1250,
            "曝光设置/回读（主机记账口径）");
    ps_camera_close(&cap);
    ps_camera_close(&fcap);

    // ---- 留桩实现必须如实不可用 ----
    {
        ps_camera_capture_t sc;
        std::memset(&sc, 0, sizeof(sc));
        const int src = ps_camera_open(&sc, PS_CAM_IMPL_SPINNAKER, &b, "SN-0001");
        t.check(src == PS_CAM_UNAVAILABLE, "Spinnaker 未接入：打开返回 Unavailable");
        std::vector<uint16_t> tmp(static_cast<size_t>(per_frame), 0u);
        t.check(ps_camera_pump(&sc, 1u, tmp.data()) == PS_CAM_UNAVAILABLE,
                "Spinnaker 拉流返回不可用（不假装成功）");
        ps_camera_capture_t gc;
        std::memset(&gc, 0, sizeof(gc));
        t.check(ps_camera_open(&gc, PS_CAM_IMPL_GENTL, &b, "GEV-0001") ==
                    PS_CAM_UNAVAILABLE,
                "GenTL 未接入：打开返回 Unavailable");
        t.check(ps_camera_impl_from_name("synthetic") == PS_CAM_IMPL_SYNTHETIC &&
                    ps_camera_impl_from_name("nope") == -1,
                "实现名解析");
    }
}

// ============================================================ D FPGA

void test_fpga(ps::PsTest& t) {
    ps_fpga_t dev;
    std::memset(&dev, 0, sizeof(dev));
    t.check(ps_fpga_open(&dev, "stub") == PS_FPGA_OK &&
                ps_fpga_is_stub(&dev) == 1 &&
                std::strcmp(ps_fpga_backend_name(&dev), "stub") == 0,
            "影子寄存器后端打开（主机桩件）");

    uint32_t v = 0u;
    ps_fpga_read32(&dev, PS_REG_FRAME_COUNT, &v);
    t.check(v == PS_REG_DEFAULT_FRAME_COUNT, "复位后 FRAME_COUNT = 4");
    ps_fpga_read32(&dev, PS_REG_ITER_COUNT, &v);
    t.check(v == PS_REG_DEFAULT_ITER_COUNT, "复位后 ITER_COUNT = 16");
    ps_fpga_read32(&dev, PS_REG_RANK, &v);
    t.check(v == PS_REG_DEFAULT_RANK, "复位后 RANK = 64");
    ps_fpga_read32(&dev, PS_REG_CANDIDATE_COUNT, &v);
    t.check(v == PS_REG_DEFAULT_CANDIDATE_COUNT, "复位后 CANDIDATE_COUNT = 243");

    t.check(ps_fpga_write32(&dev, PS_REG_SELECTED_ID, 0x1234u) == PS_FPGA_OK,
            "写 SELECTED_ID");
    ps_fpga_read32(&dev, PS_REG_SELECTED_ID, &v);
    t.check(v == 0x1234u, "读回一致");
    t.check(ps_fpga_write32(&dev, PS_REG_BYTES, 1u) == PS_FPGA_ERR_IO,
            "越界偏移（>= 64 B）被拒");
    t.check(ps_fpga_write32(&dev, 0x02u, 1u) == PS_FPGA_ERR_IO,
            "非 4 字节对齐偏移被拒");

    const uint64_t a64 = 0x123456789ull;
    // 写序判据（A3）：64 位地址对必须**先写 HI(+4) 再写 LO(+0)**。
    // 影子寄存器只存最终值看不出顺序，故读影子后端记下的写序列来断言。
    ps_fpga_stub_trace_reset(&dev);
    t.check(ps_fpga_write_addr64(&dev, PS_REG_MODEL_ADDR_LO, a64) == PS_FPGA_OK,
            "写 64 bit 地址对");
    {
        ps_fpga_write_ev_t ev[2];
        const int n = ps_fpga_stub_write_trace(&dev, ev, 2);
        const bool order_ok =
            (n == 2 &&
             ev[0].off == PS_REG_MODEL_ADDR_HI &&
             ev[1].off == PS_REG_MODEL_ADDR_LO &&
             ev[0].val == static_cast<uint32_t>(a64 >> 32) &&
             ev[1].val == static_cast<uint32_t>(a64 & 0xFFFFFFFFu));
        t.check(order_ok, "64 bit 写序 = [HI, LO]（与 board/fix_core 同一纪律）");
    }
    uint64_t back = 0u;
    ps_fpga_read_addr64(&dev, PS_REG_MODEL_ADDR_LO, &back);
    t.check(back == a64, "读回 64 bit 地址一致（LO/HI 拆分正确）");

    ps_reg_job_config_t cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.frame_count = 4u; cfg.iter_count = 16u; cfg.rank = 64u;
    cfg.candidate_count = PS_REG_DEFAULT_CANDIDATE_COUNT;
    cfg.selfloc = 1u; cfg.rgb = 1u;

    // 非法参数：rank = 40 不在允许集合
    {
        ps_reg_job_config_t bad = cfg;
        bad.rank = 40u;
        t.check(ps_fpga_configure(&dev, &bad) == PS_FPGA_ERR_BAD_PARAM,
                "rank=40 被拒（不在允许集合）");
        ps_fpga_read32(&dev, PS_REG_ERROR_CODE, &v);
        t.check(v == PS_REG_ERR_ILLEGAL_PARAM, "ERROR_CODE = 参数非法");
        ps_fpga_read32(&dev, PS_REG_STATUS, &v);
        t.check((v & PS_STS_ERROR) != 0u, "STATUS.error 置位");
    }

    // CONTROL 位按配置置位
    {
        ps_reg_job_config_t c2 = cfg;
        t.check(ps_fpga_configure(&dev, &c2) == PS_FPGA_OK, "合法配置被接受");
        ps_fpga_read32(&dev, PS_REG_CONTROL, &v);
        t.check((v & PS_CTRL_SELFLOC) != 0u && (v & PS_CTRL_RGB) != 0u,
                "CONTROL.selfloc 与 .rgb 按配置置位");
    }

    ps_reg_buffer_addrs_t addrs;
    addrs.raw = 8192u; addrs.model = 4096u; addrs.output = 12288u;

    // 模型未装载 / 未对齐都必须被拒
    {
        ps_reg_job_result_t r;
        ps_reg_buffer_addrs_t a = addrs;
        a.model = 0u;
        t.check(ps_fpga_submit(&dev, &cfg, &a, &r, 50) == PS_FPGA_ERR_BAD_PARAM,
                "模型地址 0 被拒");
        ps_fpga_read32(&dev, PS_REG_ERROR_CODE, &v);
        t.check(v == PS_REG_ERR_MODEL_NOT_LOADED, "ERROR_CODE = 模型未装载");
        a = addrs; a.model = 4096u + 8u;
        t.check(ps_fpga_submit(&dev, &cfg, &a, &r, 50) == PS_FPGA_ERR_BAD_PARAM,
                "模型地址未 4 KB 对齐被拒");
    }

    // 自定位作业
    ps_reg_job_result_t res;
    std::memset(&res, 0, sizeof(res));
    const int src = ps_fpga_submit(&dev, &cfg, &addrs, &res, 50);
    t.check(src == PS_FPGA_OK, "自定位作业提交成功");
    t.check(res.selected_id < PS_REG_DEFAULT_CANDIDATE_COUNT,
            "selected_id 落在候选范围内");
    t.check(res.error_code == 0u, "ERROR_CODE = 0");
    t.check(res.latency_cycles > 0u &&
                res.latency_us == ps_reg_cycles_to_us(res.latency_cycles),
            "latency 周期与微秒换算自洽（150 MHz）");
    t.check(res.output_bytes ==
                static_cast<uint64_t>(PS_PSM_HIDDEN_H) * PS_PSM_HIDDEN_W * 4u * 3u,
            "重建输出缓冲 = 24x18x3x4 字节（RGB）");
    ps_fpga_read32(&dev, PS_REG_STATUS, &v);
    t.check((v & PS_STS_DONE) != 0u && (v & PS_STS_BUSY) == 0u,
            "STATUS = done 且 busy 已落");
    t.check(ps_fpga_irq_count(&dev) >= 1u, "中断被消费并计数");

    // 固定位置模式
    {
        ps_reg_job_config_t c3 = cfg;
        c3.selfloc = 0u;
        c3.fixed_candidate_id = 17;
        ps_reg_job_result_t r;
        t.check(ps_fpga_submit(&dev, &c3, &addrs, &r, 50) == PS_FPGA_OK &&
                    r.selected_id == 17u,
                "固定位置模式 selected_id = 17");
        ps_fpga_read32(&dev, PS_REG_SELECTED_ID, &v);
        t.check(v == 17u, "SELECTED_ID 寄存器 = 17");
    }

    // low_conf 门限比较必须真实执行（两种结果都要出现），否则说明 flags 恒 0
    {
        int low = 0, high = 0;
        for (int id = 0; id < 243; id += 7) {
            ps_reg_job_config_t c4 = cfg;
            c4.selfloc = 0u;
            c4.fixed_candidate_id = id;
            ps_reg_job_result_t r;
            if (ps_fpga_submit(&dev, &c4, &addrs, &r, 50) != PS_FPGA_OK) continue;
            if ((r.flags & PS_REG_JOB_FLAG_LOW_CONF) != 0u) ++low; else ++high;
        }
        t.check(low > 0 && high > 0, "low_conf 门限比较真实执行（两种结果都出现）");
    }

    // 超时 + 复位
    {
        ps_reg_job_config_t c5 = cfg;
        c5.flags = PS_JOB_CFG_FLAG_INJECT_TIMEOUT;
        ps_reg_job_result_t r;
        const uint64_t to_before = ps_fpga_timeout_count(&dev);
        const int got = ps_fpga_submit(&dev, &c5, &addrs, &r, 20);
        t.check(got == PS_FPGA_ERR_TIMEOUT, "注入超时返回超时错误码");
        t.check(ps_fpga_timeout_count(&dev) == to_before + 1u, "超时被计数");
        ps_fpga_read32(&dev, PS_REG_STATUS, &v);
        t.check((v & PS_STS_BUSY) != 0u, "超时后 PL 仍 busy（未伪装成完成）");
        const uint64_t rs_before = ps_fpga_reset_count(&dev);
        t.check(ps_fpga_reset(&dev) == PS_FPGA_OK &&
                    ps_fpga_reset_count(&dev) == rs_before + 1u,
                "复位成功并计数");
        ps_fpga_read32(&dev, PS_REG_STATUS, &v);
        t.check(v == 0u, "复位后 STATUS 清 0（busy 已落）");
        ps_fpga_read32(&dev, PS_REG_FRAME_COUNT, &v);
        t.check(v == cfg.frame_count, "复位保留配置寄存器（帧数不变）");
    }

    // 整表打印
    {
        char dump[2048];
        const int n = ps_fpga_dump_regs(&dev, dump, sizeof(dump));
        t.check(n > 0 && std::strstr(dump, "CONTROL") != nullptr &&
                    std::strstr(dump, "MODEL_ADDR_LO") != nullptr &&
                    std::strstr(dump, "ERROR_CODE") != nullptr,
                "§8.2 寄存器表整表可打印（上板联调第一页）");
    }

    ps_fpga_close(&dev);

    // 真实后端必须如实不可用
    {
        ps_fpga_t real;
        std::memset(&real, 0, sizeof(real));
        t.check(ps_fpga_open(&real, "/dev/uio0") == PS_FPGA_ERR_UNAVAILABLE,
                "UIO 后端留桩：返回 Unavailable（不假装成功）");
        t.check(ps_fpga_backend_uio() != nullptr && ps_fpga_backend_xrt() != nullptr &&
                    ps_fpga_backend_stub() != nullptr,
                "三个后端实例都存在（同一张函数指针表）");
        ps_fpga_t xrt;
        std::memset(&xrt, 0, sizeof(xrt));
        t.check(ps_fpga_open(&xrt, "xrt:0") == PS_FPGA_ERR_UNAVAILABLE,
                "XRT 后端留桩：返回 Unavailable");
    }
}

// ============================================================ E 遥测

void test_telemetry(ps::PsTest& t, const std::string& workdir) {
    char hdr[256];
    const int hn = ps_telemetry_format_header(hdr, sizeof(hdr));
    t.check(hn > 0 && ps_telemetry_row_columns(hdr) == PS_TELEM_COLUMNS,
            "CSV 表头列数 = 17（列顺序即契约）");

    ps_telemetry_row_t r;
    ps_telemetry_row_init(&r);
    r.seq = 7u;
    r.t_mono_us = 1234567u;
    r.frame_rate_milli = 30123u;
    r.exposure_us = 2000;
    r.residual_q16 = 0x0123u;
    // 顺序按 telemetry.h 的契约：先灌作业结果（PL 回读的整批字段），
    // 再用 set_scores 补候选分数与由分数算出的 margin（后调用者胜）。
    ps_reg_job_result_t jr;
    std::memset(&jr, 0, sizeof(jr));
    jr.selected_id = 42u;
    jr.rank = 64u; jr.iterations = 16u;
    jr.latency_cycles = 15000000ull;
    jr.latency_us = 100000ull;
    ps_telemetry_row_from_result(&r, &jr);
    (void)ps_telemetry_set_scores(&r, 0x8000u, 0x4000u);
    ps_telemetry_set_power(&r, 41500, 3120u);
    t.check(r.selected_id == 42u && r.rank == 64u && r.iterations == 16u,
            "作业结果字段灌入（selected_id / rank / iterations）");

    char line1[512], line2[512];
    t.check(ps_telemetry_format_row(&r, line1, sizeof(line1)) > 0 &&
                ps_telemetry_format_row(&r, line2, sizeof(line2)) > 0 &&
                std::strcmp(line1, line2) == 0,
            "同一行渲染两次逐字节相同（格式确定）");
    t.check(ps_telemetry_row_columns(line1) == PS_TELEM_COLUMNS,
            "数据行列数 = 17");
    t.check(r.margin_q16 > 0u,
            "margin = (s1-s2)/(s1+eps) 落在 Q0.16 内（非 0 且不回绕）");
    t.check(std::strstr(line1, "41500") != nullptr &&
                std::strstr(line1, "3120") != nullptr,
            "温度/功耗原样落到对应列（主机为记账值，非实测）");
    {
        ps_telemetry_row_t bad = r;
        const uint16_t m0 = ps_telemetry_set_scores(&bad, 0x4000u, 0x8000u);
        t.check(m0 == 0u, "次佳分数反过来时 margin 记 0（不回绕成巨大正数）");
    }

    // 落盘 + 回读
    const std::string csv = workdir + "/host_test_telemetry.csv";
    ps_telemetry_t tm;
    std::memset(&tm, 0, sizeof(tm));
    t.check(ps_telemetry_open(&tm, csv.c_str()) == PS_TELEM_OK, "打开 CSV 并写表头");
    for (int i = 0; i < 5; ++i) {
        ps_telemetry_row_t row = r;
        row.seq = static_cast<uint64_t>(i);
        row.t_mono_us = 1000000u + static_cast<uint64_t>(i) * 100000u;   // 10 fps
        row.latency_us = 90000u + static_cast<uint64_t>(i) * 1000u;
        row.confidence_q16 = static_cast<uint16_t>(0x9000u - i * 0x100u);
        t.check(ps_telemetry_write(&tm, &row) == PS_TELEM_OK, "写一行遥测");
    }
    ps_telemetry_close(&tm);
    t.check(tm.rows == 5u, "写出行数 = 5");

    const std::vector<std::string> lines = read_lines(csv);
    t.check(lines.size() == 6u, "CSV 回读 6 行（1 表头 + 5 数据）");
    bool cols_ok = !lines.empty() &&
                   ps_telemetry_row_columns(lines[0].c_str()) == PS_TELEM_COLUMNS;
    for (size_t i = 1; i < lines.size(); ++i) {
        if (ps_telemetry_row_columns(lines[i].c_str()) != PS_TELEM_COLUMNS) {
            cols_ok = false;
        }
    }
    t.check(cols_ok, "回读的每一行都是 17 列（无列错位）");

    ps_telemetry_summary_t sum;
    std::memset(&sum, 0, sizeof(sum));
    t.check(ps_telemetry_summary(&tm, &sum) == PS_TELEM_OK && sum.rows == 5u &&
                sum.latency_us_max == 94000u && sum.confidence_min_q16 == 0x8C00u,
            "汇总：行数 / 最大延迟 / 最小可信度");
    t.check(sum.span_us == 400000u &&
                (sum.fps_avg > 12.4 && sum.fps_avg < 12.6),
            "汇总：时间跨度 0.4 s -> 平均帧率 ≈ 12.5 fps（样例数据）");
}

// ============================================================ F UI

void test_ui(ps::PsTest& t) {
    ps_ui_state_t s;
    ps_ui_state_init(&s);
    t.check(s.selfloc == 1u && s.rgb == 1u && s.frames == 4u && s.iters == 16u &&
                s.rank == 64u && s.candidate_count == 243u && s.page == PS_UI_PAGE_MAIN,
            "初始状态 = §8.2 默认档（自定位 / RGB / 4 帧 / 16 迭代 / rank 64）");

    char out[2048];
    t.check(ps_ui_exec(&s, "loc fixed", out, sizeof(out)) == PS_UI_OK && s.selfloc == 0u,
            "指令 loc fixed -> 固定位置");
    t.check(ps_ui_exec(&s, "selfloc", out, sizeof(out)) == PS_UI_OK && s.selfloc == 1u,
            "简写 selfloc -> 自定位");
    t.check(ps_ui_exec(&s, "color mono", out, sizeof(out)) == PS_UI_OK && s.rgb == 0u,
            "指令 color mono -> 灰度");
    t.check(ps_ui_exec(&s, "colour=rgb", out, sizeof(out)) == PS_UI_OK && s.rgb == 1u,
            "别名 colour=rgb 与 '=' 分隔符都认");
    t.check(ps_ui_exec(&s, "frames 8", out, sizeof(out)) == PS_UI_OK && s.frames == 8u,
            "指令 frames 8");
    t.check(ps_ui_exec(&s, "iters 32", out, sizeof(out)) == PS_UI_OK && s.iters == 32u,
            "指令 iters 32");
    t.check(ps_ui_exec(&s, "rank 96", out, sizeof(out)) == PS_UI_OK && s.rank == 96u,
            "指令 rank 96");

    // 非法参数：必须拒绝且不改状态（宽容处理会让「设了 5 跑的是 4」永远没人发现）
    {
        const uint32_t rev_before = s.rev;
        t.check(ps_ui_exec(&s, "frames 5", out, sizeof(out)) == PS_UI_ERR_BAD_ARG &&
                    s.frames == 8u && s.rev == rev_before && s.rejected == 1u,
                "frames 5 被拒且状态不变（允许集合 {1,4,8}）");
        t.check(ps_ui_exec(&s, "rank 40", out, sizeof(out)) == PS_UI_ERR_BAD_ARG &&
                    s.rank == 96u,
                "rank 40 被拒（允许集合 {32,48,64,96}）");
        t.check(ps_ui_exec(&s, "iters 24", out, sizeof(out)) == PS_UI_ERR_BAD_ARG,
                "iters 24 被拒（允许集合 {8,12,16,32}）");
        t.check(ps_ui_exec(&s, "frames x", out, sizeof(out)) == PS_UI_ERR_BAD_ARG,
                "非数字参数被拒");
        t.check(ps_ui_exec(&s, "frobnicate", out, sizeof(out)) ==
                    PS_UI_ERR_UNKNOWN_CMD,
                "未知指令返回 UNKNOWN_CMD");
        t.check(s.rejected == 5u, "被拒指令计数 = 5");
    }

    // 候选数与固定编号
    t.check(ps_ui_exec(&s, "cand 100", out, sizeof(out)) == PS_UI_OK &&
                s.candidate_count == 100u,
            "指令 cand 100");
    t.check(ps_ui_exec(&s, "fixed-id 99", out, sizeof(out)) == PS_UI_OK &&
                s.fixed_candidate_id == 99,
            "指令 fixed-id 99（< 候选数）");
    t.check(ps_ui_exec(&s, "fixed-id 100", out, sizeof(out)) == PS_UI_ERR_BAD_ARG,
            "fixed-id >= 候选数 被拒（否则 SELECTED_ID 会越界）");
    t.check(ps_ui_exec(&s, "cand 244", out, sizeof(out)) == PS_UI_ERR_BAD_ARG,
            "cand 244 被拒（B 档网格 9x9x3 = 243）");

    // 页面与渲染
    t.check(ps_ui_exec(&s, "page perf", out, sizeof(out)) == PS_UI_OK &&
                s.page == PS_UI_PAGE_PERF,
            "指令 page perf");
    {
        char scr[4096];
        ps_ui_metrics_t mt;
        ps_ui_metrics_init(&mt);
        mt.fps = 30.0;
        mt.exposure_us = 2000;
        mt.capture_latency_us = 1500u;
        mt.pl_latency_us = 100000u;
        mt.rank = s.rank; mt.bits = 12u; mt.iterations = s.iters;
        mt.power_mw = 3120u; mt.selected_id = 42u;
        mt.confidence_q16 = 0x9A00u;
        mt.best_score_q16 = 0x8000u; mt.second_score_q16 = 0x4000u;
        mt.candidate_count = s.candidate_count;
        mt.frames_processed = 128u;
        const int n1 = ps_ui_render(&s, &mt, scr, sizeof(scr));
        t.check(n1 > 0 && std::strstr(scr, "性能页") != nullptr &&
                    std::strstr(scr, "PL 计算延迟") != nullptr &&
                    std::strstr(scr, "fps") != nullptr,
                "性能页渲染（帧率/采集延迟/PL 延迟/秩/位宽/迭代/功耗）");
        s.page = PS_UI_PAGE_MAIN;
        const int n2 = ps_ui_render(&s, &mt, scr, sizeof(scr));
        t.check(n2 > 0 && std::strstr(scr, "[左上]") != nullptr &&
                    std::strstr(scr, "[右上]") != nullptr &&
                    std::strstr(scr, "[左下]") != nullptr &&
                    std::strstr(scr, "[右下]") != nullptr,
                "主画面四区齐全（§7 固定四区）");
        const int n3 = ps_ui_render(&s, nullptr, scr, sizeof(scr));
        t.check(n3 > 0 && std::strstr(scr, "--") != nullptr,
                "无 metric 时显示 -- 而不是 0（0 与「未知」不许混同）");
    }

    t.check(ps_ui_exec(&s, "status", out, sizeof(out)) == PS_UI_NOTE &&
                std::strstr(out, "PenumbraSight UI") != nullptr,
            "指令 status 返回当前画面（NOTE）");
    t.check(ps_ui_exec(&s, "help", out, sizeof(out)) == PS_UI_NOTE &&
                std::strstr(out, "frames") != nullptr,
            "指令 help");
    t.check(ps_ui_exec(&s, "# 注释行", out, sizeof(out)) == PS_UI_OK,
            "注释行与空行被忽略");

    // 状态 <-> 作业配置
    {
        ps_reg_job_config_t cfg;
        ps_ui_to_job_config(&s, &cfg);
        t.check(cfg.frame_count == s.frames && cfg.iter_count == s.iters &&
                    cfg.rank == s.rank && cfg.candidate_count == s.candidate_count &&
                    cfg.selfloc == s.selfloc && cfg.rgb == s.rgb &&
                    cfg.fixed_candidate_id == s.fixed_candidate_id,
                "状态 -> 作业配置：字段一一对应 §8.2 寄存器");
        t.check(ps_reg_frame_count_valid(cfg.frame_count) &&
                    ps_reg_iter_count_valid(cfg.iter_count) &&
                    ps_reg_rank_valid(cfg.rank),
                "导出的配置必然落在允许集合内");
        ps_ui_state_t s2;
        ps_ui_state_init(&s2);
        ps_ui_from_job_config(&s2, &cfg);
        t.check(s2.frames == s.frames && s2.iters == s.iters && s2.rank == s.rank &&
                    s2.selfloc == s.selfloc && s2.rgb == s.rgb &&
                    s2.fixed_candidate_id == s.fixed_candidate_id,
                "作业配置 -> 状态（读回同步）往返一致");
    }

    t.check(ps_ui_exec(&s, "reset", out, sizeof(out)) == PS_UI_OK && s.frames == 4u &&
                s.iters == 16u && s.rank == 64u,
            "指令 reset 回到 §8.2 默认档");
    t.check(ps_ui_exec(&s, "quit", out, sizeof(out)) == PS_UI_QUIT, "指令 quit");
}

}  // namespace

// ============================================================ main

int main(int argc, char** argv) {
    const std::string model_path = (argc > 1) ? argv[1] : "ps_tmp/host_test_model.psm";
    const std::string expected_path =
        (argc > 2) ? argv[2] : "ps_tmp/host_test_model.expected.txt";
    const std::string workdir = (argc > 3) ? argv[3] : "ps_tmp";

    std::printf("=== PenumbraSight PS 侧主机自检（make host-test）===\n");
    std::printf("模型（Python 生成）：%s\n", model_path.c_str());
    std::printf("期望表（Python 生成）：%s\n", expected_path.c_str());
    std::printf("工作目录（纯 ASCII 路径）：%s\n\n", workdir.c_str());
    std::printf("说明：本自检只验寄存器协议/CRC/偏移/量纲/状态机/CSV 列。\n");
    std::printf("      凡需 PL 或实物相机才能得到的性能与精度结论，一律不下结论。\n\n");

    ps::PsTest t_reg("A 寄存器 (ps_regs.h)");
    ps::PsTest t_model("B 模型装载 (model_loader)");
    ps::PsTest t_cam("C 取流 (camera_service)");
    ps::PsTest t_fpga("D 寄存器控制 (fpga_control)");
    ps::PsTest t_telem("E 遥测 (telemetry)");
    ps::PsTest t_ui("F 交互 (ui_server)");

    std::printf("--- A 寄存器位域拼装/解析往返（ps_regs.h）---\n");
    test_regs(t_reg);
    std::printf("\n--- B model_loader：Python 生成的 model.psm 加载 + CRC + 几何哈希 ---\n");
    test_model(t_model, model_path, expected_path, workdir);
    std::printf("\n--- C camera_service：RAW Bayer 约束与两条取流路径 ---\n");
    test_camera(t_cam, workdir);
    std::printf("\n--- D fpga_control：影子寄存器协议与超时复位 ---\n");
    test_fpga(t_fpga);
    std::printf("\n--- E telemetry：CSV 列契约与汇总 ---\n");
    test_telemetry(t_telem, workdir);
    std::printf("\n--- F ui_server：指令解析与状态机 ---\n");
    test_ui(t_ui);

    int fails = 0;
    fails += t_reg.finish();
    fails += t_model.finish();
    fails += t_cam.finish();
    fails += t_fpga.finish();
    fails += t_telem.finish();
    fails += t_ui.finish();

    std::printf("\n%s\n", (fails == 0) ? "全部通过。" : "存在失败项。");
    return (fails == 0) ? 0 : 1;
}
