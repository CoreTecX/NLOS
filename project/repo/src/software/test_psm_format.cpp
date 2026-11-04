// ============================================================================
// psm_format.h <-> psm.py 逐字节一致性测试
//
// 为什么必须有这个测试
// ---------------------------------------------------------------------------
// model.psm 是 PC 侧生成、板端读取的二进制接口。两侧的偏移算错**不会崩**，
// 只会让板上读到「看着都合理但整体错位」的数据，而这种错误在图像上表现为
// 「重建结果糊」，几乎不可能靠看图像回溯到是 4096 对齐少算了一次。
// 所以把两侧的偏移/长度都打印出来做**逐值比对**，比任何注释都可靠。
//
// 用法：
//     g++ -std=c++17 -I src/software/include test_psm_format.cpp -o test_psm_format
//     ./test_psm_format            # 打印规范行
//     python -m penumbrasight_model.psm --parity   # 打印同一批数
//     tests/run_format_parity.ps1  # 两者比对
// ============================================================================

#include <cstdio>
#include <cstdint>

#include "psm_format.h"

int main() {
    // B 档冻结尺寸 + 一个刻意取非整块的对照尺寸（rank_max=96 那档）
    struct Case {
        const char* name;
        uint16_t rank_max;
        uint16_t meas_h, meas_w;
        uint16_t hidden_h, hidden_w;
    } cases[] = {
        {"b_r64",   64, 95, 96, 18, 24},
        {"r96",     96, 95, 96, 18, 24},
        {"paper",   64, 62, 63, 29, 36},   // 论文档 ds=4 的比例（63 墙 -> 62 差分）
        {"tiny",     4,  4,  3,  2,  2},
    };

    for (const Case& c : cases) {
        const uint64_t m = (uint64_t)c.meas_h * c.meas_w;
        const uint64_t n = (uint64_t)c.hidden_h * c.hidden_w;
        const uint64_t off_u = ps_psm_u_offset(c.rank_max, m);
        const uint64_t off_s = ps_psm_s_offset(c.rank_max, m);
        const uint64_t off_v = ps_psm_v_offset(c.rank_max, m);
        const uint64_t off_k = ps_psm_mask_offset(c.rank_max, m, n);
        const uint64_t rec = ps_psm_record_bytes(c.rank_max, m, n);
        printf("%s rank=%u M=%llu N=%llu U=%llu S=%llu V=%llu MASK=%llu REC=%llu\n",
               c.name, (unsigned)c.rank_max,
               (unsigned long long)m, (unsigned long long)n,
               (unsigned long long)off_u, (unsigned long long)off_s,
               (unsigned long long)off_v, (unsigned long long)off_k,
               (unsigned long long)rec);
    }

    // 结构体大小与字段偏移（这两条是 #pragma pack(1) 的验尸报告）
    printf("sizeof_header=%zu sizeof_cand_hdr=%zu\n",
           sizeof(ps_psm_header_t), sizeof(ps_psm_candidate_hdr_t));
    printf("off_magic=%zu off_geom=%zu off_crc=%zu off_payload=%zu\n",
           offsetof(ps_psm_header_t, magic),
           offsetof(ps_psm_header_t, geometry_hash),
           offsetof(ps_psm_header_t, payload_crc32),
           offsetof(ps_psm_header_t, payload_bytes));

    // 头部自检函数必须能识别出被篡改的字段
    ps_psm_header_t h;
    for (unsigned i = 0; i < sizeof(h); ++i) ((uint8_t*)&h)[i] = 0;
    h.magic[0] = 'P'; h.magic[1] = 'S'; h.magic[2] = 'M'; h.magic[3] = '1';
    h.version = PS_PSM_VERSION;
    h.header_bytes = PS_PSM_HEADER_BYTES;
    h.meas_h = PS_PSM_MEAS_H; h.meas_w = PS_PSM_MEAS_W;
    h.hidden_h = PS_PSM_HIDDEN_H; h.hidden_w = PS_PSM_HIDDEN_W;
    h.candidates = PS_PSM_CANDIDATES;
    h.rank_max = PS_PSM_RANK_MAX; h.rank_default = PS_PSM_RANK_DEFAULT;
    h.loc_rank = PS_PSM_LOC_RANK;
    h.geometry_hash = 0xDEADBEEFu;
    h.payload_bytes = (uint64_t)h.candidates * ps_psm_record_bytes_hdr(&h);
    int rc = ps_psm_check_header(&h, 0xDEADBEEFu, PS_PSM_MEAS_H, PS_PSM_MEAS_W,
                                 PS_PSM_HIDDEN_H, PS_PSM_HIDDEN_W);
    printf("check_header_ok=%d\n", rc);
    int rc_bad_hash = ps_psm_check_header(&h, 0x12345678u, PS_PSM_MEAS_H,
                                          PS_PSM_MEAS_W, PS_PSM_HIDDEN_H,
                                          PS_PSM_HIDDEN_W);
    printf("check_header_badhash=%d (expect %d)\n", rc_bad_hash, PS_PSM_ERR_GEOM_HASH);
    // 秩关系被破坏时必须报错，而不是继续算下去
    h.loc_rank = 200;
    int rc_bad_rank = ps_psm_check_header(&h, 0, 0, 0, 0, 0);
    printf("check_header_badrank=%d (expect %d)\n", rc_bad_rank, PS_PSM_ERR_RANK);
    return 0;
}
