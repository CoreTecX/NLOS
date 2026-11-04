/* ============================================================================
 * PenumbraSight · SD 卡帧（.psf）板端读取层
 * src/software/sd_frame/sd_frame.h
 *
 * 这一层补的是「不接相机时，从哪里拿 RAW 帧」这个缺口：
 * 相机路径（camera_service 的 spinnaker/gentl 实现）在实物到位前不可用，
 * 论文实测帧经 build/scripts/pack_paper_frame.py 转成 .psf 落在 SD 卡上，
 * 由本层读进 DDR，再交给 raw_preproc 那一级。
 *
 * 格式契约：docs/sd_frame_format.md（**唯一权威**）
 * ---------------------------------------------------------------------------
 *   偏移 0x0000  128 字节定长头（小端，字段表见文档 §2）
 *   偏移 0x0080  裸 payload：width*height 个 16 bit 小端字，光栅序，右对齐
 * payload 就是 raw_preproc.sv 的 s_axis_tdata 流，本层不改变它的排布，
 * 所以「读出来」与「喂进去」之间没有第二次打包。
 *
 * 为什么头部不映射成 C 结构体
 * ---------------------------------------------------------------------------
 * 结构体对齐/填充与主机字节序都会让字段偏移无声漂移；一旦漂移，读出来的
 * width 可能是 height，而程序照样跑。本层按偏移用 rd_u16/rd_u32/rd_u64 从
 * const uint8_t* 逐字节拼小端值，因此：
 *   · 主机（x86，小端，但可能任意对齐）与板端（aarch64）行为一致；
 *   · 同样的输入在两条路径上给出同样的结果，主机自检的结论对板端有效；
 *   · 头部越界的读永远不会发生（先判 len，再取字段）。
 *
 * 依赖纪律（与 meas_norm/ 同风格）
 * ---------------------------------------------------------------------------
 *   · 核心（parse / verify / payload 填充 / 反量化 / 块和）只依赖
 *     <stdint.h>/<stddef.h> + psm_format.h 里的 ps_crc32 声明；
 *     不 malloc、不用 libm、不打印、不碰寄存器；输入输出都是调用方给的缓冲区。
 *   · 只有 **文件入口**（ps_sd_frame_read_file / _verify_file）用 <stdio.h>，
 *     因为它要按路径读文件 —— 这是 PetaLinux 用户态的事；
 *     裸机或自管 DMA 的场景请直接调核心三件套（解析/校验/填充）。
 *   · CRC 复用 model_loader/psm_crc32.c 的 ps_crc32（与 Python zlib.crc32 同参数），
 *     不在本层另立一套校验和。
 *
 * 未验证声明
 * ---------------------------------------------------------------------------
 *   · 主机侧已跑通「转换 -> 读回 -> 逐值一致 -> 块和与期望值比对」
 *     （见 src/software/test_sd_frame.c 与 build/scripts/sd_frame_selftest.py）；
 *   · **未在板上跑过**：SD 卡驱动、DMA 搬运、以及把 payload 交给 DataMover 的
 *     那一段都没有实测；本层只保证「给定文件与缓冲区，读出来的字节是对的」。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_SD_FRAME_H
#define PENUMBRASIGHT_SD_FRAME_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------ 格式常量 */

#define PS_SD_MAGIC_LEN      4u
#define PS_SD_HEADER_BYTES   128u   /* 也是 payload 起始偏移 */
#define PS_SD_VERSION        1u
#define PS_SD_CONTAINER_BITS 16u    /* 只接受 16 */
#define PS_SD_VALID_BITS_MAX 16u

#define PS_SD_BAYER_RGGB 0u
#define PS_SD_BAYER_BGGR 1u
#define PS_SD_BAYER_GRBG 2u
#define PS_SD_BAYER_GBRG 3u

/* 12 bit 在 16 bit 容器里的打包口径（camera_b.yaml 的 sensor.value_packing，
 * 该配置项当前仍为 todo）。本层只如实转述头里的取值，不替它下结论。 */
#define PS_SD_PACKING_PACKED    0u   /* 右对齐 0..4095 */
#define PS_SD_PACKING_FULLSCALE 1u   /* 左移 4 位 0..65520 */

#define PS_SD_FLAG_LITTLE_ENDIAN 0x0001u

/* ------------------------------------------------------------ 字段偏移 */

/* 与 docs/sd_frame_format.md §2 的表逐行对应；改表必须同时改这里与写入端。 */
#define PS_SD_OFF_MAGIC          0x00u
#define PS_SD_OFF_VERSION        0x04u
#define PS_SD_OFF_HEADER_BYTES   0x06u
#define PS_SD_OFF_WIDTH          0x08u
#define PS_SD_OFF_HEIGHT         0x0Cu
#define PS_SD_OFF_VALID_BITS     0x10u
#define PS_SD_OFF_CONTAINER_BITS 0x12u
#define PS_SD_OFF_BAYER_PHASE    0x14u
#define PS_SD_OFF_VALUE_PACKING  0x15u
#define PS_SD_OFF_FLAGS          0x16u
#define PS_SD_OFF_FRAME_INDEX    0x18u
#define PS_SD_OFF_PAYLOAD_BYTES  0x1Cu
#define PS_SD_OFF_SCALE_NUM      0x20u
#define PS_SD_OFF_SCALE_DEN      0x28u
#define PS_SD_OFF_PAYLOAD_CRC32  0x30u
#define PS_SD_OFF_CLIPPED_PIXELS 0x34u
#define PS_SD_OFF_PAYLOAD_SUM    0x38u
#define PS_SD_OFF_PAYLOAD_SUMSQ  0x40u
#define PS_SD_OFF_PAYLOAD_MIN    0x48u
#define PS_SD_OFF_PAYLOAD_MAX    0x4Au
#define PS_SD_OFF_MEDIAN_CODE    0x4Cu
#define PS_SD_OFF_P999_CODE      0x4Eu
#define PS_SD_OFF_SCALE_RULE_OK  0x50u
#define PS_SD_OFF_SOURCE_MEDIAN  0x54u
#define PS_SD_OFF_SOURCE_P999    0x58u

/* ------------------------------------------------------------ 返回码 */

#define PS_SD_OK             0
#define PS_SD_ERR_ARGS      (-1)   /* 空指针 / 尺寸 0 */
#define PS_SD_ERR_TRUNCATED (-2)   /* 文件短于头或短于声明的 payload */
#define PS_SD_ERR_BAD_HEADER (-3)  /* 魔数/版本/字段自洽性不过 */
#define PS_SD_ERR_CHECKSUM  (-4)   /* crc32 / sum / sumsq / min / max 与头不符 */
#define PS_SD_ERR_IO        (-5)   /* 打开或读文件失败 */
#define PS_SD_ERR_BUFFER    (-6)   /* 调用方缓冲区装不下 payload */

/* ------------------------------------------------------------ 解析结果 */

typedef struct {
    uint32_t version;
    uint32_t header_bytes;
    uint32_t width;
    uint32_t height;
    uint32_t valid_bits;
    uint32_t container_bits;
    uint32_t bayer_phase;
    uint32_t value_packing;
    uint32_t flags;
    uint32_t frame_index;
    uint32_t payload_bytes;
    uint64_t scale_num;
    uint64_t scale_den;
    uint32_t payload_crc32;
    uint32_t clipped_pixels;
    uint64_t payload_sum;
    uint64_t payload_sumsq;
    uint32_t payload_min;
    uint32_t payload_max;
    uint32_t median_code;
    uint32_t p999_code;
    uint32_t scale_rule_ok;
    uint32_t source_median;
    uint32_t source_p999;
} ps_sd_frame_header_t;

/* 校验时**实测**的 payload 统计量。与头里的对应字段逐项比对后填回。 */
typedef struct {
    uint32_t crc32;
    uint64_t sum;
    uint64_t sumsq;
    uint32_t min;
    uint32_t max;
    uint32_t words;        /* payload 的 16 bit 字数 */
} ps_sd_frame_check_t;

/* ------------------------------------------------------------ 核心（无 I/O） */

/* 解析并做「格式自洽」检查（不含 payload 校验）：
 *   magic / version / header_bytes / 尺寸非零 / container_bits==16 /
 *   valid_bits<=16 / payload_bytes==width*height*container_bits/8 / packing 取值合法
 * 通过返回 0；失败写 reason（可为 NULL）并返回负码。
 * buf 只需覆盖头部 128 字节；返回值不依赖 buf 之后的字节。 */
int ps_sd_frame_parse_header(const uint8_t *buf, size_t len,
                             ps_sd_frame_header_t *hdr,
                             char *reason, size_t reason_cap);

/* 重算 payload 的 crc32/sum/sumsq/min/max，与头部字段逐项比对。
 * buf 是**整个文件**（含头），len 是文件长度；out 可为 NULL。
 * 返回 PS_SD_OK / PS_SD_ERR_TRUNCATED / PS_SD_ERR_CHECKSUM / PS_SD_ERR_ARGS。 */
int ps_sd_frame_verify(const uint8_t *buf, size_t len,
                       const ps_sd_frame_header_t *hdr,
                       ps_sd_frame_check_t *out);

/* 把 payload 拷成码字缓冲（16 bit 字、按头里的字节序解释）。
 * dst_buf 是调用方给的缓冲区；capacity_words 是它的字数。
 * 需要 capacity_words >= width*height，否则 PS_SD_ERR_BUFFER（不部分拷贝）。
 * 本函数**不做校验**（校验交给 ps_sd_frame_verify），
 * 以便「先校验后搬运」与「边搬边算 CRC」两种板端流程都能用。 */
int ps_sd_frame_payload_codes(const uint8_t *buf, size_t len,
                              const ps_sd_frame_header_t *hdr,
                              uint16_t *dst, size_t capacity_words);

/* ------------------------------------------------------------ 文件入口（stdio） */

/* 按路径读整个文件，解析 + 校验 + 把 payload 填进 dst。
 * dst/dst_capacity_words 允许为 0/NULL（只要头与校验，不要 payload）。
 * hdr/check 允许为 NULL。失败写 reason。 */
int ps_sd_frame_read_file(const char *path,
                          ps_sd_frame_header_t *hdr,
                          uint16_t *dst, size_t dst_capacity_words,
                          ps_sd_frame_check_t *check,
                          char *reason, size_t reason_cap);

/* 只校验不搬 payload（板端上电自检用；实现为 read_file 的 dst=NULL 路径的包装）。 */
int ps_sd_frame_verify_file(const char *path, ps_sd_frame_header_t *hdr,
                            ps_sd_frame_check_t *check,
                            char *reason, size_t reason_cap);

/* ------------------------------------------------------------ 语义辅助 */

/* 反量化：码值 code 对应 v_hat = code * den / num，以有理数 (num_out, den_out)
 * 形式交还，由调用方决定精度（本层不做除法，不用浮点）。
 * 返回 0 成功；hdr 空或 den==0 时返回负码。 */
int ps_sd_frame_dequantize(const ps_sd_frame_header_t *hdr, uint16_t code,
                           uint64_t *num_out, uint64_t *den_out);

/* RGGB 解交织后按 block x block 分块求和，口径与 raw_preproc.sv 一致：
 *   R=raw[2i,2j]、G1=raw[2i,2j+1]、G2=raw[2i+1,2j]、B=raw[2i+1,2j+1]
 *   G = G1 + G2（按**和**进累加，不平均、不右移 —— 仓库原则 P2）
 *   块内只求和、不除法（P1）
 * 输出 ch-major 展平：out[ch*wall_n*wall_n + br*wall_n + bc]，ch 0=R/1=G/2=B。
 * 供板上与 *_expected.json 的 block_sum 段直接比对（见 docs/sd_frame_format.md §8：
 * block 与 wall_n 是**流水线参数**，不在文件头里，由调用方按几何配置传入）。
 * 要求方形帧且 plane_side % block == 0；否则返回负码。 */
int ps_sd_frame_block_sum_rggb(const uint16_t *codes, uint32_t side,
                               uint32_t block, uint32_t *out,
                               size_t out_words);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* PENUMBRASIGHT_SD_FRAME_H */
