// ============================================================================
// model_loader.h —— PS 侧模型装载器（方案 §7 第 2 条）
//
// 职责（只有这三条）：
//   1. 校验 model.psm 的魔数、版本、几何哈希、payload 长度与 CRC32；
//   2. 把 payload 搬进 DDR（板端 DMA / 主机 4 KB 对齐堆内存）；
//   3. **只有全部校验通过才对外给出 DDR 地址**；几何哈希不一致时拒绝出图。
//
// 为什么校验顺序不能改
// ---------------------------------------------------------------------------
//   1) 先读 64 字节头部做 ps_psm_check_header()：不碰大文件，
//      几何哈希不一致时**立刻**拒绝（§6.6：不一致就强制灰屏，不许出图）。
//   2) 再比「文件长度 == 64 + payload_bytes」：长度不对说明文件被截断或版本错；
//      这种文件读下去会读到别的候选的数据，结果「看着像图但在漂」。
//   3) 最后流式读 payload，**边读边算 CRC**，读完即知真伪（单遍 IO）。
//      不先算一遍 CRC 再读一遍搬 DDR —— 286 MB 的文件那样要读两遍。
//
// 与 psm_format.h 的关系
// ---------------------------------------------------------------------------
// psm_format.h 是**冻结**的（偏移与结构体不许改），本文件只 include 它并复用
// ps_psm_check_header()，不重复实现任何偏移推导。
//
// 主机 vs 板端的唯一差异
// ---------------------------------------------------------------------------
// 板端：物理连续内存来自 device tree 的 reserved-memory（或 udmabuf / CMA），
//       物理地址交给 §8.2 的 BUFFER_ADDR 寄存器（见 fpga_control）。
// 主机：4 KB 对齐的堆内存；物理地址**显式记为 0**，不伪装成假地址。
// 两者的差别被压在 ps_model_dma_reserve() / ps_model_dma_release() 两个函数里，
// 上层的 ps_model_load() 一行都不改。板端要换的就是这两个函数（见 model_loader.c）。
// ============================================================================

#ifndef PENUMBRASIGHT_MODEL_LOADER_H
#define PENUMBRASIGHT_MODEL_LOADER_H

#include <stdint.h>

#include "psm_format.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 返回码

#define PS_MODEL_OK               0
#define PS_MODEL_ERR_ARG          19   // 入参为空等调用错误
#define PS_MODEL_ERR_OPEN         20   // 打开失败（含非 ASCII 路径被拒）
#define PS_MODEL_ERR_TRUNCATED    21   // 连 64 字节头部都读不满
#define PS_MODEL_ERR_HEADER_BASE  100  // 100 + PS_PSM_ERR_*（1..10），保持可区分
#define PS_MODEL_ERR_CRC          120  // payload CRC32 不匹配
#define PS_MODEL_ERR_SIZE         121  // 文件长度与 payload_bytes 不符
#define PS_MODEL_ERR_ALLOC        122  // DDR 区申请失败
#define PS_MODEL_ERR_READ         123  // payload 读取失败

const char* ps_model_status_name(int rc);

// ---------------------------------------------------------------- 期望形状

// §8.1 的尺寸组合一旦与当前配置不符，PL 侧的 K/R/M/N 全都要重算，而读进来的
// U/S/V 大小是对的、数字也「看着合理」，只是语义整体错位。故形状是装载的
// **必要条件**，不是可选优化。某一维填 0 表示该维不检查（仅调试用）。
typedef struct {
    uint32_t meas_h;
    uint32_t meas_w;
    uint32_t hidden_h;
    uint32_t hidden_w;
} ps_model_shape_t;

static inline ps_model_shape_t ps_model_shape_make(uint32_t mh, uint32_t mw,
                                                   uint32_t hh, uint32_t hw) {
    ps_model_shape_t s;
    s.meas_h = mh; s.meas_w = mw; s.hidden_h = hh; s.hidden_w = hw;
    return s;
}

// B 档冻结形状（configs/geometry_b.yaml）：95x96 -> 18x24
static inline ps_model_shape_t ps_model_shape_b(void) {
    return ps_model_shape_make(PS_PSM_MEAS_H, PS_PSM_MEAS_W,
                               PS_PSM_HIDDEN_H, PS_PSM_HIDDEN_W);
}

// ---------------------------------------------------------------- 装载句柄

typedef struct {
    ps_psm_header_t header;
    uint64_t file_bytes;
    uint64_t payload_bytes;
    uint32_t crc_computed;         // 实算值
    uint32_t crc_expected;         // 头部声明值
    uint32_t expect_geometry_hash; // 调用方给的期望值（0 = 未检查）

    void*    payload;              // 装载区可写指针
    uint64_t payload_capacity;     // 装载区字节数（≥ payload_bytes，4 KB 对齐）
    uint64_t payload_phys;         // 板端 = 真实物理地址；主机桩件 = 0
    int      is_stub;              // 1 = 主机桩件

    int      loaded_payload;       // 1 = payload 已在装载区
    int      geometry_verified;    // 1 = 几何哈希**真的比对过**且一致
    int      status;               // PS_MODEL_*
} ps_model_t;

// ---------------------------------------------------------------- 候选头取值

// 取第 index 个候选的 s_max（候选头 +20，2026-09-29 起入 psm）。
//
// psm 里的 S 是归一化奇异值（最大 1），绝对幅度为
//     S_abs[k] = S_code[k] * s_scale * s_max
// 此前 s_max 只在 manifest.json，只读 psm 的板端拿到的是 B/s_max；现在每个候选
// 记录自带该值，重建不必再向调用方要系数（见 psm_format.h 候选头注释与
// sim/results/x28b_psm_u_transpose_fix.md §7）。未装载 / 下标越界时返回 0。
static inline float ps_model_candidate_s_max(const ps_model_t* m, uint32_t index) {
    if (!m || !m->payload || index >= (uint32_t)m->header.candidates) return 0.0f;
    const uint8_t* cand = (const uint8_t*)m->payload
                        + (uint64_t)index * ps_psm_record_bytes_hdr(&m->header);
    return ps_psm_read_s_max(cand);
}

// 只读 64 字节头部并做形状/哈希/长度自检（板端启动路径，只碰一次 64 字节）。
// expect_geometry_hash 传 0 表示跳过哈希检查（**仅调试**）。
int ps_model_read_header(const char* path, uint32_t expect_geometry_hash,
                         const ps_model_shape_t* shape, ps_psm_header_t* out,
                         uint64_t* file_bytes);

// 完整装载：头部自检 -> 长度比对 -> 单遍读 payload（同时算 CRC）-> 放进装载区。
// 任何一步失败都不留下半装载状态（失败时自动释放装载区，见 .c 的说明）。
int ps_model_load(const char* path, uint32_t expect_geometry_hash,
                  const ps_model_shape_t* shape, ps_model_t* out);

void ps_model_unload(ps_model_t* m);

// §6.6 出图门：只有「装载成功 + payload 已在 + 几何哈希比对过且一致」才返回 1。
// 三个条件缺一不可 —— 传 expect_geometry_hash = 0 跳过检查时，本函数返回 0，
// 因为「没验过」和「验过且对」不是一回事，而 §6.6 要求不一致时必须灰屏。
int ps_model_render_allowed(const ps_model_t* m);

// 人工对照用打印（与 psm.py 的 summarize 口径一致）。
void ps_model_print(const ps_model_t* m);

// ---------------------------------------------------------------- 装载区

// 申请/释放装载区。**板端要替换的就是这两个函数**（见 model_loader.c 的 TODO）。
// 返回 PS_MODEL_OK / PS_MODEL_ERR_ALLOC。
int ps_model_dma_reserve(ps_model_t* m, uint64_t bytes);
void ps_model_dma_release(ps_model_t* m);

// ---------------------------------------------------------------- CRC 自检

// 实现在 psm_crc32.c：返回 0 = 全部通过，非 0 = 失败的用例号。
// 覆盖空输入、标准向量 "123456789"、以及**分块累加**（psm.py 真正用到的语义）。
int ps_crc32_selftest(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PENUMBRASIGHT_MODEL_LOADER_H
