// ============================================================================
// model_loader.c —— model.psm 的校验与装载（纯 C，板端复用同一份）
//
// 本文件只有两块内容：**校验**与**搬运**。不解释 U/S/V 的含义，不做任何矩阵
// 运算 —— 模型怎么用是 PL 的事（方案 §7 第 2 条）。
//
// 需要 S 的绝对幅度时用 model_loader.h 的 ps_model_candidate_s_max()：psm 的 S 是
// 归一化值，绝对幅度 σ = S_code · s_scale · s_max，而 s_max 自 2026-09-29 起随
// 候选头（+20）入 psm，板端不必再依赖 manifest.json。
//
// CRC32 为什么必须与 Python 同参数
// ---------------------------------------------------------------------------
// model.psm 是 PC 侧 numpy 生成、板端 C 读取的二进制接口。写侧是
//     crc = zlib.crc32(blob, crc)      # 按候选逐块累加，见 psm.py write_model
// 读侧必须给出**同一个数**：不一致时要么把好模型判成损坏（拒载，无法演示），
// 要么把损坏模型判成完好（板上出图但结果错且无提示）。两者都是灾难。
// 实现在同目录的 psm_crc32.c（与 psm_format.h 的声明一一对应）。
//
// 失败时不许留下半装载状态
// ---------------------------------------------------------------------------
// 校验失败必须把已申请的装载区还回去：板上那是 286 MB 的 DDR 保留区，
// 一次坏模型就泄漏掉，后续作业会拿不到缓冲。
//
// 板端替换点（只有这两个函数）
// ---------------------------------------------------------------------------
//   ps_model_dma_reserve() / ps_model_dma_release()
// 真实路径见这两个函数体内的 TODO；其余代码（校验、单遍 CRC、长度比对）
// 板上原样复用。
//
// 主机上还有一个「窄字符 fopen」的现实约束
// ---------------------------------------------------------------------------
// 板端 Linux 路径没有这个问题；但开发主机是 Windows + MinGW，窄字符 fopen 走
// 当前 ANSI 码页（本机为 GBK/936）。把 UTF-8 的路径喂进去会被按 GBK 解释，
// 结果可能是「打开失败」，也可能是**打开到另一个存在的文件**。故本文件对非
// ASCII 路径显式拒绝并打印原因（本仓库路径含中文，这不是假想问题）。
// 自检与 demo 一律用相对路径（./ps_tmp/...），天然 ASCII。
// ============================================================================

#include "model_loader.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#  include <malloc.h>
#endif

// ---------------------------------------------------------------- 平台小件

// 只放行可打印 ASCII：这类路径在 GBK / ANSI / UTF-8 三种编码下字节相同，
// 窄字符 fopen 的行为因此与代码页无关。
static int ps_model_path_is_ascii(const char* s) {
    if (!s) return 0;
    for (; *s != '\0'; ++s) {
        const unsigned char c = (unsigned char)*s;
        if (c < 0x20u || c > 0x7Eu) return 0;
    }
    return 1;
}

// 非 ASCII 路径不返回失败就完事，而是**明确打印拒绝原因**：
// 这类问题的排查成本极高（错误信息是「打开失败」，而文件其实存在）。
static int ps_model_require_ascii_path(const char* path) {
    if (ps_model_path_is_ascii(path)) return 1;
    fprintf(stderr,
            "[model_loader] 拒绝非 ASCII 路径（窄字符 fopen 在含中文的路径上会"
            "乱码）：%s\n"
            "        请用纯 ASCII 路径（本仓库自检一律用 ./ps_tmp/...）。\n",
            path ? path : "(null)");
    return 0;
}

// 4 KB 对齐的内存。板端的等价物是保留内存，见 ps_model_dma_reserve()。
static void* ps_model_aligned_alloc(size_t bytes) {
    const size_t want = (bytes == 0u) ? 1u : bytes;
#if defined(_WIN32)
    return _aligned_malloc(want, (size_t)PS_PSM_ARRAY_ALIGN);
#else
    {
        void* p = NULL;
        if (posix_memalign(&p, (size_t)PS_PSM_ARRAY_ALIGN, want) != 0) return NULL;
        return p;
    }
#endif
}

static void ps_model_aligned_free(void* p) {
    if (!p) return;
#if defined(_WIN32)
    _aligned_free(p);
#else
    free(p);
#endif
}

static uint64_t ps_model_align_up(uint64_t n, uint64_t a) {
    return ((n + a - 1u) / a) * a;
}

// ---------------------------------------------------------------- 返回码名字

const char* ps_model_status_name(int rc) {
    switch (rc) {
        case PS_MODEL_OK:           return "OK";
        case PS_MODEL_ERR_ARG:      return "入参非法";
        case PS_MODEL_ERR_OPEN:     return "打开失败";
        case PS_MODEL_ERR_TRUNCATED:return "头部不完整";
        case PS_MODEL_ERR_CRC:      return "CRC32 不匹配";
        case PS_MODEL_ERR_SIZE:     return "文件/payload 长度不符";
        case PS_MODEL_ERR_ALLOC:    return "装载区申请失败";
        case PS_MODEL_ERR_READ:     return "payload 读取失败";
        default: break;
    }
    // 头部自检的细分错误码：把 ps_psm_check_header 的结论原样翻译出来，
    // 而不是笼统报「模型坏了」—— 不同原因对应完全不同的排查动作。
    switch (rc - PS_MODEL_ERR_HEADER_BASE) {
        case PS_PSM_ERR_MAGIC:       return "头部魔数错";
        case PS_PSM_ERR_VERSION:     return "版本不支持";
        case PS_PSM_ERR_HEADER_SIZE: return "header_bytes 不是 64";
        case PS_PSM_ERR_SHAPE:       return "尺寸组合不合法/与期望不符";
        case PS_PSM_ERR_RANK:        return "秩关系不满足";
        case PS_PSM_ERR_GEOM_HASH:   return "几何哈希与当前配置不一致";
        case PS_PSM_ERR_CRC32:       return "头部 CRC 字段错";
        case PS_PSM_ERR_SIZE:        return "payload 长度自检失败";
        case PS_PSM_ERR_CANDIDATES:  return "候选数不符";
        case PS_PSM_ERR_ALIGN:       return "数组偏移未按 4096 对齐";
        default: break;
    }
    return "未知错误";
}

// ---------------------------------------------------------------- 装载区

int ps_model_dma_reserve(ps_model_t* m, uint64_t bytes) {
    if (!m) return PS_MODEL_ERR_ARG;
    const uint64_t want = ps_model_align_up(bytes, (uint64_t)PS_PSM_ARRAY_ALIGN);
    m->payload = ps_model_aligned_alloc((size_t)want);
    if (!m->payload) {
        fprintf(stderr, "[model_loader] 申请 %llu 字节装载区（4 KB 对齐）失败\n",
                (unsigned long long)want);
        m->payload_capacity = 0;
        return PS_MODEL_ERR_ALLOC;
    }
    m->payload_capacity = want;
    // TODO(板端)：把这两行换成保留内存取块 ——
    //   1) 从 device tree 的 reserved-memory / udmabuf / CMA 池取一段物理连续内存：
    //        /sys/class/u-dma-buf/udmabuf0/phys_addr  （或 /proc/iomem 里的保留段）
    //      mmap 到用户态拿 virt，phys 就是驱动给的真实物理地址；
    //   2) 首地址必须 4096 对齐（跨 4 KB 边界的 AXI 突发会被拆包，§5.3 的带宽
    //      假设随之作废）；不对齐时必须往前挪，而不是照用；
    //   3) 把 phys 填进 m->payload_phys（fpga_control 会写进 §8.2 的
    //      MODEL_ADDR_LO/HI），并置 m->is_stub = 0；
    //   4) 释放走 munmap + 归还池子，而不是 free。
    m->payload_phys = 0;   // 主机没有物理地址。**显式留 0**，不许填个假值。
    m->is_stub = 1;
    return PS_MODEL_OK;
}

void ps_model_dma_release(ps_model_t* m) {
    if (!m) return;
    ps_model_aligned_free(m->payload);
    m->payload = NULL;
    m->payload_capacity = 0;
    m->payload_phys = 0;
}

// ---------------------------------------------------------------- 头部自检

int ps_model_read_header(const char* path, uint32_t expect_geometry_hash,
                         const ps_model_shape_t* shape, ps_psm_header_t* out,
                         uint64_t* file_bytes) {
    if (!path) return PS_MODEL_ERR_ARG;
    if (!ps_model_require_ascii_path(path)) return PS_MODEL_ERR_OPEN;

    FILE* f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[model_loader] 打不开 %s\n", path);
        return PS_MODEL_ERR_OPEN;
    }

    if (out) {
        const size_t got = fread(out, 1, (size_t)PS_PSM_HEADER_BYTES, f);
        if (got != (size_t)PS_PSM_HEADER_BYTES) {
            fclose(f);
            return PS_MODEL_ERR_TRUNCATED;
        }
    }
    if (file_bytes) {
        // 注意：主机桩件用 long 长度，够覆盖 B 档的 286 MB；板端若要用
        // >2 GB 的模型需换 _fseeki64 / fstat64。
        if (fseek(f, 0, SEEK_END) != 0) {
            fclose(f);
            return PS_MODEL_ERR_READ;
        }
        *file_bytes = (uint64_t)ftell(f);
    }
    fclose(f);

    if (!out) return PS_MODEL_OK;

    const uint32_t mh = shape ? shape->meas_h : 0u;
    const uint32_t mw = shape ? shape->meas_w : 0u;
    const uint32_t hh = shape ? shape->hidden_h : 0u;
    const uint32_t hw = shape ? shape->hidden_w : 0u;

    // ps_psm_check_header 里包含：魔数、版本、header_bytes、形状自洽与期望比对、
    // 秩关系、几何哈希、四个数组的 4096 对齐、payload 长度自洽。
    const int hrc = ps_psm_check_header(out, expect_geometry_hash, mh, mw, hh, hw);
    if (hrc != PS_PSM_OK) return PS_MODEL_ERR_HEADER_BASE + hrc;

    // 文件长度必须正好是「64 + payload_bytes」。少了是截断，多了是追加了垃圾。
    if (file_bytes &&
        *file_bytes != (uint64_t)PS_PSM_HEADER_BYTES + out->payload_bytes) {
        return PS_MODEL_ERR_SIZE;
    }
    return PS_MODEL_OK;
}

// ---------------------------------------------------------------- 完整装载

int ps_model_load(const char* path, uint32_t expect_geometry_hash,
                  const ps_model_shape_t* shape, ps_model_t* out) {
    if (!path || !out) return PS_MODEL_ERR_ARG;

    memset(out, 0, sizeof(*out));
    out->status = PS_MODEL_ERR_OPEN;

    int rc = ps_model_read_header(path, expect_geometry_hash, shape, &out->header,
                                  &out->file_bytes);
    out->expect_geometry_hash = expect_geometry_hash;
    if (rc != PS_MODEL_OK) {
        out->status = rc;
        return rc;
    }

    out->payload_bytes = out->header.payload_bytes;
    out->crc_expected = out->header.payload_crc32;
    out->geometry_verified =
        (expect_geometry_hash != 0u &&
         out->header.geometry_hash == expect_geometry_hash) ? 1 : 0;

    rc = ps_model_dma_reserve(out, out->payload_bytes);
    if (rc != PS_MODEL_OK) {
        out->status = rc;   // payload == NULL，无需释放
        return rc;
    }

    FILE* f = fopen(path, "rb");
    if (!f) {
        ps_model_dma_release(out);
        out->status = PS_MODEL_ERR_OPEN;
        return PS_MODEL_ERR_OPEN;
    }
    if (fseek(f, (long)PS_PSM_HEADER_BYTES, SEEK_SET) != 0) {
        fclose(f);
        ps_model_dma_release(out);
        out->status = PS_MODEL_ERR_READ;
        return PS_MODEL_ERR_READ;
    }

    // 单遍 IO：边读边累加 CRC。1 MiB 分块与 psm.py 的 verify_model 同粒度，
    // 于是两侧的中间 CRC 值（调试时打印）也可以直接对照。
    uint8_t* dst = (uint8_t*)out->payload;
    uint64_t done = 0;
    uint32_t crc = 0u;
    rc = PS_MODEL_OK;
    while (done < out->payload_bytes) {
        const uint64_t remain = out->payload_bytes - done;
        const size_t want = (size_t)((remain < (1u << 20)) ? remain : (1u << 20));
        const size_t got = fread(dst + (size_t)done, 1, want, f);
        if (got != want) {
            rc = PS_MODEL_ERR_READ;
            break;
        }
        crc = ps_crc32(crc, dst + (size_t)done, got);
        done += (uint64_t)got;
    }
    fclose(f);
    out->crc_computed = crc;

    if (rc != PS_MODEL_OK) {
        ps_model_dma_release(out);
        out->status = rc;
        return rc;
    }
    if (crc != out->crc_expected) {
        // 校验失败必须把装载区还回去（板上 286 MB 的 DDR 不能因一次坏模型泄漏）。
        ps_model_dma_release(out);
        out->status = PS_MODEL_ERR_CRC;
        return PS_MODEL_ERR_CRC;
    }

    out->loaded_payload = 1;
    out->status = PS_MODEL_OK;
    return PS_MODEL_OK;
}

void ps_model_unload(ps_model_t* m) {
    if (!m) return;
    ps_model_dma_release(m);
    m->loaded_payload = 0;
    m->geometry_verified = 0;
}

// ---------------------------------------------------------------- 出图门

int ps_model_render_allowed(const ps_model_t* m) {
    if (!m) return 0;
    if (m->status != PS_MODEL_OK) return 0;
    if (!m->loaded_payload) return 0;
    // 「没验过」与「验过且一致」不是一回事：expect_geometry_hash = 0 跳过检查时
    // geometry_verified 为 0，本函数返回 0（§6.6：不一致必须灰屏，宁可不显示）。
    if (!m->geometry_verified) return 0;
    return 1;
}

// ---------------------------------------------------------------- 打印

void ps_model_print(const ps_model_t* m) {
    if (!m) return;
    const ps_psm_header_t* hd = &m->header;
    const uint64_t mlen = ps_psm_meas_len(hd);
    const uint64_t nlen = ps_psm_hidden_len(hd);

    printf("model.psm: %ux%u -> %ux%u，%u 候选，rank max/default/loc = %u/%u/%u\n",
           (unsigned)hd->meas_h, (unsigned)hd->meas_w,
           (unsigned)hd->hidden_h, (unsigned)hd->hidden_w,
           (unsigned)hd->candidates,
           (unsigned)hd->rank_max, (unsigned)hd->rank_default,
           (unsigned)hd->loc_rank);
    printf("  记录：单候选 %llu B，payload %llu B，文件 %llu B\n",
           (unsigned long long)ps_psm_record_bytes_hdr(hd),
           (unsigned long long)m->payload_bytes,
           (unsigned long long)m->file_bytes);
    printf("  CRC32：期望 %08X，实算 %08X（%s）\n",
           (unsigned)m->crc_expected, (unsigned)m->crc_computed,
           (m->crc_computed == m->crc_expected) ? "一致" : "不一致");
    printf("  geometry_hash：%08X（期望 %08X，%s）\n",
           (unsigned)hd->geometry_hash, (unsigned)m->expect_geometry_hash,
           (m->expect_geometry_hash == 0u)
               ? "未检查（调试）"
               : (m->geometry_verified ? "一致" : "不一致"));
    printf("  装载区：%llu B（4 KB 对齐），物理地址 = 0x%llX（%s）\n",
           (unsigned long long)m->payload_capacity,
           (unsigned long long)m->payload_phys,
           m->is_stub ? "主机桩件，占位 0" : "板端真实地址");
    printf("  §6.6 出图门：%s\n",
           ps_model_render_allowed(m) ? "允许出图" : "拒绝出图（灰屏）");
    printf("  数组相对偏移：U=%llu S=%llu V=%llu MASK=%llu（M=%llu N=%llu）\n",
           (unsigned long long)ps_psm_u_offset(hd->rank_max, mlen),
           (unsigned long long)ps_psm_s_offset(hd->rank_max, mlen),
           (unsigned long long)ps_psm_v_offset(hd->rank_max, mlen),
           (unsigned long long)ps_psm_mask_offset(hd->rank_max, mlen, nlen),
           (unsigned long long)mlen, (unsigned long long)nlen);
}
