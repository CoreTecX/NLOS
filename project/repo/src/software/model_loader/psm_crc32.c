// ============================================================================
// psm_crc32.c —— model.psm 的 CRC32（与 Python zlib.crc32 **同参数**）
//
// 为什么不能在 C 端随手换一个「差不多」的 CRC
// ---------------------------------------------------------------------------
// model.psm 是 PC 侧用 numpy 生成、板端用 C 读取的二进制接口
// （见 src/model/penumbrasight_model/psm.py）。写文件时
//     crc = zlib.crc32(blob, crc)      # 逐候选累加
// 读文件时必须给出**同一个数**，否则要么把好模型判成损坏（拒载，无法演示），
// 要么把损坏模型判成完好（板上出图但结果错，且没有任何提示）。两者都是灾难。
//
// 参数（与 zlib.crc32 完全一致）
// ---------------------------------------------------------------------------
//   * 多项式 0x04C11DB7 的反射形式 0xEDB88320
//   * 初值：内部先取反（本函数入口把入参 ^0xFFFFFFFF）
//   * 反射输入 / 反射输出
//   * 末取反
// 于是：
//     zlib.crc32(b"123456789")            == 0xCBF43926
//     zlib.crc32(chunk2, zlib.crc32(chunk1)) == ps_crc32(ps_crc32(0, chunk1), chunk2)
// 第二条（可链式累加）才是真正被 psm.py 用到的语义：它按候选逐块累加 CRC，
// 而不是一次性喂全 300 MB。因此本实现的**入参 crc 就是「上一块的返回值」**，
// 这一点如果理解错（例如把入参当成「已经取反过」的值），
// 单块测试会通过、多块累加会全错。
//
// 与其它 CRC 的区别（别搞混）
// ---------------------------------------------------------------------------
//   * geometry_hash（§8.1）也是 zlib.crc32，但它算的是**规范字符串**，不是 payload；
//   * FNV-1a（ps_platform.hpp 的 fnv1a64）只用于日志指纹，不是校验和。
//
// 线程安全
// ---------------------------------------------------------------------------
// 查表是惰性构建、无锁。板端唯一调用点是 model_loader 的启动路径（单线程），
// 主机侧自检也是单线程。若将来要在多线程里用，先把表改成编译期常量表。
// ============================================================================

#include "psm_format.h"

#include <stddef.h>
#include <stdint.h>

// ---------------------------------------------------------------- 查表

static uint32_t g_crc_table[256];
static int g_crc_table_ready = 0;

static void ps_crc32_build_table(void) {
    for (uint32_t i = 0u; i < 256u; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1u)) : (c >> 1u);
        }
        g_crc_table[i] = c;
    }
    g_crc_table_ready = 1;
}

// ---------------------------------------------------------------- 主函数

// 与 zlib.crc32(crc, data, len) 逐位相同。crc 传 0 表示「从头开始」。
// data 为 NULL 或 len 为 0 时返回 crc 的规范化结果（对 0 返回 0）。
uint32_t ps_crc32(uint32_t crc, const uint8_t* data, size_t len) {
    if (!g_crc_table_ready) {
        ps_crc32_build_table();
    }
    uint32_t c = crc ^ 0xFFFFFFFFu;
    if (data != NULL) {
        for (size_t i = 0u; i < len; ++i) {
            c = g_crc_table[(c ^ (uint32_t)data[i]) & 0xFFu] ^ (c >> 8u);
        }
    }
    return c ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------- 自检

// 已知向量自检，返回 0 = 通过，非 0 = 失败的用例号。
// 三条用例分别覆盖：空输入、标准向量、**分块累加**（psm.py 真正用到的语义）。
int ps_crc32_selftest(void) {
    static const uint8_t kStd[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    static const uint8_t kA[1] = {'a'};

    // 用例 1：空输入。zlib.crc32(b"") == 0
    if (ps_crc32(0u, NULL, 0u) != 0x00000000u) return 1;

    // 用例 2：标准向量 "123456789" -> 0xCBF43926
    if (ps_crc32(0u, kStd, 9u) != 0xCBF43926u) return 2;

    // 用例 3："a" -> 0xE8B7BE43
    if (ps_crc32(0u, kA, 1u) != 0xE8B7BE43u) return 3;

    // 用例 4：4+5 分块累加 == 一次性 9 字节。这条不过，说明链式语义错了，
    // 而链式语义正是 psm.py 写文件与 verify_model 的方式。
    {
        const uint32_t part1 = ps_crc32(0u, kStd, 4u);
        const uint32_t part2 = ps_crc32(part1, kStd + 4, 5u);
        if (part2 != 0xCBF43926u) return 4;
    }

    // 用例 5：单字节分块累加（最坏分块粒度）也必须一致。
    {
        uint32_t c = 0u;
        for (size_t i = 0u; i < 9u; ++i) {
            c = ps_crc32(c, kStd + i, 1u);
        }
        if (c != 0xCBF43926u) return 5;
    }

    return 0;
}
