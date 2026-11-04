// ============================================================================
// PenumbraSight PS 侧平台层（header-only）
//
// 五条 ARM/Linux 用户态进程（camera_service / model_loader / fpga_control /
// telemetry / ui）共用这一层。它存在的唯一理由：**把与操作系统耦合的东西收在
// 一个文件里**，于是各进程的 main 里剩下的全是业务逻辑，且同一份源码既能在
// AXU9EGB 的 Linux ARM64 上跑，也能在开发主机（Windows + MinGW）上编译运行。
//
// 为什么必须强调「ASCII 路径」
// ---------------------------------------------------------------------------
// 板端 Linux 路径没有这个问题；但本仓库的开发主机在 Windows 上，MinGW/MSVCRT
// 的窄字符 fopen 走**当前 ANSI 代码页**（本机为 GBK/936）。把 UTF-8 的源码
// 字面量或含中文的绝对路径直接喂给 fopen，系统会按 GBK 解释这些字节，结果是
// 打开失败——或者更糟，打开到另一个存在的文件。这类错误编译期不报、运行期不崩，
// 只是读到的字节不对，极难回溯。
//
// 所以本文件提供 scratch_dir() / scratch_path()：
//   1. 优先用 $TEMP / $TMP / $TMPDIR（与用户给的测试口径一致；板上就是 /tmp）；
//   2. 若该目录含非 ASCII 字节（本机 $env:TEMP 就含中文用户名），自动退到
//      **相对目录 ./ps_tmp** —— 相对路径里没有非 ASCII 字节，
//      窄字符 fopen 天然安全。
// **所有要 fopen 的代码与测试都必须走这两个函数，不要自己拼路径字符串。**
//
// 时间
// ---------------------------------------------------------------------------
// 统一用 steady_clock 的微秒计数做单调时间（不受墙钟调整影响），
// 用 system_clock 生成 ISO8601 UTC 时间戳（遥测 JSONL 用）。两者语义不同，
// 不能混用：前者算延迟，后者写日志。
// ============================================================================

#ifndef PENUMBRASIGHT_PS_PLATFORM_HPP
#define PENUMBRASIGHT_PS_PLATFORM_HPP

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#  include <direct.h>
#  include <malloc.h>
#  define PS_HOST_WINDOWS 1
#else
#  include <sys/stat.h>
#  include <sys/types.h>
#  include <unistd.h>
#  define PS_HOST_POSIX 1
#endif

namespace ps {

// ============================================================================
// 一、时间与睡眠
// ============================================================================

// 单调微秒。板端对应 clock_gettime(CLOCK_MONOTONIC)，主机对应 steady_clock；
// 两者都保证「只前进不回退」，故可直接做差值算延迟。
inline uint64_t mono_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

inline void sleep_ms(unsigned ms) {
#if defined(PS_HOST_WINDOWS)
    Sleep(static_cast<DWORD>(ms));
#else
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(ms / 1000u);
    ts.tv_nsec = static_cast<long>(ms % 1000u) * 1000000L;
    ::nanosleep(&ts, 0);
#endif
}

// ISO8601 UTC，毫秒精度，形如 2026-09-26T07:31:12.345Z。
// 固定宽度输出是刻意的：遥测 JSONL 要能被 diff 与列对齐，不允许时区/格式漂移。
inline void iso8601_now(char* out, size_t cap) {
    if (!out || cap < 25u) return;
    const std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    const std::chrono::milliseconds total_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    const std::time_t secs = static_cast<std::time_t>(total_ms.count() / 1000);
    const long frac = static_cast<long>(total_ms.count() % 1000);
    std::tm tmv;
    std::memset(&tmv, 0, sizeof(tmv));
#if defined(PS_HOST_WINDOWS)
    gmtime_s(&tmv, &secs);
#else
    gmtime_r(&secs, &tmv);
#endif
    std::snprintf(out, cap, "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
                  tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
                  tmv.tm_hour, tmv.tm_min, tmv.tm_sec, frac);
}

// ============================================================================
// 二、路径与目录（窄字符 fopen 安全）
// ============================================================================

// 只放行可打印 ASCII：这类路径在 GBK / ANSI / UTF-8 三种编码下字节相同，
// 因此窄字符 fopen 的行为在不同代码页上一致。
inline bool is_ascii_path(const char* s) {
    if (!s) return false;
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(s); *p; ++p) {
        if (*p < 0x20u || *p > 0x7Eu) return false;
    }
    return true;
}

// 非 ASCII 路径不返回 false 就完事，而是**明确打印拒绝原因**：
// 这类问题的排查成本极高，必须让它在日志里留下痕迹。
inline bool require_ascii_path(const char* path, const char* tag) {
    if (is_ascii_path(path)) return true;
    std::fprintf(stderr,
                 "[%s] 拒绝非 ASCII 路径（窄字符 fopen 在含中文的路径上会乱码）："
                 "%s\n        请改用 ps::scratch_path() 或 $env:TEMP 下的 ASCII 路径。\n",
                 tag ? tag : "path", path ? path : "(null)");
    return false;
}

inline bool path_exists(const char* p) {
#if defined(PS_HOST_WINDOWS)
    return GetFileAttributesA(p) != INVALID_FILE_ATTRIBUTES;
#else
    struct stat st;
    return ::stat(p, &st) == 0;
#endif
}

inline bool ps_mkdir(const std::string& dir) {
    if (path_exists(dir.c_str())) return true;
#if defined(PS_HOST_WINDOWS)
    return ::_mkdir(dir.c_str()) == 0;
#else
    return ::mkdir(dir.c_str(), 0755) == 0;
#endif
}

// 取 $TEMP / $TMP / $TMPDIR。取不到就返回空串（由 scratch_dir 兜底）。
inline std::string env_temp_dir() {
    static const char* kKeys[3] = {"TEMP", "TMP", "TMPDIR"};
    for (int i = 0; i < 3; ++i) {
        const char* v = std::getenv(kKeys[i]);
        if (v && v[0]) return std::string(v);
    }
    return std::string();
}

// 去掉尾部的 '/' 或 '\'，便于拼接。
inline std::string strip_trailing_sep(std::string s) {
    while (!s.empty() && (s[s.size() - 1] == '/' || s[s.size() - 1] == '\\')) {
        s.erase(s.size() - 1);
    }
    return s;
}

// 一个**保证窄字符 fopen 可用**的暂存目录。结果被缓存（进程内只判定一次）。
//
// 判定顺序：
//   1. $TEMP/$TMP/$TMPDIR 且为纯 ASCII（板端即 /tmp，主机上用户名是英文时也走这里）
//   2. 相对目录 ./ps_tmp —— 相对路径不含非 ASCII 字节，一定安全
// 之所以还留着相对兜底而不是报错退出：本机 $env:TEMP 形如
// C:\Users\<中文用户名>\AppData\Local\Temp，直接拒绝会让所有自检无法运行。
inline const std::string& scratch_dir() {
    static const std::string dir = []() -> std::string {
        const std::string env = strip_trailing_sep(env_temp_dir());
        if (!env.empty() && is_ascii_path(env.c_str())) {
            if (ps_mkdir(env)) return env;
        } else if (!env.empty()) {
            std::fprintf(stderr,
                         "[ps_platform] $TEMP 含非 ASCII 字节，按窄字符 fopen 口径"
                         "改用相对目录 ./ps_tmp（原因见 ps_platform.hpp 头注释）\n");
        }
        const std::string rel = "./ps_tmp";
        ps_mkdir(rel);
        return rel;
    }();
    return dir;
}

// 暂存目录下的一个文件路径。name 必须是 ASCII（中文文件名没有意义，
// 且会把上面那套「编码一致」的论证作废）。
inline std::string scratch_path(const char* name) {
    return scratch_dir() + "/" + (name ? name : "unnamed");
}

// ============================================================================
// 三、对齐内存（板端 DMA 缓冲的替身）
// ============================================================================
//
// 板端真实路径：用保留内存（device tree reserved-memory / udmabuf / CMA）
// 拿到一段物理连续内存，物理地址交给 PL 的 AXI DataMover。
// 主机桩件用 4 KB 对齐的堆内存顶替，**物理地址记为 0**，让「主机没有物理地址」
// 这件事在打印和日志里显式可见，而不是伪装成一个看着像回事的数字。

inline void* aligned_alloc_bytes(size_t bytes, size_t align) {
    const size_t want = (bytes == 0u) ? 1u : bytes;
#if defined(PS_HOST_WINDOWS)
    return _aligned_malloc(want, align);
#else
    void* p = 0;
    if (::posix_memalign(&p, align, want) != 0) return 0;
    return p;
#endif
}

inline void aligned_free_bytes(void* p) {
    if (!p) return;
#if defined(PS_HOST_WINDOWS)
    _aligned_free(p);
#else
    ::free(p);
#endif
}

inline uint64_t ps_align_up(uint64_t n, uint64_t a) {
    return ((n + a - 1u) / a) * a;
}

// ============================================================================
// 三之二、内容指纹（日志与跨进程比对用，不是密码学哈希）
// ============================================================================
//
// 用它做「同一帧走两条路径是否逐字节相同」的粗筛：一旦不等就没必要再逐元素比。
// 真正的模型完整性校验用 CRC32（model_loader/psm_crc32.c），两者不要混用。
inline uint64_t fnv1a64(const void* data, size_t bytes,
                        uint64_t seed = 1469598103934665603ULL) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    uint64_t h = seed;
    for (size_t i = 0; i < bytes; ++i) {
        h ^= static_cast<uint64_t>(p[i]);
        h *= 1099511628211ULL;
    }
    return h;
}

// ============================================================================
// 四、极简测试计数（五条进程的 --selftest 共用）
// ============================================================================
//
// 不引入任何测试框架：这几条进程的判据都是「纯逻辑 + 文件往返」，
// 一个计数器加一堆 [ok]/[FAIL] 已经足够定位问题，且不增加依赖。

struct PsTest {
    const char* name;
    int pass;
    int fail;
    explicit PsTest(const char* n) : name(n), pass(0), fail(0) {}
    void check(bool ok, const char* tag) {
        if (ok) {
            ++pass;
            std::printf("  [ok]   %s\n", tag);
        } else {
            ++fail;
            std::printf("  [FAIL] %s\n", tag);
        }
    }
    int finish() const {
        std::printf("%s: PASS %d  FAIL %d\n", name ? name : "selftest", pass, fail);
        return (fail == 0) ? 0 : 1;
    }
};

}  // namespace ps

#endif  // PENUMBRASIGHT_PS_PLATFORM_HPP
