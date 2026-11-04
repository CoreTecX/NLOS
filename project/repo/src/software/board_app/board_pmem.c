/* ============================================================================
 * PenumbraSight · 板端连续物理内存分配（可移植逻辑）
 * src/software/board_app/board_pmem.c
 *
 * 见 board_pmem.h 的三条路说明。本文件不含平台头文件，只调 ps_pmem_env_t。
 * ==========================================================================*/

#include "board_pmem.h"

#include <stdio.h>
#include <string.h>

/* 与 Linux 取值一致的 prot/flags（本文件只把这些数透传给 env->map）。 */
#define PMEM_PROT_RW    3
#define PMEM_MAP_SHARED 1
#define PMEM_O_RDWR     2

const ps_pmem_env_t *ps_pmem_env_default(void)
{
#if defined(__linux__)
    return ps_pmem_env_posix();
#else
    return 0;
#endif
}

/* ---------------------------------------------------------------- 小工具 */

int ps_pmem_parse_u64(const char *s, uint64_t *out)
{
    uint64_t v = 0ull;
    uint64_t base = 10ull;
    int any = 0;

    if (out) { *out = 0ull; }
    if (!s) { return 0; }
    while (*s == ' ' || *s == '\t') { ++s; }
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) { base = 16ull; s += 2; }
    for (; *s != '\0'; ++s) {
        unsigned d;
        char c = *s;
        if (c == '\n' || c == '\r' || c == ' ' || c == '\t') { break; }
        if (c >= '0' && c <= '9') { d = (unsigned)(c - '0'); }
        else if (base == 16ull && c >= 'a' && c <= 'f') { d = (unsigned)(c - 'a') + 10u; }
        else if (base == 16ull && c >= 'A' && c <= 'F') { d = (unsigned)(c - 'A') + 10u; }
        else { return 0; }
        if ((uint64_t)d >= base) { return 0; }
        if (v > (UINT64_MAX - (uint64_t)d) / base) { return 0; }
        if (v > (UINT64_MAX - (uint64_t)d) / base) return 0;
        v = v * base + (uint64_t)d;
        any = 1;
    }
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') ++s;
    if (!any || *s) { return 0; }
    if (out) { *out = v; }
    return 1;
}

static uint64_t pmem_round_up(uint64_t v, uint64_t a)
{
    if (a == 0ull) { return v; }
    return ((v + a - 1ull) / a) * a;
}

static int pmem_read_text(const ps_pmem_env_t *env, const char *path,
                          char *buf, size_t cap)
{
    int fd;
    long n;
    size_t i;

    if (!env || !env->open || !env->read || !env->close || !path || !buf || cap == 0u) {
        return PS_PMEM_ERR_ARGS;
    }
    fd = env->open(path, 0 /* 只读 */);
    if (fd < 0) { return PS_PMEM_ERR_OPEN; }
    n = env->read(fd, buf, (unsigned long)(cap - 1u));
    (void)env->close(fd);
    if (n < 0) { return PS_PMEM_ERR_OPEN; }
    buf[(size_t)n < cap ? (size_t)n : cap - 1u] = '\0';
    for (i = 0; i < (size_t)n && i < cap; ++i) {
        if (buf[i] == '\n' || buf[i] == '\r') { buf[i] = '\0'; break; }
    }
    return PS_PMEM_OK;
}

static int pmem_file_contains(const ps_pmem_env_t *env, const char *path,
                              const char *needle)
{
    static char buf[4096];      /* /proc/meminfo 一行行很长，一次读 4 KiB 足够 */
    if (pmem_read_text(env, path, buf, sizeof(buf)) != PS_PMEM_OK) { return 0; }
    return (strstr(buf, needle) != NULL) ? 1 : 0;
}

/* ---------------------------------------------------------------- 探测后端 */

int ps_pmem_find_udmabuf(const ps_pmem_env_t *env,
                         char *dev, size_t dev_cap,
                         char *sysdir, size_t dir_cap)
{
    static const char *names[] = { "udmabuf0", "udmabuf1", "udmabuf2", "udmabuf3" };
    unsigned i;

    if (!env || !env->exists || !dev || !sysdir || dev_cap == 0u || dir_cap == 0u) {
        return PS_PMEM_ERR_ARGS;
    }
    for (i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
        char d[64], s1[96], s2[96];
        snprintf(d, sizeof(d), "/dev/%s", names[i]);
        if (!env->exists(d)) { continue; }
        snprintf(s1, sizeof(s1), "/sys/class/u-dma-buf/%s", names[i]);
        snprintf(s2, sizeof(s2), "/sys/class/udmabuf/%s", names[i]);
        if (env->exists(s1)) {
            snprintf(dev, dev_cap, "%s", d);
            snprintf(sysdir, dir_cap, "%s", s1);
            return PS_PMEM_OK;
        }
        if (env->exists(s2)) {
            snprintf(dev, dev_cap, "%s", d);
            snprintf(sysdir, dir_cap, "%s", s2);
            return PS_PMEM_OK;
        }
    }
    return PS_PMEM_ERR_NO_BACKEND;
}

int ps_pmem_find_dma_heap(const ps_pmem_env_t *env, int want_contig,
                          char *path, size_t cap)
{
    char names[PS_PMEM_MAX_NAMES][PS_PMEM_NAME_CAP];
    int n, i;

    if (!env || !env->list_dir || !path || cap == 0u) { return 0; }
    n = env->list_dir("/dev/dma_heap", names, PS_PMEM_MAX_NAMES);
    if (n <= 0) { return 0; }
    if (n > PS_PMEM_MAX_NAMES) { n = PS_PMEM_MAX_NAMES; }

    /* 先找物理连续有保证的（名字含 cma / reserved）。 */
    for (i = 0; i < n; ++i) {
        if (strstr(names[i], "cma") != NULL || strstr(names[i], "reserved") != NULL) {
            snprintf(path, cap, "/dev/dma_heap/%s", names[i]);
            return 1;
        }
    }
    if (want_contig) { return 0; }
    /* 退而取任意一个（含 system；调用方已显式允许才走到这里）。 */
    if (n > 0) {
        snprintf(path, cap, "/dev/dma_heap/%s", names[0]);
        return 1;
    }
    return 0;
}

int ps_pmem_probe(const ps_pmem_env_t *env, uint32_t *kinds,
                  char *reason, size_t reason_cap)
{
    uint32_t bits = 0u;
    char dev[128], dir[96], heap[128];
    int has_udmabuf, has_dmaheap, has_cma, has_devmem;

    if (reason && reason_cap > 0u) { reason[0] = '\0'; }
    if (kinds) { *kinds = 0u; }
    if (!env) { return PS_PMEM_ERR_ARGS; }

    has_udmabuf = (ps_pmem_find_udmabuf(env, dev, sizeof(dev), dir, sizeof(dir))
                   == PS_PMEM_OK) ? 1 : 0;
    has_dmaheap = ps_pmem_find_dma_heap(env, 0, heap, sizeof(heap));
    has_cma     = pmem_file_contains(env, "/proc/meminfo", "CmaTotal");
    has_devmem  = (env->exists && env->exists("/dev/mem") &&
                   env->devmem_bytes > 0ull) ? 1 : 0;

    if (has_udmabuf) { bits |= (1u << PS_PMEM_KIND_UDMABUF); }
    if (has_dmaheap) { bits |= (1u << PS_PMEM_KIND_DMAHEAP); }
    if (has_cma)     { bits |= (1u << PS_PMEM_KIND_CMA); }
    if (has_devmem)  { bits |= (1u << PS_PMEM_KIND_DEVMEM); }

    if (kinds) { *kinds = bits; }
    if (reason && reason_cap > 0u) {
        snprintf(reason, reason_cap,
                 "udmabuf=%d dma-head=%d cma=%d devmem=%d%s",
                 has_udmabuf, has_dmaheap, has_cma, has_devmem,
                 (bits == 0u) ? "（三条路都不可用）" : "");
    }
    return PS_PMEM_OK;
}

/* ---------------------------------------------------------------- 具体分配 */

static int pmem_alloc_udmabuf(const ps_pmem_env_t *env, uint64_t bytes,
                              ps_pmem_block_t *out, char *reason, size_t cap)
{
    char dev[128], dir[96], text[64];
    char size_path[160], phys_path[160];
    uint64_t dt_size = 0ull, phys = 0ull, len;
    int fd;

    if (ps_pmem_find_udmabuf(env, dev, sizeof(dev), dir, sizeof(dir)) != PS_PMEM_OK) {
        return PS_PMEM_ERR_NO_BACKEND;
    }
    snprintf(size_path, sizeof(size_path), "%s/size", dir);
    if (pmem_read_text(env, size_path, text, sizeof(text)) != PS_PMEM_OK ||
        !ps_pmem_parse_u64(text, &dt_size) || dt_size == 0ull) {
        if (reason && cap > 0u) { snprintf(reason, cap, "读 %s 失败", size_path); }
        return PS_PMEM_ERR_OPEN;
    }
    if (bytes > dt_size) {
        if (reason && cap > 0u) {
            snprintf(reason, cap, "请求 %llu B > udmabuf %llu B（大小在设备树里固定）",
                     (unsigned long long)bytes, (unsigned long long)dt_size);
        }
        return PS_PMEM_ERR_SIZE;
    }
    snprintf(phys_path, sizeof(phys_path), "%s/phys_addr", dir);
    if (pmem_read_text(env, phys_path, text, sizeof(text)) != PS_PMEM_OK ||
        !ps_pmem_parse_u64(text, &phys) || phys == 0ull) {
        if (reason && cap > 0u) {
            snprintf(reason, cap, "读 %s 失败或为 0（拿不到物理地址）", phys_path);
        }
        return PS_PMEM_ERR_PHYS;
    }

    fd = env->open(dev, PMEM_O_RDWR | 0x101000); /* O_SYNC: u-dma-buf uncached mapping. */
    if (fd < 0) {
        if (reason && cap > 0u) { snprintf(reason, cap, "open(%s) 失败 errno=%d", dev, -fd); }
        return PS_PMEM_ERR_OPEN;
    }
    len = pmem_round_up(bytes, (uint64_t)env->page_size);
    if (len > dt_size) { len = dt_size; }
    out->virt = env->map(NULL, (unsigned long)len, PMEM_PROT_RW, PMEM_MAP_SHARED, fd, 0L);
    if (!out->virt) {
        (void)env->close(fd);
        if (reason && cap > 0u) { snprintf(reason, cap, "mmap(%s) 失败", dev); }
        return PS_PMEM_ERR_MMAP;
    }
    out->kind       = PS_PMEM_KIND_UDMABUF;
    out->fd         = fd;
    out->bytes      = len;
    out->phys       = phys;
    out->phys_valid = 1;
    snprintf(out->dev_path, sizeof(out->dev_path), "%s", dev);
    return PS_PMEM_OK;
}

static int pmem_alloc_dmaheap(const ps_pmem_env_t *env, uint64_t bytes,
                              uint32_t flags, ps_pmem_block_t *out,
                              char *reason, size_t cap)
{
    char heap[128];
    ps_dma_heap_alloc_t data;
    uint64_t len, phys = 0ull;
    int fd, dfd;

    /* 先要连续的（cma/reserved）；没有且调用方允许才退到 system。 */
    if (!ps_pmem_find_dma_heap(env, 1, heap, sizeof(heap))) {
        if (!(flags & PS_PMEM_F_ALLOW_SYSTEM_HEAP) ||
            !ps_pmem_find_dma_heap(env, 0, heap, sizeof(heap))) {
            return PS_PMEM_ERR_NO_BACKEND;
        }
    }
    if (!env->ioctl || env->dma_heap_alloc_ioctl == 0ul) { return PS_PMEM_ERR_NO_BACKEND; }

    fd = env->open(heap, PMEM_O_RDWR);
    if (fd < 0) {
        if (reason && cap > 0u) { snprintf(reason, cap, "open(%s) 失败 errno=%d", heap, -fd); }
        return PS_PMEM_ERR_OPEN;
    }
    memset(&data, 0, sizeof(data));
    data.len       = bytes;
    data.fd_flags  = 0x80002u;   /* O_RDWR | O_CLOEXEC */
    data.heap_flags = 0u;
    if (env->ioctl(fd, env->dma_heap_alloc_ioctl, &data) != 0) {
        (void)env->close(fd);
        if (reason && cap > 0u) {
            snprintf(reason, cap, "DMA_HEAP_IOCTL_ALLOC(%s, %llu B) 失败", heap,
                     (unsigned long long)bytes);
        }
        return PS_PMEM_ERR_IOCTL;
    }
    dfd = (int)data.fd;
    len = pmem_round_up(bytes, (uint64_t)env->page_size);
    out->virt = env->map(NULL, (unsigned long)len, PMEM_PROT_RW, PMEM_MAP_SHARED, dfd, 0L);
    if (!out->virt) {
        (void)env->close(dfd);
        (void)env->close(fd);
        if (reason && cap > 0u) { snprintf(reason, cap, "mmap(dmabuf fd) 失败"); }
        return PS_PMEM_ERR_MMAP;
    }
    if (env->dmabuf_phys &&
        env->dmabuf_phys(dfd, out->virt, len, &phys) == 0 && phys != 0ull) {
        out->phys = phys;
        out->phys_valid = 1;
    } else {
        out->phys = 0ull;
        out->phys_valid = 0;
    }
    if (!out->phys_valid && (flags & PS_PMEM_F_REQUIRE_PHYS)) {
        (void)env->unmap(out->virt, (unsigned long)len);
        (void)env->close(dfd);
        (void)env->close(fd);
        memset(out, 0, sizeof(*out));
        out->fd = -1;
        if (reason && cap > 0u) {
            snprintf(reason, cap,
                     "dma-heap 拿不到物理地址（mainline 无标准接口；仅 cma/reserved 且驱动提供才行）");
        }
        return PS_PMEM_ERR_PHYS;
    }
    out->kind  = PS_PMEM_KIND_DMAHEAP;
    out->fd    = dfd;
    out->bytes = len;
    snprintf(out->dev_path, sizeof(out->dev_path), "%s", heap);
    (void)env->close(fd);   /* dmabuf fd 已持有引用，heap fd 可关 */
    return PS_PMEM_OK;
}

static int pmem_alloc_devmem(const ps_pmem_env_t *env, uint64_t bytes,
                             uint32_t flags, ps_pmem_block_t *out,
                             char *reason, size_t cap)
{
    uint64_t len;
    int fd;

    if (!(flags & PS_PMEM_F_ALLOW_DEVMEM)) { return PS_PMEM_ERR_NO_BACKEND; }
    if (env->devmem_base == 0ull || env->devmem_bytes == 0ull) { return PS_PMEM_ERR_NO_BACKEND; }
    if (!env->exists || !env->exists("/dev/mem")) { return PS_PMEM_ERR_NO_BACKEND; }
    if (bytes > env->devmem_bytes) {
        if (reason && cap > 0u) {
            snprintf(reason, cap, "请求 %llu B > /dev/mem 预留 %llu B",
                     (unsigned long long)bytes, (unsigned long long)env->devmem_bytes);
        }
        return PS_PMEM_ERR_SIZE;
    }
    fd = env->open("/dev/mem", PMEM_O_RDWR);
    if (fd < 0) {
        if (reason && cap > 0u) { snprintf(reason, cap, "open(/dev/mem) 失败 errno=%d", -fd); }
        return PS_PMEM_ERR_OPEN;
    }
    len = pmem_round_up(bytes, (uint64_t)env->page_size);
    out->virt = env->map(NULL, (unsigned long)len, PMEM_PROT_RW, PMEM_MAP_SHARED,
                         fd, (long)env->devmem_base);
    if (!out->virt) {
        (void)env->close(fd);
        if (reason && cap > 0u) {
            snprintf(reason, cap, "mmap(/dev/mem, 0x%llX) 失败（CONFIG_STRICT_DEVMEM？）",
                     (unsigned long long)env->devmem_base);
        }
        return PS_PMEM_ERR_MMAP;
    }
    out->kind       = PS_PMEM_KIND_DEVMEM;
    out->fd         = fd;
    out->bytes      = len;
    out->phys       = env->devmem_base;
    out->phys_valid = 1;
    snprintf(out->dev_path, sizeof(out->dev_path), "/dev/mem");
    return PS_PMEM_OK;
}

int ps_pmem_alloc(const ps_pmem_env_t *env, uint64_t bytes, uint32_t flags,
                  ps_pmem_block_t *out, char *reason, size_t reason_cap)
{
    int rc;

    if (reason && reason_cap > 0u) { reason[0] = '\0'; }
    if (!env || !out || bytes == 0ull || bytes > (uint64_t)SIZE_MAX - 4096ull) { return PS_PMEM_ERR_ARGS; }
    if (!env->open || !env->close || !env->map || !env->unmap ||
        env->page_size == 0u) {
        return PS_PMEM_ERR_ARGS;
    }
    memset(out, 0, sizeof(*out));
    out->fd = -1;

    /* 顺序：udmabuf（phys 可直读）-> dma-heap -> /dev/mem 回退。 */
    rc = pmem_alloc_udmabuf(env, bytes, out, reason, reason_cap);
    if (rc == PS_PMEM_OK) { return PS_PMEM_OK; }
    if (rc == PS_PMEM_ERR_PHYS || rc == PS_PMEM_ERR_SIZE || rc == PS_PMEM_ERR_MMAP ||
        rc == PS_PMEM_ERR_OPEN) {
        /* udmabuf 存在但这一步失败：如实上报，不偷偷换后端掩盖。 */
        return rc;
    }

    if (flags & PS_PMEM_F_REQUIRE_UDMABUF) return PS_PMEM_ERR_NO_BACKEND;
    rc = pmem_alloc_dmaheap(env, bytes, flags, out, reason, reason_cap);
    if (rc == PS_PMEM_OK) { return PS_PMEM_OK; }
    if (rc != PS_PMEM_ERR_NO_BACKEND) { return rc; }

    rc = pmem_alloc_devmem(env, bytes, flags, out, reason, reason_cap);
    if (rc == PS_PMEM_OK) { return PS_PMEM_OK; }
    if (rc != PS_PMEM_ERR_NO_BACKEND) { return rc; }

    if (reason && reason_cap > 0u) {
        snprintf(reason, reason_cap,
                 "无可用后端：udmabuf 未配、dma-heap 无 cma/reserved heap，"
                 "/dev/mem 回退未开启或未预留");
    }
    return PS_PMEM_ERR_NO_BACKEND;
}

void ps_pmem_free(const ps_pmem_env_t *env, ps_pmem_block_t *b)
{
    if (!b) { return; }
    if (env) {
        if (env->unmap && b->virt && b->bytes > 0ull) {
            (void)env->unmap(b->virt, (unsigned long)b->bytes);
        }
        if (env->close && b->fd >= 0) { (void)env->close(b->fd); }
    }
    memset(b, 0, sizeof(*b));
    b->fd = -1;
}
