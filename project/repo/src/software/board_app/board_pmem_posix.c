/* ============================================================================
 * PenumbraSight · 板端连续物理内存分配（POSIX/Linux 系统调用）
 * src/software/board_app/board_pmem_posix.c
 *
 * 只在 Linux 构建里编到内容；主机（MinGW）下是空 TU。
 *
 * 这里的 dmabuf_phys 是**尽力而为**：mainline 没有「读 dmabuf 物理地址」的标准
 * 接口，本实现退到 /proc/self/pagemap 查 PFN（需要权限，且对 dma-buf 映射常常
 * 拿不到，拿不到就返回 -1，调用方据此置 phys_valid=0，不猜）。若换到在 sysfs 里
 * 给出 phys 的厂商 heap，改这个钩子即可。
 * ==========================================================================*/

#if defined(__linux__)

#include "board_pmem.h"

#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

static int posix_exists(const char *path)
{
    return (access(path, F_OK) == 0) ? 1 : 0;
}

static int posix_open(const char *path, int flags)
{
    if (!strcmp(path, "/dev/mem")) flags |= O_SYNC;
    int fd = open(path, flags);
    if (fd < 0) { return -errno; }
    return fd;
}

static int posix_close(int fd) { return close(fd); }

static long posix_read(int fd, void *buf, unsigned long n)
{
    ssize_t r = read(fd, buf, (size_t)n);
    if (r < 0) { return -(long)errno; }
    return (long)r;
}

static int posix_ioctl(int fd, unsigned long req, void *arg)
{
    return ioctl(fd, req, arg);
}

static int posix_list_dir(const char *dir,
                          char names[][PS_PMEM_NAME_CAP], int max)
{
    DIR *d;
    struct dirent *e;
    int n = 0;

    if (!dir || !names || max <= 0) { return -EINVAL; }
    d = opendir(dir);
    if (!d) { return -errno; }
    while ((e = readdir(d)) != NULL && n < max) {
        if (e->d_name[0] == '.' &&
            (e->d_name[1] == '\0' ||
             (e->d_name[1] == '.' && e->d_name[2] == '\0'))) {
            continue;
        }
        if (strlen(e->d_name) >= PS_PMEM_NAME_CAP) continue;
        memcpy(names[n], e->d_name, strlen(e->d_name) + 1);
        ++n;
    }
    closedir(d);
    return n;
}

static void *posix_map(void *addr, unsigned long len, int prot, int flags,
                       int fd, long off)
{
    void *p = mmap(addr, (size_t)len, prot, flags, fd, (off_t)off);
    if (p == MAP_FAILED) { return NULL; }
    return p;
}

static int posix_unmap(void *addr, unsigned long len)
{
    return munmap(addr, (size_t)len);
}

/* Raw m_axi needs physical, not IOVA addresses. Verify EVERY page and reject
 * fragmented allocations; looking at only the first PFN is insufficient. */
static int posix_dmabuf_phys(int fd, void *virt, uint64_t len, uint64_t *phys)
{
    long ps = sysconf(_SC_PAGESIZE);
    uint64_t va, first = 0, count, i;
    int pf;
    (void)fd;
    if (!virt || !phys || ps <= 0 || !len) return -1;
    va = (uint64_t)(uintptr_t)virt;
    if (len > UINT64_MAX - (va % (uint64_t)ps) - (uint64_t)ps) return -1;
    count = (len + va % (uint64_t)ps + (uint64_t)ps - 1) / (uint64_t)ps;
    pf = open("/proc/self/pagemap", O_RDONLY | O_CLOEXEC);
    if (pf < 0) return -1;
    for (i = 0; i < count; ++i) {
        uint64_t ent = 0, pfn;
        off_t off = (off_t)((va / (uint64_t)ps + i) * 8);
        if (pread(pf, &ent, sizeof(ent), off) != sizeof(ent) || !(ent & (1ull << 63))) {
            close(pf); return -1;
        }
        pfn = ent & ((1ull << 55) - 1);
        if (!pfn || (i && pfn != first + i)) { close(pf); return -1; }
        if (!i) first = pfn;
    }
    close(pf);
    *phys = first * (uint64_t)ps + va % (uint64_t)ps;
    return 0;
}

const ps_pmem_env_t *ps_pmem_env_posix(void)
{
    /* 惰性初始化：page_size 与 ioctl 编码只能在运行期取。 */
    static ps_pmem_env_t dyn;
    static int inited = 0;
    if (!inited) {
        long ps = sysconf(_SC_PAGESIZE);
        dyn.exists    = posix_exists;
        dyn.open      = posix_open;
        dyn.close     = posix_close;
        dyn.read      = posix_read;
        dyn.ioctl     = posix_ioctl;
        dyn.list_dir  = posix_list_dir;
        dyn.map       = posix_map;
        dyn.unmap     = posix_unmap;
        dyn.dmabuf_phys = posix_dmabuf_phys;
        dyn.page_size = (ps > 0) ? (unsigned long)ps : 4096ul;
        /* 与 uapi/linux/dma-heap.h 的 DMA_HEAP_IOCTL_ALLOC 同值：
         * _IOWR('H', 0x00, struct dma_heap_allocation_data)，结构 24 字节。 */
        dyn.dma_heap_alloc_ioctl = (unsigned long)_IOWR('H', 0x00, ps_dma_heap_alloc_t);
        /* /dev/mem 回退：默认关闭（区间为 0）。要用时由调用方拷一份 env 填区间。 */
        dyn.devmem_base  = 0ull;
        dyn.devmem_bytes = 0ull;
        inited = 1;
    }
    return &dyn;
}

#endif /* __linux__ */
