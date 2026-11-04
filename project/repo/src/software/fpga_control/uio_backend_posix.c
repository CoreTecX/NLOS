/* ============================================================================
 * PenumbraSight · 板端 UIO 访问后端（POSIX/Linux 系统调用）
 * src/software/fpga_control/uio_backend_posix.c
 *
 * 只有 Linux 构建会编到内容；主机（MinGW）下整份文件是空 TU，从而
 * src/software/Makefile 的「board_app/fpga_control 目录下 .c 全量入模块库」
 * 这条规则不会把 <sys/mman.h> 带进主机构建。
 *
 * 这里把 ps_uio_env_t 的三个平台原语落到真系统调用上：
 *   open/read/close、opendir/readdir（列 /sys/class/uio）、mmap/munmap。
 * 逻辑（扫描、覆盖校验、页对齐、错误码）全在 uio_backend.c，本文件不做判断。
 *
 * 未在板上跑过：本仓库无 aarch64 板卡在环，见 uio_backend.h 的未验证声明。
 * ==========================================================================*/

#if defined(__linux__)

#include "uio_backend.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/mman.h>

/* 打开：失败时把 errno 取负返回，调用方据此区分 EACCES。 */
static int posix_open(const char *path, int flags)
{
    int fd = open(path, flags);
    if (fd < 0) { return -errno; }
    return fd;
}

static int posix_close(int fd)
{
    return close(fd);
}

static long posix_read(int fd, void *buf, unsigned long n)
{
    ssize_t r = read(fd, buf, (size_t)n);
    if (r < 0) { return -(long)errno; }
    return (long)r;
}

/* 列目录：只取条目名，跳过 "." / ".."。返回条目数，失败返回负值。 */
static int posix_list_dir(const char *dir,
                          char names[][PS_UIO_NAME_CAP], int max)
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
        if (strlen(e->d_name) >= PS_UIO_NAME_CAP) continue;
        memcpy(names[n], e->d_name, strlen(e->d_name) + 1);
        ++n;
    }
    closedir(d);
    return n;
}

static void *posix_map(void *addr, unsigned long len, int prot, int flags,
                       int fd, long off)
{
    /* PS_UIO_PROT_RD=PROT_READ 等取值与 Linux 一致（见 uio_backend.h），直接透传。 */
    void *p = mmap(addr, (size_t)len, prot, flags, fd, (off_t)off);
    if (p == MAP_FAILED) { return NULL; }
    return p;
}

static int posix_unmap(void *addr, unsigned long len)
{
    return munmap(addr, (size_t)len);
}

const ps_uio_env_t *ps_uio_env_posix(void)
{
    /* 惰性初始化：page_size 只能在运行期取（sysconf）。 */
    static ps_uio_env_t dyn;
    static int inited = 0;
    if (!inited) {
        long ps = sysconf(_SC_PAGESIZE);
        dyn.open      = posix_open;
        dyn.close     = posix_close;
        dyn.read      = posix_read;
        dyn.list_dir  = posix_list_dir;
        dyn.map       = posix_map;
        dyn.unmap     = posix_unmap;
        dyn.page_size = (ps > 0) ? (unsigned long)ps : 4096ul;
        inited = 1;
    }
    return &dyn;
}

#endif /* __linux__ */
