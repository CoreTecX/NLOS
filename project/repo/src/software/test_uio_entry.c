/* ============================================================================
 * PenumbraSight · UIO 入口与连续物理内存分配的主机单测
 * src/software/test_uio_entry.c
 *
 * 可立即运行（不依赖板卡；设备访问全部走注入的假后端）：
 *     make -C src/software test-uio-entry
 * 或在仓库根：make test-uio-entry
 * 退出码非零表示失败。
 *
 * 覆盖
 * ---------------------------------------------------------------------------
 *   A. UIO 后端逻辑：
 *        · 正常映射（按名字扫描命中、map0 addr/size、读写窗口、close）；
 *        · 设备缺失（扫描不到 / 显式路径不存在）；
 *        · 权限不足（open 返回 EACCES）；
 *        · 映射长度不对（map0 size 小于所需窗口、addr 覆盖不到所需基址、size 读不到）；
 *        · mmap 失败；
 *        · sysfs 数值解析与页对齐。
 *   B. 连续物理内存分配：
 *        · udmabuf 正常（phys_addr 直读）；
 *        · udmabuf 请求超过设备树大小；
 *        · udmabuf 拿不到 phys_addr（物理地址获取失败）；
 *        · dma-heap 在「要求 phys」时拿不到 phys 报错、不要求时给出 phys_valid=0；
 *        · dma-heap 通过钩子拿到 phys；
 *        · /dev/mem 回退：仅显式允许时可用；
 *        · 无后端；
 *        · 探测位掩码。
 *
 * 不覆盖（只能在板上验）
 * ---------------------------------------------------------------------------
 *   真 /dev/uioN 与真 mmap 的时序、板端 map0 的 addr/size 实际取值、内核是否
 *   编了 CONFIG_UIO / U-DMA-BUF / DMA_HEAP、真 udmabuf 的 phys_addr、真 dma-heap
 *   的 phys 是否可读、/dev/mem 的 STRICT_DEVMEM 放宽 —— 一律真板实测，本文件不代跑。
 * ==========================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "uio_backend.h"
#include "board_pmem.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do {                                   \
        if (cond) { g_pass++; printf("  PASS  %s\n", (msg)); }  \
        else { g_fail++; printf("  FAIL  %s\n", (msg)); }       \
    } while (0)

/* ==========================================================================
 * 假 syscall 层（UIO 与 pmem 共用；两边的函数指针类型逐字相同）
 * ========================================================================*/

#define FK_MAX_FILES 32
#define FK_MAX_DIRS   6
#define FK_MAX_ITEMS  8
#define FK_MAX_FD    64

static char g_f_path[FK_MAX_FILES][192];
static char g_f_data[FK_MAX_FILES][512];
static int  g_f_n;

static char g_d_path[FK_MAX_DIRS][128];
static char g_d_item[FK_MAX_DIRS][FK_MAX_ITEMS][64];
static int  g_d_count[FK_MAX_DIRS];
static int  g_d_n;

static const char *g_f_fd[FK_MAX_FD];   /* fd -> 文件内容指针；设备为 NULL */
static int  g_f_fd_isdev[FK_MAX_FD];
static int  g_f_next_fd;

static char g_deny_path[192];          /* open 返回 -13 */
static char g_nodev_path[192];         /* open 返回 -2 */
static int  g_map_fail;
static int  g_ioctl_fail;
static int  g_dmabuf_phys_ok;
static uint64_t g_dmabuf_phys_val;
static int  g_ioctl_calls;

static int fk_strncopy(char *dst, size_t cap, const char *src)
{
    if (!src) { dst[0] = '\0'; return 0; }
    snprintf(dst, cap, "%s", src);
    return 1;
}

static int fk_file_index(const char *path);   /* 前置声明：fk_put 里要用 */

static void fk_reset(void)
{
    g_f_n = 0; g_d_n = 0; g_f_next_fd = 3;
    memset(g_f_fd, 0, sizeof(g_f_fd));
    memset(g_f_fd_isdev, 0, sizeof(g_f_fd_isdev));
    memset(g_deny_path, 0, sizeof(g_deny_path));
    memset(g_nodev_path, 0, sizeof(g_nodev_path));
    g_map_fail = 0; g_ioctl_fail = 0;
    g_dmabuf_phys_ok = 0; g_dmabuf_phys_val = 0ull;
    g_ioctl_calls = 0;
}

static void fk_put(const char *path, const char *data)
{
    int idx = fk_file_index(path);
    if (idx >= 0) {
        /* 已注册则覆盖：否则重复注册时旧值会被先命中，改文件不生效。 */
        fk_strncopy(g_f_data[idx], sizeof(g_f_data[0]), data);
        return;
    }
    if (g_f_n >= FK_MAX_FILES) { return; }
    fk_strncopy(g_f_path[g_f_n], sizeof(g_f_path[0]), path);
    fk_strncopy(g_f_data[g_f_n], sizeof(g_f_data[0]), data);
    ++g_f_n;
}

static void fk_dir(const char *path, const char *const *items, int n)
{
    int i, j;
    if (g_d_n >= FK_MAX_DIRS) { return; }
    j = g_d_n++;
    fk_strncopy(g_d_path[j], sizeof(g_d_path[0]), path);
    g_d_count[j] = 0;
    for (i = 0; i < n && i < FK_MAX_ITEMS; ++i) {
        fk_strncopy(g_d_item[j][i], 64, items[i]);
        g_d_count[j]++;
    }
}

static int fk_file_index(const char *path)
{
    int i;
    for (i = 0; i < g_f_n; ++i) {
        if (strcmp(g_f_path[i], path) == 0) { return i; }
    }
    return -1;
}

static int fk_exists(const char *path)
{
    int i;
    if (!path) { return 0; }
    if (fk_file_index(path) >= 0) { return 1; }
    for (i = 0; i < g_f_n; ++i) {
        size_t l = strlen(path);
        if (strncmp(g_f_path[i], path, l) == 0 && g_f_path[i][l] == '/') { return 1; }
    }
    if (strcmp(path, g_nodev_path) == 0 && g_nodev_path[0] != '\0') { return 0; }
    if (strncmp(path, "/dev/", 5) == 0) { return 1; }
    return 0;
}

static int fk_open(const char *path, int flags)
{
    int idx, fd;
    (void)flags;
    if (path && g_deny_path[0] != '\0' && strcmp(path, g_deny_path) == 0) { return -13; }
    if (path && g_nodev_path[0] != '\0' && strcmp(path, g_nodev_path) == 0) { return -2; }
    idx = fk_file_index(path);
    fd = g_f_next_fd++;
    if (fd >= FK_MAX_FD) { return -24; }
    if (idx >= 0) {
        g_f_fd[fd] = g_f_data[idx];
        g_f_fd_isdev[fd] = 0;
        return fd;
    }
    if (path && strncmp(path, "/dev/", 5) == 0) {
        g_f_fd[fd] = NULL;
        g_f_fd_isdev[fd] = 1;
        return fd;
    }
    return -2;   /* ENOENT */
}

static int fk_close(int fd)
{
    if (fd >= 0 && fd < FK_MAX_FD) { g_f_fd[fd] = NULL; g_f_fd_isdev[fd] = 0; }
    return 0;
}

static long fk_read(int fd, void *buf, unsigned long n)
{
    const char *s;
    size_t l;
    if (fd < 0 || fd >= FK_MAX_FD) { return -9; }
    s = g_f_fd[fd];
    if (!s) { return -5; }
    l = strlen(s);
    if (l > n) { l = n; }
    memcpy(buf, s, l);
    return (long)l;
}

static int fk_list_dir(const char *dir, char names[][64], int max)
{
    int i, j, n = 0;
    for (i = 0; i < g_d_n; ++i) {
        if (strcmp(g_d_path[i], dir) == 0) {
            for (j = 0; j < g_d_count[i] && n < max; ++j) {
                fk_strncopy(names[n], 64, g_d_item[i][j]);
                ++n;
            }
            return n;
        }
    }
    return 0;
}

static void *fk_map(void *addr, unsigned long len, int prot, int flags,
                    int fd, long off)
{
    (void)addr; (void)prot; (void)flags; (void)fd; (void)off;
    if (g_map_fail) { return NULL; }
    return malloc((size_t)(len ? len : 1u));
}

static int fk_unmap(void *addr, unsigned long len)
{
    (void)len;
    free(addr);
    return 0;
}

static int fk_ioctl(int fd, unsigned long req, void *arg)
{
    ps_dma_heap_alloc_t *d = (ps_dma_heap_alloc_t *)arg;
    (void)fd;
    ++g_ioctl_calls;
    if (req != (unsigned long)0xC0184800ul) { return -25; }
    if (g_ioctl_fail) { return -1; }
    if (d) { d->fd = 700u; }
    return 0;
}

static int fk_dmabuf_phys(int fd, void *virt, uint64_t len, uint64_t *phys)
{
    (void)fd; (void)virt; (void)len;
    if (!g_dmabuf_phys_ok) { return -1; }
    if (phys) { *phys = g_dmabuf_phys_val; }
    return 0;
}

static const ps_uio_env_t g_uio_env = {
    fk_open, fk_close, fk_read, fk_list_dir, fk_map, fk_unmap, 4096ul
};

static ps_pmem_env_t g_pmem_env;   /* 运行期填（见 make_pmem_env） */

static void make_pmem_env(void)
{
    memset(&g_pmem_env, 0, sizeof(g_pmem_env));
    g_pmem_env.exists          = fk_exists;
    g_pmem_env.open            = fk_open;
    g_pmem_env.close           = fk_close;
    g_pmem_env.read            = fk_read;
    g_pmem_env.ioctl           = fk_ioctl;
    g_pmem_env.map             = fk_map;
    g_pmem_env.unmap           = fk_unmap;
    g_pmem_env.list_dir        = fk_list_dir;
    g_pmem_env.dmabuf_phys     = fk_dmabuf_phys;
    g_pmem_env.page_size       = 4096ul;
    g_pmem_env.dma_heap_alloc_ioctl = 0xC0184800ul;
    g_pmem_env.devmem_base     = 0ull;
    g_pmem_env.devmem_bytes    = 0ull;
}

/* ==========================================================================
 * A. UIO 后端
 * ========================================================================*/

/* 注册一套可用的设备树映射：uio0 = penumbrasight，map0 = 0xA0000000 / 0x80000。 */
static void fk_install_good_uio(void)
{
    static const char *items[] = { "uio0" };
    fk_dir("/sys/class/uio", items, 1);
    fk_put("/sys/class/uio/uio0/name", "penumbrasight_regs@a0000000\n");
    fk_put("/sys/class/uio/uio0/maps/map0/size", "0x00080000\n");
    fk_put("/sys/class/uio/uio0/maps/map0/addr", "0xA0000000\n");
}

static void test_uio_helpers(void)
{
    uint64_t v = 0ull;
    printf("\n---- A0. 解析与对齐 ----\n");
    CHECK(ps_uio_parse_u64("0x00080000\n", &v) && v == 0x80000ull, "解析十六进制 0x80000");
    CHECK(ps_uio_parse_u64(" 67108864 ", &v) && v == 67108864ull, "解析十进制 67108864");
    CHECK(!ps_uio_parse_u64("", &v), "空串判失败");
    CHECK(!ps_uio_parse_u64("0xZZ", &v), "非数字判失败（不截断）");
    CHECK(ps_uio_round_up(0x80000ull, 4096ull) == 0x80000ull, "已对齐不变");
    CHECK(ps_uio_round_up(0x80001ull, 4096ull) == 0x81000ull, "非对齐向上取整到页");
}

static void test_uio_normal(void)
{
    ps_uio_map_t m;
    char err[256];
    int rc;

    printf("\n---- A1. 正常映射 ----\n");
    fk_reset();
    fk_install_good_uio();

    rc = ps_uio_open_map(&m, &g_uio_env, "auto", PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, sizeof(err));
    CHECK(rc == PS_UIO_OK, "按名字扫描命中并打开（auto）");
    CHECK(strcmp(m.dev_path, "/dev/uio0") == 0 && strcmp(m.name, "uio0") == 0,
          "命中设备 = /dev/uio0（sysfs name = uio0）");
    CHECK(m.map_phys == 0xA0000000ull, "map0 物理基址 = 0xA0000000");
    CHECK(m.map_bytes == 0x80000ull, "map_bytes = 0x80000（页对齐）");
    CHECK(m.map != NULL, "映射指针非空");

    /* 映射出来的窗口可读写（模拟寄存器）。 */
    if (m.map) {
        volatile uint32_t *w = (volatile uint32_t *)m.map;
        w[0] = 0x12345678u;
        CHECK(w[0] == 0x12345678u, "映射窗口读写一致");
    }
    ps_uio_close_map(&g_uio_env, &m);
    CHECK(m.map == NULL && m.fd == -1, "关闭后句柄清零");

    /* 显式路径同样可用。 */
    rc = ps_uio_open_map(&m, &g_uio_env, "/dev/uio0", PS_UIO_REQ_BASE,
                         PS_UIO_REQ_BYTES, err, sizeof(err));
    CHECK(rc == PS_UIO_OK, "显式路径 /dev/uio0 打开成功");
    ps_uio_close_map(&g_uio_env, &m);

    /* 更大的窗口（1 MiB）也接受（版本页落在窗口内）。 */
    fk_put("/sys/class/uio/uio0/maps/map0/size", "0x00100000\n");
    rc = ps_uio_open_map(&m, &g_uio_env, "auto", PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, sizeof(err));
    CHECK(rc == PS_UIO_OK && m.map_bytes == 0x100000ull, "1 MiB 窗口接受且长度正确");
    ps_uio_close_map(&g_uio_env, &m);

    /* 名字不匹配 -> 找不到。 */
    fk_reset();
    fk_install_good_uio();
    fk_put("/sys/class/uio/uio0/name", "some-other-ip\n");
    rc = ps_uio_open_map(&m, &g_uio_env, "auto", PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, sizeof(err));
    CHECK(rc == PS_UIO_ERR_NO_DEVICE, "name 不匹配令牌 -> NO_DEVICE");
}

static void test_uio_device_missing(void)
{
    ps_uio_map_t m;
    char err[256];
    int rc;

    printf("\n---- A2. 设备缺失 ----\n");
    fk_reset();
    rc = ps_uio_open_map(&m, &g_uio_env, "auto", PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, sizeof(err));
    CHECK(rc == PS_UIO_ERR_NO_DEVICE, "扫描不到 UIO 设备 -> NO_DEVICE");
    CHECK(err[0] != '\0', "给出可读原因");

    fk_reset();
    fk_install_good_uio();
    strcpy(g_nodev_path, "/dev/uio0");
    rc = ps_uio_open_map(&m, &g_uio_env, "/dev/uio0", PS_UIO_REQ_BASE,
                         PS_UIO_REQ_BYTES, err, sizeof(err));
    CHECK(rc == PS_UIO_ERR_NO_DEVICE, "显式设备节点不存在 -> NO_DEVICE");
}

static void test_uio_permission(void)
{
    ps_uio_map_t m;
    char err[256];
    int rc;

    printf("\n---- A3. 权限不足 ----\n");
    fk_reset();
    fk_install_good_uio();
    strcpy(g_deny_path, "/dev/uio0");
    rc = ps_uio_open_map(&m, &g_uio_env, "/dev/uio0", PS_UIO_REQ_BASE,
                         PS_UIO_REQ_BYTES, err, sizeof(err));
    CHECK(rc == PS_UIO_ERR_PERM, "open 返回 EACCES -> PERM");
    CHECK(strstr(err, "权限") != NULL, "原因里点明权限");
}

static void test_uio_bad_map_size(void)
{
    ps_uio_map_t m;
    char err[256];
    int rc;

    printf("\n---- A4. 映射长度不对 ----\n");
    /* map0 太小：覆盖不了 512 KiB。 */
    fk_reset();
    fk_install_good_uio();
    fk_put("/sys/class/uio/uio0/maps/map0/size", "0x00001000\n");
    rc = ps_uio_open_map(&m, &g_uio_env, "auto", PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, sizeof(err));
    CHECK(rc == PS_UIO_ERR_MAP_SIZE, "map0 只有 4 KiB -> MAP_SIZE");

    /* map0 起点不对：addr 高于所需基址，覆盖不到。 */
    fk_reset();
    fk_install_good_uio();
    fk_put("/sys/class/uio/uio0/maps/map0/addr", "0xA0008000\n");
    rc = ps_uio_open_map(&m, &g_uio_env, "auto", PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, sizeof(err));
    CHECK(rc == PS_UIO_ERR_MAP_SIZE, "map0 起始 0xA0008000 覆盖不到 0xA0000000 -> MAP_SIZE");

    /* size 文件读不到。 */
    fk_reset();
    fk_install_good_uio();
    fk_put("/sys/class/uio/uio0/maps/map0/size", "");
    rc = ps_uio_open_map(&m, &g_uio_env, "auto", PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, sizeof(err));
    CHECK(rc == PS_UIO_ERR_MAP_SIZE, "map0 size 读不到 -> MAP_SIZE");
}

static void test_uio_mmap_fail(void)
{
    ps_uio_map_t m;
    char err[256];
    int rc;

    printf("\n---- A5. mmap 失败 ----\n");
    fk_reset();
    fk_install_good_uio();
    g_map_fail = 1;
    rc = ps_uio_open_map(&m, &g_uio_env, "auto", PS_UIO_REQ_BASE, PS_UIO_REQ_BYTES,
                         err, sizeof(err));
    CHECK(rc == PS_UIO_ERR_MAP, "mmap 返回 NULL -> MAP");
    CHECK(strstr(err, "mmap") != NULL, "原因里点明 mmap");
}

/* ==========================================================================
 * B. 连续物理内存分配
 * ========================================================================*/

static void fk_install_good_udmabuf(void)
{
    fk_put("/sys/class/u-dma-buf/udmabuf0/size", "0x00100000\n");
    fk_put("/sys/class/u-dma-buf/udmabuf0/phys_addr", "0x0000000070000000\n");
}

static void test_pmem_udmabuf(void)
{
    ps_pmem_block_t b;
    char err[256];
    int rc;
    uint32_t kinds = 0u;

    printf("\n---- B1. udmabuf 正常 ----\n");
    fk_reset();
    fk_install_good_udmabuf();
    rc = ps_pmem_alloc(&g_pmem_env, 0x10000ull, PS_PMEM_F_REQUIRE_PHYS, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_OK, "udmabuf 分配成功");
    CHECK(b.kind == PS_PMEM_KIND_UDMABUF && b.phys_valid == 1, "kind=UDMABUF 且 phys 有效");
    CHECK(b.phys == 0x70000000ull, "phys 直读 = 0x70000000");
    CHECK(b.bytes == 0x10000ull, "映射长度 = 0x10000（页对齐）");
    ps_pmem_free(&g_pmem_env, &b);
    CHECK(b.virt == NULL && b.fd == -1, "free 后清零");

    rc = ps_pmem_probe(&g_pmem_env, &kinds, err, sizeof(err));
    CHECK(rc == PS_PMEM_OK && (kinds & (1u << PS_PMEM_KIND_UDMABUF)) != 0u,
          "probe 报告 udmabuf 可用");
}

static void test_pmem_udmabuf_limits(void)
{
    ps_pmem_block_t b;
    char err[256];
    int rc;

    printf("\n---- B2. udmabuf 容量与 phys 失败 ----\n");
    fk_reset();
    fk_install_good_udmabuf();
    rc = ps_pmem_alloc(&g_pmem_env, 0x200000ull, 0u, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_ERR_SIZE, "请求 > 设备树 size -> ERR_SIZE");

    /* 不注册 phys_addr：拿不到物理地址。 */
    fk_reset();
    fk_put("/sys/class/u-dma-buf/udmabuf0/size", "0x00100000\n");
    rc = ps_pmem_alloc(&g_pmem_env, 0x10000ull, PS_PMEM_F_REQUIRE_PHYS, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_ERR_PHYS, "udmabuf phys_addr 读不到 -> PHYS");
}

static void test_pmem_dmaheap(void)
{
    ps_pmem_block_t b;
    char err[256];
    int rc;
    static const char *heaps[] = { "linux,cma" };

    printf("\n---- B3. dma-heap ----\n");
    fk_reset();
    fk_dir("/dev/dma_heap", heaps, 1);

    /* 要求 phys 但钩子拿不到 -> PHYS。 */
    g_dmabuf_phys_ok = 0;
    rc = ps_pmem_alloc(&g_pmem_env, 0x20000ull, PS_PMEM_F_REQUIRE_PHYS, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_ERR_PHYS, "dma-heap 拿不到 phys 且要求 phys -> PHYS");
    CHECK(g_ioctl_calls == 1, "确实调过 DMA_HEAP_IOCTL_ALLOC");

    /* 不要求 phys：给出 phys_valid=0 的块。 */
    fk_reset();
    fk_dir("/dev/dma_heap", heaps, 1);
    rc = ps_pmem_alloc(&g_pmem_env, 0x20000ull, 0u, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_OK && b.kind == PS_PMEM_KIND_DMAHEAP && b.phys_valid == 0,
          "不要求 phys -> 成功且 phys_valid=0");
    ps_pmem_free(&g_pmem_env, &b);

    /* 钩子拿得到 phys。 */
    fk_reset();
    fk_dir("/dev/dma_heap", heaps, 1);
    g_dmabuf_phys_ok = 1;
    g_dmabuf_phys_val = 0x80000000ull;
    rc = ps_pmem_alloc(&g_pmem_env, 0x20000ull, PS_PMEM_F_REQUIRE_PHYS, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_OK && b.phys_valid == 1 && b.phys == 0x80000000ull,
          "钩子给出 phys -> 有效");
    ps_pmem_free(&g_pmem_env, &b);

    /* ioctl 失败。 */
    fk_reset();
    fk_dir("/dev/dma_heap", heaps, 1);
    g_ioctl_fail = 1;
    rc = ps_pmem_alloc(&g_pmem_env, 0x20000ull, 0u, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_ERR_IOCTL, "DMA_HEAP_IOCTL_ALLOC 失败 -> ERR_IOCTL");
}

static void test_pmem_no_backend_and_devmem(void)
{
    ps_pmem_block_t b;
    char err[256];
    ps_pmem_env_t env;
    int rc;
    uint32_t kinds = 0u;

    printf("\n---- B4. 无后端与 /dev/mem 回退 ----\n");
    fk_reset();
    rc = ps_pmem_alloc(&g_pmem_env, 0x1000ull, 0u, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_ERR_NO_BACKEND, "三条路都不可用 -> NO_BACKEND");
    rc = ps_pmem_probe(&g_pmem_env, &kinds, err, sizeof(err));
    CHECK(rc == PS_PMEM_OK && kinds == 0u, "probe 报告无后端");

    /* /dev/mem：默认不允许 -> NO_BACKEND。 */
    fk_reset();
    make_pmem_env();
    env = g_pmem_env;
    env.devmem_base = 0x0000000080000000ull;
    env.devmem_bytes = 0x00100000ull;
    rc = ps_pmem_alloc(&env, 0x10000ull, PS_PMEM_F_REQUIRE_PHYS, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_ERR_NO_BACKEND, "未开 ALLOW_DEVMEM -> NO_BACKEND");

    /* 显式允许且已预留 -> 映射成功，phys = 预留基址。 */
    rc = ps_pmem_alloc(&env, 0x10000ull,
                       PS_PMEM_F_REQUIRE_PHYS | PS_PMEM_F_ALLOW_DEVMEM,
                       &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_OK && b.kind == PS_PMEM_KIND_DEVMEM && b.phys == 0x80000000ull,
          "开 ALLOW_DEVMEM 且已预留 -> 映射成功");
    ps_pmem_free(&env, &b);

    /* 只有 system heap、且未允许 -> 不用于 PL（NO_BACKEND）。 */
    fk_reset();
    {
        static const char *sysonly[] = { "system" };
        fk_dir("/dev/dma_heap", sysonly, 1);
    }
    g_dmabuf_phys_ok = 1; g_dmabuf_phys_val = 0x90000000ull;
    rc = ps_pmem_alloc(&g_pmem_env, 0x1000ull, PS_PMEM_F_REQUIRE_PHYS, &b, err, sizeof(err));
    CHECK(rc == PS_PMEM_ERR_NO_BACKEND, "只有 system heap 且未允许 -> NO_BACKEND（不保证连续）");
}

static void test_pmem_parse(void)
{
    uint64_t v = 0ull;
    printf("\n---- B5. 容量解析 ----\n");
    CHECK(ps_pmem_parse_u64("0x00100000\n", &v) && v == 0x100000ull, "解析 0x100000");
    CHECK(ps_pmem_parse_u64("1048576", &v) && v == 1048576ull, "解析十进制 1048576");
    CHECK(!ps_pmem_parse_u64("x1", &v), "非法输入判失败");
}

/* ---------------------------------------------------------------- 主入口 */

int main(void)
{
    make_pmem_env();

    printf("==== UIO 入口与连续物理内存分配主机单测（注入式后端，无板卡）====\n");
    test_uio_helpers();
    test_uio_normal();
    test_uio_device_missing();
    test_uio_permission();
    test_uio_bad_map_size();
    test_uio_mmap_fail();
    test_pmem_udmabuf();
    test_pmem_udmabuf_limits();
    CHECK(sizeof(ps_dma_heap_alloc_t)==24 && offsetof(ps_dma_heap_alloc_t,fd)==8 &&
          offsetof(ps_dma_heap_alloc_t,heap_flags)==16,"Linux DMA_HEAP ABI offsets and size");
    {uint64_t value;
     CHECK(!ps_pmem_parse_u64("18446744073709551616",&value),"decimal overflow rejected");
     CHECK(!ps_pmem_parse_u64("0x10000000000000000",&value),"hex overflow rejected");}
    {uint64_t value;
     CHECK(!ps_uio_parse_u64("18446744073709551616",&value),"UIO numeric overflow rejected");
     CHECK(!ps_uio_parse_u64("12 junk",&value),"UIO trailing junk rejected");
     CHECK(!ps_pmem_parse_u64("12 junk",&value),"DDR trailing junk rejected");}
    test_pmem_dmaheap();
    test_pmem_no_backend_and_devmem();
    test_pmem_parse();

    printf("\n------------------------------------------------------------\n");
    printf("通过 %d ，失败 %d\n", g_pass, g_fail);
    if (g_fail != 0) {
        printf("结果：FAIL\n");
        return 1;
    }
    printf("结果：PASS（仅覆盖后端选择与错误分支；真 open/mmap/ioctl 与设备树前提等板卡）\n");
    return 0;
}
