/* ============================================================================
 * PenumbraSight · 板端连续物理内存分配（可注入）
 * src/software/board_app/board_pmem.h
 *
 * board_top 的账本（ps_board_arena_t）只做**地址规划**，不真分配；PL 的 m_axi
 * 要的是**物理连续的 DDR**，主机 malloc 给不了。这一层补上真分配，并把三条路
 * 收在同一张函数指针表后面（与 camera_service 同一做法，主机可注入假后端）。
 *
 * 三条路（探测顺序 = udmabuf -> dma-heap -> /dev/mem 回退）
 * ---------------------------------------------------------------------------
 *   1) udmabuf（推荐，phys 可直接读）
 *      设备 /dev/udmabuf0，sysfs 暴露物理地址。dts 里由 ikwzm,u-dma-buf 节点
 *      绑定一段 reserved-memory/CMA，启动时分配。物理地址读自
 *        /sys/class/u-dma-buf/<name>/phys_addr
 *      （旧内核路径为 /sys/class/udmabuf/<name>/phys_addr，本层两者都试）。
 *      优点：phys_addr 是内核给的，userspace 直接读，不猜不推导。
 *      缺点：大小在设备树里固定；一块 udmabuf 一次只服务一个用途。
 *   2) dma-heap（次选）
 *      /dev/dma_heap/ 下的 heap，open 后 ioctl(DMA_HEAP_IOCTL_ALLOC) 拿 dmabuf fd，再
 *      mmap fd。名字含 cma/reserved 的 heap 才保证物理连续；/dev/dma_heap/system
 *      **不保证连续**，只有显式允许（PS_PMEM_F_ALLOW_SYSTEM_HEAP）才用。
 *      缺点：mainline 没有给出「读 dmabuf 物理地址」的标准 ioctl；本层的
 *      dmabuf_phys 钩子在 POSIX 侧是**尽力而为**（读 /proc/self/pagemap，需
 *      权限且对 dma-buf 映射常常拿不到），拿不到就如实置 phys_valid=0；要求
 *      phys 的调用方会拿到 PS_PMEM_ERR_PHYS。若某厂商 heap 在 sysfs 里给出
 *      phys，则把 dmabuf_phys 钩子指向它即可。
 *   3) CMA（第三条路，**是供给方式，不是独立 ioctl**）
 *      在 dts 里留一块 reserved-memory（`linux,cma` 或专用节点），再把它交给
 *      (a) udmabuf 的 memory-region，或 (b) 一个 UIO 节点的 mapN。取舍：
 *        · 好处：一次预留、整段物理连续、不随内存压力迁移；
 *        · 代价：预留量在启动时固定，占死内存；预留过大挤压系统，过小在需要
 *          300 MB 模型时分配失败；多进程共享同一段需要自己定分配策略。
 *      所以本层不单独实现 CMA ioctl；走 udmabuf(+memory-region) 即可，探测时
 *      用 /proc/meminfo 的 CmaTotal 报告「有 CMA 池」。
 *   4) /dev/mem（最后手段，默认关闭）
 *      只映射一段**已经预留**的物理区间，不做分配。风险与场景见下。
 *
 * /dev/mem 回退该不该留
 * ---------------------------------------------------------------------------
 *   保留，但必须显式打开（PS_PMEM_F_ALLOW_DEVMEM）且 env 里给出预留区间
 *   （devmem_base/devmem_bytes）。理由与代价：
 *     · 风险：mmap /dev/mem 会绕过内核的内存管理。现代内核默认
 *       CONFIG_STRICT_DEVMEM=y，只允许映射 IO 区（如 0xA0000000）而拒绝系统
 *       RAM；要映射 DDR 段得关掉它或加 iomem=relaxed，等于放宽了整机的一条
 *       安全边界；地址写错会踩到内核自己的内存。
 *     · 适用场景：设备树/内核不含 udmabuf 也不含 dma-heap 的出厂镜像，且
 *       已经有 `mem=`/reserved-memory 把一段 DDR 从内核手里摘出来的应急场合；
 *       或者只映射 0xA... 的寄存器区（那本来就该走 UIO，不必动 /dev/mem）。
 *     · 结论：寄存器访问走 UIO；DDR 缓冲优先 udmabuf；/dev/mem 只在前面都不
 *       可用且已预留区间时用，并在日志里显式提示「已放宽 STRICT_DEVMEM」。
 *
 * 未验证声明
 * ---------------------------------------------------------------------------
 * ps_pmem_env_posix() 的系统调用（open/ioctl/mmap）**未在板上跑过**。主机测试
 * 覆盖的是选择与错误分支（用假 env）。真机上「udmabuf 的 phys_addr 能读、
 * dma-heap 的 phys 拿不到」等结论属板端实测，见
 * sim/results/uio_entry_20261001.md 的板上待验清单。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_BOARD_PMEM_H
#define PENUMBRASIGHT_BOARD_PMEM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 返回码 */
#define PS_PMEM_OK             0
#define PS_PMEM_ERR_ARGS     (-1)
#define PS_PMEM_ERR_NO_BACKEND (-2)  /* 三条路都没有可用后端 */
#define PS_PMEM_ERR_SIZE     (-3)    /* 请求超过后端可用容量 */
#define PS_PMEM_ERR_OPEN     (-4)
#define PS_PMEM_ERR_IOCTL    (-5)
#define PS_PMEM_ERR_MMAP     (-6)
#define PS_PMEM_ERR_PHYS     (-7)    /* 物理地址获取失败（调用方要求 phys 时） */

/* ---------------------------------------------------------------- 后端种类 */
#define PS_PMEM_KIND_NONE    0
#define PS_PMEM_KIND_UDMABUF 1
#define PS_PMEM_KIND_DMAHEAP 2
#define PS_PMEM_KIND_CMA     3   /* 只用于探测报告：有 CMA 池可给 udmabuf/UIO 用 */
#define PS_PMEM_KIND_DEVMEM  4

/* ---------------------------------------------------------------- 分配标志 */
#define PS_PMEM_F_REQUIRE_UDMABUF 0x8u /* persistent reserved DDR, no heap fallback */
#define PS_PMEM_F_REQUIRE_PHYS     0x1u  /* 拿不到 phys 视为失败 */
#define PS_PMEM_F_ALLOW_DEVMEM     0x2u  /* 允许 /dev/mem 回退 */
#define PS_PMEM_F_ALLOW_SYSTEM_HEAP 0x4u /* 允许 /dev/dma_heap/system（不保证连续） */

/* ---------------------------------------------------------------- 平台原语表 */

/* mainline struct dma_heap_allocation_data 的布局（uapi/linux/dma-heap.h）：
 *   __u64 len; __u32 fd; __u32 fd_flags; __u64 heap_flags;
 * 本层自带一份以免主机构建依赖 <linux/dma-heap.h>。 */
typedef struct {
    uint64_t len;
    uint32_t fd;
    uint32_t fd_flags;
    uint64_t heap_flags;
} ps_dma_heap_alloc_t;

#define PS_PMEM_MAX_NAMES 8
#define PS_PMEM_NAME_CAP  64

/* 列目录：把 dir 下条目名写进 names（最多 max 个），返回条目数或负值。 */
typedef int (*ps_pmem_list_dir_fn)(const char *dir,
                                   char names[][PS_PMEM_NAME_CAP], int max);

typedef struct ps_pmem_env {
    int   (*exists)(const char *path);
    int   (*open)(const char *path, int flags);   /* 失败返回 -errno */
    int   (*close)(int fd);
    long  (*read)(int fd, void *buf, unsigned long n);
    int   (*ioctl)(int fd, unsigned long req, void *arg);
    void *(*map)(void *addr, unsigned long len, int prot, int flags, int fd, long off);
    int   (*unmap)(void *addr, unsigned long len);
    ps_pmem_list_dir_fn list_dir;
    /* 尽力而为地取 dmabuf 的物理地址：成功返回 0 写 *phys，否则返回负值。
     * 可为 NULL（表示该平台不提供）。 */
    int   (*dmabuf_phys)(int fd, void *virt, uint64_t len, uint64_t *phys);
    unsigned long page_size;
    unsigned long dma_heap_alloc_ioctl;   /* DMA_HEAP_IOCTL_ALLOC 的 _IOWR 值 */
    /* /dev/mem 回退用的预留区间（都为 0 表示不可用）。 */
    uint64_t devmem_base;
    uint64_t devmem_bytes;
} ps_pmem_env_t;

/* Linux 后端；非 Linux 构建下不参与编译，返回 NULL。 */
const ps_pmem_env_t *ps_pmem_env_posix(void);
const ps_pmem_env_t *ps_pmem_env_default(void);

/* ---------------------------------------------------------------- 分配块 */
typedef struct {
    int      kind;        /* PS_PMEM_KIND_* */
    int      fd;          /* 设备/ dmabuf fd，free 时关闭 */
    void    *virt;        /* mmap 得到的可读写首址 */
    uint64_t bytes;       /* 映射长度（页对齐） */
    uint64_t phys;        /* 物理基址；phys_valid=0 时无意义 */
    int      phys_valid;  /* 1 = phys 可信 */
    char     dev_path[128];
} ps_pmem_block_t;

/* ---------------------------------------------------------------- API */

/* 探测本机有哪些后端。kinds 是位掩码（1<<PS_PMEM_KIND_*）。reason 可读原因。 */
int  ps_pmem_probe(const ps_pmem_env_t *env, uint32_t *kinds,
                   char *reason, size_t reason_cap);

/* 分配一块物理连续内存。flags 见 PS_PMEM_F_*。成功填 out；失败写 reason。 */
int  ps_pmem_alloc(const ps_pmem_env_t *env, uint64_t bytes, uint32_t flags,
                   ps_pmem_block_t *out, char *reason, size_t reason_cap);

void ps_pmem_free(const ps_pmem_env_t *env, ps_pmem_block_t *b);

/* 解析 sysfs 里的十进制/十六进制 64 位量（容忍前缀 0x、空白、换行）。 */
int  ps_pmem_parse_u64(const char *s, uint64_t *out);

/* 找一个可用的 udmabuf：dev 形如 "/dev/udmabuf0"，sysdir 形如
 * "/sys/class/u-dma-buf/udmabuf0"（旧内核为 /sys/class/udmabuf/udmabuf0）。
 * 返回 PS_PMEM_OK / PS_PMEM_ERR_NO_BACKEND。 */
int  ps_pmem_find_udmabuf(const ps_pmem_env_t *env,
                          char *dev, size_t dev_cap,
                          char *sysdir, size_t dir_cap);

/* 找 /dev/dma_heap/ 下可用的 heap。want_contig=1 时只接受名字含 cma/reserved
 * 的（物理连续有保证）；否则也接受 system。返回布尔（1 找到）。 */
int  ps_pmem_find_dma_heap(const ps_pmem_env_t *env, int want_contig,
                           char *path, size_t cap);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* PENUMBRASIGHT_BOARD_PMEM_H */
