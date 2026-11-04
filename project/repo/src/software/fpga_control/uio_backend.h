/* ============================================================================
 * PenumbraSight · 板端 UIO 访问后端（可注入）
 * src/software/fpga_control/uio_backend.h
 *
 * 这一层只做一件事：把「打开 /dev/uioN、按 /sys/class/uio/.../maps/map0 的长度
 * 做 mmap、把映射基址与物理基址交出去」这套**标准 UIO 用法**写成可测试的代码。
 *
 * 为什么要把系统调用抽成函数指针表（与 camera_service 同一做法）
 * ---------------------------------------------------------------------------
 * 板端要 open/mmap，主机（本仓库的 Windows 构建）没有 <sys/mman.h>，也没有
 * 任何 UIO 设备。若把 open/mmap 直接写进逻辑里，主机侧就一行都测不到。于是把
 * 「平台原语」收进 ps_uio_env_t：
 *
 *       逻辑（本头文件声明、uio_backend.c 实现）
 *           ↑ 只调 env 里的函数
 *       ps_uio_env_t
 *           ├─ ps_uio_env_posix()   真板 Linux：open/read/opendir/mmap/munmap
 *           └─ 测试用的假 env        内存文件表，模拟缺失/权限/长度不对/映射失败
 *
 * 于是「设备不存在」「权限不足」「map0 长度不够」「mmap 失败」这些分支在主机上
 * 就能逐条跑；真板上跑的是同一份逻辑，只换 env。
 *
 * ----------------------------------------------------------------------------
 * 标准 UIO 用法（本文件的依据）
 * ----------------------------------------------------------------------------
 *   · 打开设备：open("/dev/uioN", O_RDWR | O_SYNC)
 *   · 映射长度：读 /sys/class/uio/uioN/maps/map0/size，mmap 的 length 取它
 *     （内核要求 length 按页对齐，故向上取整到 getpagesize()）；
 *   · 映射调用：mmap(NULL, length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0)
 *     —— map0 用 offset 0；mapN 用 offset = N * page_size；
 *   · 物理基址：读 /sys/class/uio/uioN/maps/map0/addr（用于校验映射窗口是否
 *     覆盖四个定点核的八段）。
 *   出处：Linux 内核 uio-howto 与 tcmu-design 文档（mmap length 必须等于
 *   map0/size；mapX/addr 是「可映射内存的起址」，mapX/size 是它的字节数）。
 *
 * ----------------------------------------------------------------------------
 * 中断：本层**不**用阻塞 read()
 * ----------------------------------------------------------------------------
 * UIO 的中断语义是「读 4 字节唤醒并使能下一次中断」（read(fd,&count,4)），它要
 * 求 PL 有中断线接进 PS 的 GIC、设备树里有 interrupts = <...>。四个定点核的
 * 完成标志是 CTRL.ap_done 位，board/fix_core.c 的 ps_fix_start_and_wait() 已经
 * 按轮询实现（poll_limit 次，不依赖 OS 计时器）。本后端因此**只做寄存器窗口**，
 * 不碰 read()，也不假设有中断线。将来若要走中断驱动，先补设备树中断，再把
 * ps_fix_start_and_wait 换成 read(fd,&n,4)，那是一次独立改动。
 *
 * ----------------------------------------------------------------------------
 * 未验证声明
 * ----------------------------------------------------------------------------
 * ps_uio_env_posix() 的系统调用（open/opendir/mmap）**未在板上跑过**：本仓库
 * 无 aarch64 板卡在环。主机测试覆盖的是 ps_uio_open_map() 的逻辑分支，用的是
 * 假 env；「真设备树里 map0 的 addr/size 到底是不是 0xA0000000 / 0x80000」
 * 属板端实测，见 sim/results/uio_entry_20261001.md 的板上待验清单。
 * ==========================================================================*/

#ifndef PENUMBRASIGHT_UIO_BACKEND_H
#define PENUMBRASIGHT_UIO_BACKEND_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 返回码 */
/* 这些值与 fix_core_driver.h 的 PS_FIX_DRV_ERR_* 一一对应（同数字）：
 *   PS_UIO_ERR_ARGS      == PS_FIX_ERR_ARG / PS_FIX_DRV_ERR_ARGS
 *   PS_UIO_ERR_UNAVAIL   == PS_FIX_DRV_ERR_UNAVAILABLE
 *   ...                  == PS_FIX_DRV_ERR_NO_DEVICE / _PERM / _MAP / _MAP_SIZE / _IO
 * 这样 ps_fix_window_open_uio 可以直接把本层返回码透传出去，不必再转一道。 */
#define PS_UIO_OK              0
#define PS_UIO_ERR_ARGS       (-2)   /* 入参为空等 */
#define PS_UIO_ERR_UNAVAIL    (-3)   /* 本平台无 POSIX 后端（如主机 Windows） */
#define PS_UIO_ERR_NO_DEVICE  (-6)   /* 扫描不到匹配的 UIO 设备 */
#define PS_UIO_ERR_PERM       (-7)   /* open 被拒（EACCES：权限/被占用） */
#define PS_UIO_ERR_MAP        (-8)   /* mmap 失败 */
#define PS_UIO_ERR_MAP_SIZE   (-9)   /* map0 的 addr/size 读不到，或覆盖不了所需窗口 */
#define PS_UIO_ERR_IO        (-10)   /* read/close 等 I/O 失败 */

/* open 的标志位（与 Linux 取值一致，POSIX 后端直接透传；假 env 只用到读写位）。 */
#define PS_UIO_O_RDONLY   0
#define PS_UIO_O_RDWR     2
/* mmap 的 prot/flags（同样与 Linux 取值一致）。 */
#define PS_UIO_PROT_RD    1
#define PS_UIO_PROT_WR    2
#define PS_UIO_MAP_SHARED 1

/* 需要的映射窗口：四个核八段共 512 KiB（ps_fix_regs.h 的 0xA0000000 起八段）。
 * 若设备树把 0xA00A0000（PL 控制页 VERSION）也划进同一条 reg，窗口会更大，
 * 那里读版本就能命中；只划 512 KiB 时版本读回为 UNMAPPED（board_top 已处理）。 */
#define PS_UIO_REQ_BASE   0xA0000000u
#define PS_UIO_REQ_BYTES  0x00080000u

/* 按名字扫描 /sys/class/uio 时的默认令牌。设备树节点名不同就在打开时把它作为
 * 参数传进来（见 ps_fix_window_open_uio 的 uio_dev 语义）。 */
#define PS_UIO_DEFAULT_TOKEN "penumbrasight"

#define PS_UIO_MAX_NAMES 16
#define PS_UIO_NAME_CAP  64

/* ---------------------------------------------------------------- 平台原语表 */

/* 列目录：把 dir 下的条目名（不含 "." / ".."）写进 names，最多 max 个。
 * 返回条目数；失败返回负值（-errno 约定）。 */
typedef int (*ps_uio_list_dir_fn)(const char *dir,
                                  char names[][PS_UIO_NAME_CAP], int max);

typedef struct {
    /* 打开路径；成功返回 fd >= 0，失败返回 -errno。
     * 权限不足请返回 -EACCES(=-13)，本层据此给出 PS_UIO_ERR_PERM。 */
    int   (*open)(const char *path, int flags);
    int   (*close)(int fd);
    /* 读文件；返回读到的字节数，失败返回 -errno。 */
    long  (*read)(int fd, void *buf, unsigned long n);
    ps_uio_list_dir_fn list_dir;
    /* mmap/munmap；返回映射首址，失败返回 NULL。off 为字节偏移。 */
    void *(*map)(void *addr, unsigned long len, int prot, int flags,
                 int fd, long off);
    int   (*unmap)(void *addr, unsigned long len);
    unsigned long page_size;
} ps_uio_env_t;

/* 真板（Linux）的后端。非 Linux 构建下该文件不参与编译，本函数返回 NULL。 */
const ps_uio_env_t *ps_uio_env_posix(void);

/* 本平台的默认后端：Linux 返回 ps_uio_env_posix()，其余平台返回 NULL。 */
const ps_uio_env_t *ps_uio_env_default(void);

/* ---------------------------------------------------------------- 映射句柄 */

typedef struct {
    void    *map;          /* mmap 首址（可读写） */
    uint64_t map_phys;     /* map0 的物理基址（来自 maps/map0/addr） */
    uint64_t map_bytes;    /* 实际映射长度（已按页对齐） */
    char     dev_path[128];/* 实际打开的 /dev/uioN */
    char     name[PS_UIO_NAME_CAP]; /* sysfs 里的 uioN */
    int      fd;
} ps_uio_map_t;

/* 打开映射窗口。
 *   spec        : "/dev/uioN" 直接用它；"auto"/NULL/"" 用默认令牌扫描；
 *                 其它字符串当作 sysfs 里的 name 令牌扫描。
 *   need_base   : 期望窗口覆盖的物理基址（四个核填 PS_UIO_REQ_BASE）。
 *   need_bytes  : 期望覆盖的字节数（四个核填 PS_UIO_REQ_BYTES）。
 * 成功返回 PS_UIO_OK 并填满 m；失败返回 PS_UIO_ERR_* 并写 err（可为 NULL/0）。 */
int  ps_uio_open_map(ps_uio_map_t *m, const ps_uio_env_t *env, const char *spec,
                     uint32_t need_base, uint32_t need_bytes,
                     char *err, size_t errcap);

void ps_uio_close_map(const ps_uio_env_t *env, ps_uio_map_t *m);

/* 按 name 令牌扫描 /sys/class/uio/uioN/name，命中则把 "/dev/uioN" 写进 out。
 * 返回 PS_UIO_OK / PS_UIO_ERR_NO_DEVICE / PS_UIO_ERR_IO。 */
int  ps_uio_find_by_name(const ps_uio_env_t *env, const char *token,
                         char *out_dev, size_t out_cap,
                         char *out_name, size_t name_cap);

/* ---------------------------------------------------------------- 小工具（可单测） */

/* 解析 sysfs 里以 "0x..." 或十进制写出的 64 位整数，容忍首尾空白与结尾换行。
 * 空串/越界返回 0。 */
int ps_uio_parse_u64(const char *s, uint64_t *out);

/* 把 v 向上取整到 a 的整数倍（a 为 0 时原样返回）。 */
uint64_t ps_uio_round_up(uint64_t v, uint64_t a);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* PENUMBRASIGHT_UIO_BACKEND_H */
