/* ============================================================================
 * PenumbraSight · 板端 UIO 访问后端（可移植逻辑）
 * src/software/fpga_control/uio_backend.c
 *
 * 见 uio_backend.h 的用法说明。本文件**不包含任何平台头文件**：它只把
 * ps_uio_env_t 里的 open/read/list_dir/map/unmap 拼成标准 UIO 的打开流程，
 * 因此主机与板端跑的是同一份逻辑。
 *
 * 打开流程（每一步都有明确失败码，不静默降级）
 * ---------------------------------------------------------------------------
 *   1) 解析设备：spec 是 "/dev/uioN" 就直接用；否则按 name 令牌扫
 *      /sys/class/uio/uioN/name -> NO_DEVICE；
 *   2) open(dev, O_RDWR|O_SYNC)：失败按 errno 区分 PERM(权限) 与 NO_DEVICE/IO；
 *   3) 读 /sys/class/uio/uioN/maps/map0/size 与 .../map0/addr：读不到 -> MAP_SIZE；
 *   4) 校验 [addr, addr+size) 覆盖 [need_base, need_base+need_bytes)：不覆盖 -> MAP_SIZE；
 *   5) 长度按页向上取整后 mmap(map0, offset 0)：失败 -> MAP；
 *   6) 交回 map/map_phys/map_bytes/fd，关闭由 ps_uio_close_map 负责。
 * ==========================================================================*/

#include "uio_backend.h"

#include <stdio.h>
#include <string.h>
#include <limits.h>

/* 平台原语表：真板用 POSIX 的；主机没有可用的就返回 NULL（调用方给 UNAVAIL）。 */
const ps_uio_env_t *ps_uio_env_default(void)
{
#if defined(__linux__)
    return ps_uio_env_posix();
#else
    return 0;
#endif
}

/* ---------------------------------------------------------------- 小工具 */

int ps_uio_parse_u64(const char *s, uint64_t *out)
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
        else { return 0; }              /* 遇到非数字：整体判失败，不截断 */
        if ((uint64_t)d >= base) { return 0; }
        if (v > (UINT64_MAX - (uint64_t)d) / base) return 0;
        v = v * base + (uint64_t)d;
        any = 1;
    }
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') ++s;
    if (!any || *s) { return 0; }
    if (out) { *out = v; }
    return 1;
}

uint64_t ps_uio_round_up(uint64_t v, uint64_t a)
{
    if (a == 0ull) { return v; }
    return ((v + a - 1ull) / a) * a;
}

/* ---------------------------------------------------------------- 读小文件 */

/* 读取 sysfs 里的一行文本（去掉尾部空白）。返回 PS_UIO_OK / 负码。 */
static int uio_read_text(const ps_uio_env_t *env, const char *path,
                         char *buf, size_t cap)
{
    int fd;
    long n;
    size_t i;

    if (!env || !env->open || !env->read || !env->close || !path || !buf || cap == 0u) {
        return PS_UIO_ERR_ARGS;
    }
    fd = env->open(path, PS_UIO_O_RDONLY);
    if (fd < 0) { return PS_UIO_ERR_IO; }
    n = env->read(fd, buf, (unsigned long)(cap - 1u));
    (void)env->close(fd);
    if (n < 0) { return PS_UIO_ERR_IO; }
    buf[(size_t)n < cap ? (size_t)n : cap - 1u] = '\0';
    for (i = 0; i < (size_t)n && i < cap; ++i) {
        if (buf[i] == '\n' || buf[i] == '\r') { buf[i] = '\0'; break; }
    }
    return PS_UIO_OK;
}

/* ---------------------------------------------------------------- 扫描设备 */

int ps_uio_find_by_name(const ps_uio_env_t *env, const char *token,
                        char *out_dev, size_t out_cap,
                        char *out_name, size_t name_cap)
{
    char names[PS_UIO_MAX_NAMES][PS_UIO_NAME_CAP];
    int n, i;

    if (!env || !env->list_dir || !token || !out_dev || out_cap == 0u) {
        return PS_UIO_ERR_ARGS;
    }
    n = env->list_dir("/sys/class/uio", names, PS_UIO_MAX_NAMES);
    if (n <= 0) { return PS_UIO_ERR_NO_DEVICE; }
    if (n > PS_UIO_MAX_NAMES) { n = PS_UIO_MAX_NAMES; }

    for (i = 0; i < n; ++i) {
        char path[160];
        char text[128];
        /* 条目名形如 "uio0"；只认这个前缀，别的（如 "uio" 之外的软链）跳过。 */
        if (strncmp(names[i], "uio", 3) != 0) { continue; }
        snprintf(path, sizeof(path), "/sys/class/uio/%.*s/name", PS_UIO_NAME_CAP - 1, names[i]);
        text[0] = '\0';
        if (uio_read_text(env, path, text, sizeof(text)) != PS_UIO_OK) { continue; }
        if (strstr(text, token) != NULL) {
            snprintf(out_dev, out_cap, "/dev/%.*s", PS_UIO_NAME_CAP - 1, names[i]);
            if (out_name && name_cap > 0u) { snprintf(out_name, name_cap, "%.*s", PS_UIO_NAME_CAP - 1, names[i]); }
            return PS_UIO_OK;
        }
    }
    return PS_UIO_ERR_NO_DEVICE;
}

/* ---------------------------------------------------------------- 打开映射 */

int ps_uio_open_map(ps_uio_map_t *m, const ps_uio_env_t *env, const char *spec,
                    uint32_t need_base, uint32_t need_bytes,
                    char *err, size_t errcap)
{
    char dev[128];
    char name[PS_UIO_NAME_CAP];
    char path[192];
    char text[64];
    uint64_t size_v = 0ull, addr_v = 0ull;
    int fd, rc;
    unsigned long len;

    if (err && errcap > 0u) { err[0] = '\0'; }
    if (!m || !env) { return (env ? PS_UIO_ERR_ARGS : PS_UIO_ERR_UNAVAIL); }
    if (!env->open || !env->close || !env->read || !env->map || !env->unmap ||
        env->page_size == 0u) {
        return PS_UIO_ERR_ARGS;
    }
    memset(m, 0, sizeof(*m));
    m->fd = -1;
    if (need_bytes == 0u) { need_bytes = PS_UIO_REQ_BYTES; }

    /* 1) 解析设备名。 */
    dev[0] = '\0';
    name[0] = '\0';
    if (spec && spec[0] == '/') {
        snprintf(dev, sizeof(dev), "%s", spec);
        {
            const char *slash = strrchr(dev, '/');
            if (slash && slash[1] != '\0') { snprintf(name, sizeof(name), "%s", slash + 1); }
        }
    } else {
        const char *token =
            (spec && spec[0] != '\0' && strcmp(spec, "auto") != 0)
                ? spec : PS_UIO_DEFAULT_TOKEN;
        rc = ps_uio_find_by_name(env, token, dev, sizeof(dev), name, sizeof(name));
        if (rc != PS_UIO_OK) {
            if (err && errcap > 0u) {
                snprintf(err, errcap,
                         "扫描 /sys/class/uio 找不到 name 含 \"%s\" 的设备（%d）",
                         token, rc);
            }
            return rc;
        }
    }
    if (name[0] == '\0') {
        if (err && errcap > 0u) { snprintf(err, errcap, "设备路径无 uioN 名：%s", dev); }
        return PS_UIO_ERR_NO_DEVICE;
    }

    /* 2) 打开设备。 */
    fd = env->open(dev, PS_UIO_O_RDWR);
    if (fd < 0) {
        rc = (-fd == 13) ? PS_UIO_ERR_PERM : PS_UIO_ERR_NO_DEVICE;
        if (err && errcap > 0u) {
            snprintf(err, errcap, "open(%s) 失败：%s（errno=%d）", dev,
                     (rc == PS_UIO_ERR_PERM) ? "权限不足/被占用" : "设备不存在或不可用",
                     -fd);
        }
        return rc;
    }

    /* 3) 读 map0 的 size 与 addr。 */
    snprintf(path, sizeof(path), "/sys/class/uio/%s/maps/map0/size", name);
    if (uio_read_text(env, path, text, sizeof(text)) != PS_UIO_OK ||
        !ps_uio_parse_u64(text, &size_v) || size_v == 0ull) {
        if (err && errcap > 0u) {
            snprintf(err, errcap, "读 %s 失败或为 0（map0 未在设备树里定义？）", path);
        }
        (void)env->close(fd);
        return PS_UIO_ERR_MAP_SIZE;
    }
    snprintf(path, sizeof(path), "/sys/class/uio/%s/maps/map0/addr", name);
    if (uio_read_text(env, path, text, sizeof(text)) != PS_UIO_OK ||
        !ps_uio_parse_u64(text, &addr_v)) {
        if (err && errcap > 0u) { snprintf(err, errcap, "读 %s 失败", path); }
        (void)env->close(fd);
        return PS_UIO_ERR_MAP_SIZE;
    }

    /* 4) 覆盖校验：窗口必须包含 [need_base, need_base+need_bytes)。否则核的偏移
     * 会算到映射之外，表现为「写到别人的地址上」，不报错。 */
    if (addr_v>UINT32_MAX || size_v>UINT32_MAX || addr_v%env->page_size ||
        size_v>UINT32_MAX-env->page_size || !(addr_v <= (uint64_t)need_base &&
          (uint64_t)need_base + (uint64_t)need_bytes <= addr_v + size_v)) {
        if (err && errcap > 0u) {
            snprintf(err, errcap,
                     "map0 覆盖不了所需窗口：map0=[0x%llX,+0x%llX) 需要 [0x%lX,+0x%lX)",
                     (unsigned long long)addr_v, (unsigned long long)size_v,
                     (unsigned long)need_base, (unsigned long)need_bytes);
        }
        (void)env->close(fd);
        return PS_UIO_ERR_MAP_SIZE;
    }

    /* 5) 长度按页对齐后 mmap（map0 的 offset 恒为 0）。 */
    len = (unsigned long)ps_uio_round_up(size_v, (uint64_t)env->page_size);
    m->map = env->map(NULL, len, PS_UIO_PROT_RD | PS_UIO_PROT_WR, PS_UIO_MAP_SHARED,
                      fd, 0L);
    if (!m->map) {
        if (err && errcap > 0u) {
            snprintf(err, errcap, "mmap(%s, %lu B) 失败", dev, len);
        }
        (void)env->close(fd);
        return PS_UIO_ERR_MAP;
    }

    m->map_phys  = addr_v;
    m->map_bytes = (uint64_t)len;
    m->fd        = fd;
    snprintf(m->dev_path, sizeof(m->dev_path), "%s", dev);
    snprintf(m->name, sizeof(m->name), "%s", name);
    return PS_UIO_OK;
}

void ps_uio_close_map(const ps_uio_env_t *env, ps_uio_map_t *m)
{
    if (!m) { return; }
    if (env && m->map && m->map_bytes > 0ull && env->unmap) {
        (void)env->unmap(m->map, (unsigned long)m->map_bytes);
    }
    if (env && m->fd >= 0 && env->close) { (void)env->close(m->fd); }
    memset(m, 0, sizeof(*m));
    m->fd = -1;
}
