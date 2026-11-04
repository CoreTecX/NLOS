// ============================================================================
// camera_service.h —— PS 侧取流服务（方案 §7 第 1 条）
//
// 方案原文：「camera_service：用 Spinnaker/GenTL 采集 RAW12，设置曝光、ROI、
//            帧号和时间戳」。
//
// 取流为什么要抽象成一张函数指针表
// ---------------------------------------------------------------------------
// 板端要用 Spinnaker/GenTL（厂商 SDK），主机上既没有相机也没有 SDK，但**上层
// 的驱动代码必须现在就能写、能跑、能自检**。于是把「怎么取一帧」收进
//
//     ps_camera_capture_t { open, close, pull, rewind, set_exposure_us, get_exposure_us }
//
// 四个实现：
//   file       裸帧文件（uint16 小端、帧主序、无头部）—— 与 ps_sources.hpp 同布局
//   synthetic  由 (seed, 像素下标) 的**位置无关哈希**给出，可从任意偏移续取
//   spinnaker  Spinnaker SDK（**留桩**：打开即 Unavailable）
//   gentl      GenTL/GenICam（**留桩**：同上）
// 上层的 pump 循环一行不用改，换实现只换一个枚举值。
//
// ★ 硬约束（方案 §4.1/§7，本文件用注释 + 编译期 + 运行期三重方式落实）★
// ---------------------------------------------------------------------------
//   **禁止相机 SDK 先做 8bit RGB 转换（解马赛克）**。相机只允许输出 RAW Bayer
//   （单通道马赛克），所有解交织（bayer_extract）都在 M1 里做。原因：
//     * 一旦 SDK 先转 RGB，进 M1 的就不是 RAW12 而是三通道 8bit，M1 的
//       「彩色先解交织再分块」口径立刻作废（墙网格边长算错一半）；
//     * 8bit 会把 12bit 的动态范围压掉 4 bit，而本项目正好靠低比特差分成像，
//       这一压就是不可逆的信息损失；
//     * 方案 §7 明确「CPU 不得调用 NumPy/OpenCV 完成在线重建」，把解马赛克
//       交给 SDK 等于把预处理搬到 CPU/SDK 侧，FPGA 核心性被削弱。
//
//   落实方式：
//     1. 编译期：定义 PS_CAMERA_ALLOW_SDK_RGB_CONVERSION 会直接编译失败（见 .c）；
//     2. 运行期：ps_camera_format_check() 拒绝任何「解交织/大于 2 字节每像素/
//        非法容器位深」的格式描述 —— 这种格式描述只可能来自「SDK 已经转换过」
//        或「配置写错」，两种都必须拦下；
//     3. 结构体里 confirmed_raw_bayer 只在格式检查通过后置 1，
//        上层可以据此判断「这一路流的 RAW 口径是真的被确认过」。
//
// 三条实现共用的 pull 返回语义（照 read(2) 的口径，避免两套码混用）
//   >= 0  实际取到的像素个数；**0 表示流已尽**
//   -1    数据源不可用（实物未到）
//   -2    出错
//   -3    格式描述违反 RAW Bayer 约束
//   -4    入参错误
// ============================================================================

#ifndef PENUMBRASIGHT_CAMERA_SERVICE_H
#define PENUMBRASIGHT_CAMERA_SERVICE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- 返回码

#define PS_CAM_OK          0
#define PS_CAM_UNAVAILABLE (-1)   // 实物未到（与 ps_sources.hpp 的 PS_PULL_UNAVAILABLE 同值）
#define PS_CAM_ERR_IO      (-2)   // 出错（与 ps_sources.hpp 的 PS_PULL_ERROR 同值）
#define PS_CAM_ERR_FORMAT  (-3)   // 违反 RAW Bayer 约束
#define PS_CAM_ERR_ARG     (-4)

// ---------------------------------------------------------------- 实现种类

#define PS_CAM_IMPL_FILE      0
#define PS_CAM_IMPL_SYNTHETIC 1
#define PS_CAM_IMPL_SPINNAKER 2   // 留桩
#define PS_CAM_IMPL_GENTL     3   // 留桩

const char* ps_camera_impl_name(int impl);
int ps_camera_impl_from_name(const char* name);   // 找不到返回 -1

// ---------------------------------------------------------------- 流规格

// **只描述 RAW Bayer**：单通道马赛克 + 16 bit 容器。
//   bytes_per_pixel 必须 2（RAW12 装在 16 bit 容器里，与 ps_contract.hpp 的
//     PS_PIX_BYTES 一致）；给 1 或 3 说明有人想塞 8bit / RGB8 进来。
//   interleaved_rgb 必须 0；给 1 说明这是已经解交织过的数据。
// 这两个字段存在的唯一目的：让「SDK 先转了 RGB」这件事在**格式描述这一层**
// 就无处可藏，而不是等到 M1 输出边长对不上时才发现。
typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t container_bits;    // 有效位深：8 / 12 / 16（RAW12 -> 12）
    uint32_t bayer_phase;       // 0 = RGGB（左上 R）；1 = BGGR（左上 B）
    uint32_t bytes_per_pixel;   // 必须 2
    uint32_t interleaved_rgb;   // 必须 0
    uint32_t crop;              // 中心裁剪边长；0 = 取 min(width,height)
    uint32_t block;             // 分块边长（方形）
    uint32_t frames;            // 帧累积数（1/4/8）
    uint64_t dark_level;        // 暗电平，先减后处理
} ps_camera_format_t;

// 一帧的像素数（按 RAW 幅面算，不是按 M1 输出算）
static inline uint64_t ps_camera_frame_pixels(const ps_camera_format_t* f) {
    if (!f) return 0u;
    return (uint64_t)f->width * (uint64_t)f->height;
}

// 裁剪边长：crop 为 0 时取 min(width,height)。与 ps_contract.hpp 的
// ps_crop_side 同口径（host_selftest 里有两者的一致性比对）。
static inline uint32_t ps_camera_crop_side(const ps_camera_format_t* f) {
    if (!f) return 0u;
    if (f->crop > 0u) return f->crop;
    return (f->width < f->height) ? f->width : f->height;
}

// 解交织后的平面边长：RAW Bayer 是彩色的，故每维减半。
// （彩色时先解交织再分块 —— 这条口径在 ps_contract.hpp 里被冻结。）
static inline uint32_t ps_camera_plane_side(const ps_camera_format_t* f) {
    return ps_camera_crop_side(f) / 2u;
}

// 墙网格边长 = 平面边长 / 分块边长。**必须等于几何配置的 wall.n**
// （X1 给 M1 定的那条新约束）。
static inline uint32_t ps_camera_wall_side(const ps_camera_format_t* f) {
    if (!f || f->block == 0u) return 0u;
    return ps_camera_plane_side(f) / f->block;
}

// ---------------------------------------------------------------- 格式检查

// RAW Bayer 约束的运行期检查（硬约束第 2 条）。
// 通过返回 PS_CAM_OK；不通过返回 PS_CAM_ERR_FORMAT 并把原因写进 reason。
int ps_camera_format_check(const ps_camera_format_t* f, char* reason, size_t cap);

// 几何一致性：推出的墙网格边长必须**恰好等于** wall_n。
// 与 ps_contract.hpp 的 ps_validate_spec 同判据（自检里有交叉比对）。
int ps_camera_geometry_check(const ps_camera_format_t* f, uint32_t wall_n,
                             char* reason, size_t cap);

// ---------------------------------------------------------------- 取流句柄

typedef struct {
    int      impl;
    void*    ctx;
    const char* name;

    int  (*open)(void* ctx, const ps_camera_format_t* fmt, const char* dev);
    void (*close)(void* ctx);
    long long (*pull)(void* ctx, uint16_t* dst, long long max_pixels);
    int  (*rewind)(void* ctx);
    int  (*set_exposure_us)(void* ctx, int64_t us);
    int64_t (*get_exposure_us)(void* ctx);

    ps_camera_format_t fmt;         // 打开时锁定的规格（之后不允许改）
    uint64_t seed;                  // 只有 synthetic 用（与 ps_sources.hpp 的 seed 同义）
    uint64_t total_pixels;          // 流总像素数（file = 文件长度推断；synthetic = 传入）
    unsigned frames_served;         // 已成功取出的帧数
    unsigned dropped_frames;        // 取不满/半帧的次数（帧号跳变检测用）
    int      confirmed_raw_bayer;   // 1 = 格式检查通过，RAW 口径已确认
} ps_camera_capture_t;

// 打开取流。dev 语义随实现变化：
//   file       -> dev 是裸帧文件路径（纯 ASCII！见 model_loader.c 的说明）
//   synthetic  -> dev 忽略；total_pixels 由 fmt.frames * 每帧像素数给出
//   spinnaker/gentl -> dev 是相机序列号（留桩：返回 Unavailable）
// 返回 PS_CAM_OK / PS_CAM_ERR_FORMAT / PS_CAM_ERR_IO / PS_CAM_UNAVAILABLE。
int ps_camera_open(ps_camera_capture_t* c, int impl, const ps_camera_format_t* fmt,
                   const char* dev);

void ps_camera_close(ps_camera_capture_t* c);

// 取 n_frames 帧（每帧 ps_camera_frame_pixels 个像素）灌满 dst。
// 返回灌入的像素数；负数为错误码。与 ps_contract.hpp 的 ps_pump_frames 同一件事，
// 只是这边是 C 口径且带帧计数与丢帧统计。
long long ps_camera_pump(ps_camera_capture_t* c, unsigned n_frames, uint16_t* dst);

// 重置流位置（等价于板上「重置采集队列」）。
int ps_camera_rewind(ps_camera_capture_t* c);

// ---------------------------------------------------------------- 工具

// 把像素写成裸帧文件（测试/造向量用）。布局 = 无头、uint16 小端、帧主序，
// 与 ps_sources.hpp 的 ps_write_raw_frames 逐字节一致。
int ps_camera_write_raw_frames(const char* path, const uint16_t* data,
                               uint64_t n_pixels);

// synthetic 源的纯函数取像素：位置无关（可从任意下标独立取值）。
// **必须与 ps_sources.hpp 的 ps_sim_pixel 逐值相同**（自检里有交叉比对），
// 否则「主机用 synthetic、板上用真相机」两条路的量纲会分叉。
uint16_t ps_camera_synthetic_pixel(const ps_camera_format_t* f, uint64_t seed,
                                   uint64_t k);

// 从第 first_index 个像素开始生成 n_pixels 个。
// first_index 必须显式传入而**不是**默认从 0 起：分段 pull 时第二段若不带上
// 绝对下标，第二帧就会从 0 重新开始（画面会「重播第一帧」，而且不报错）。
// 离线造向量传 0；pull 路径传当前读游标。
void ps_camera_synthetic_fill(const ps_camera_format_t* f, uint64_t seed,
                              uint64_t first_index, uint16_t* dst,
                              uint64_t n_pixels);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // PENUMBRASIGHT_CAMERA_SERVICE_H
