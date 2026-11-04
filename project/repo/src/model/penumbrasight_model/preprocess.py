# -*- coding: utf-8 -*-
"""预处理链浮点参考实现（T0-2）。

数据链路（与设计文档第 6.1 节的顶层数据流一致）：

    RAW Bayer 方图
      -> 中心裁剪 (crop)
      -> Bayer 解交织、双绿平均   -> plane = crop/2 的三通道平面
      -> block x block 分块累加求平均 -> out = plane/block 的测量图
      -> 帧累积（直接求和，不除以帧数）
      -> 垂直差分                 -> (out-1, out) 的差分测量

已由官方数据验证的三条规则（见 data/inventory/data_contract.md 与
sim/results/preprocess_verification.md）：

  R1. Bayer 相位为 RGGB，即 (0,0)=R、(0,1)=G1、(1,0)=G2、(1,1)=B。
      验证方式：用该相位解交织得到的各通道均值与官方 meas_r/g/b 的均值一致。
  R2. 绿通道为 G1 与 G2 的算术平均。
  R3. 累积为「求和」而非「平均」。
      验证方式：Mush/ 的 35 个单帧求和与 image_test_mushroom.mat 的 image
      逐元素完全相等（max|diff| = 0）。

本模块不复制官方 MATLAB 代码，按上述公开的数学定义独立实现。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

try:  # 允许作为脚本或包两种方式导入
    from .types import CameraConfig, PreprocProfile
except ImportError:  # pragma: no cover
    from types import (  # type: ignore  # noqa: F401
        CameraConfig,
        PreprocProfile,
    )
    from types import CameraConfig, PreprocProfile  # type: ignore


# ---------------------------------------------------------------- Bayer 解交织

#: 各 Bayer 相位在 2x2 单元中的位置（行, 列, 通道）
BAYER_PATTERNS = {
    "RGGB": ((0, 0, "R"), (0, 1, "G1"), (1, 0, "G2"), (1, 1, "B")),
    "BGGR": ((0, 0, "B"), (0, 1, "G1"), (1, 0, "G2"), (1, 1, "R")),
    "GRBG": ((0, 0, "G1"), (0, 1, "R"), (1, 0, "B"), (1, 1, "G2")),
    "GBRG": ((0, 0, "G1"), (0, 1, "B"), (1, 0, "R"), (1, 1, "G2")),
}


@dataclass(frozen=True)
class Measurement:
    """预处理输出。

    rgb  : (H, W, 3) 三通道测量，量纲与源数据一致（累积和，未归一化）
    diff : (H-1, W, 3) 垂直相邻差分
    """

    rgb: np.ndarray
    diff: np.ndarray

    @property
    def shape(self) -> Tuple[int, int, int]:
        return tuple(self.rgb.shape)  # type: ignore[return-value]


def deinterleave_bayer(raw: np.ndarray, pattern: str = "RGGB") -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Bayer 解交织并合并双绿通道。

    参数：
        raw     : 二维方图，边长必须为偶数
        pattern : 'RGGB' / 'BGGR' / 'GRBG' / 'GBRG'

    返回：
        (R, G, B)，每个为 (n/2, n/2) 的 float64 数组
    """
    if raw.ndim != 2:
        raise ValueError(f"raw 必须为二维数组，得到 {raw.shape}")
    h, w = raw.shape
    if h % 2 or w % 2:
        raise ValueError(f"raw 边长必须为偶数，得到 {raw.shape}")
    if pattern not in BAYER_PATTERNS:
        raise ValueError(f"未知 Bayer 相位 {pattern}，支持 {sorted(BAYER_PATTERNS)}")

    planes = {}
    for r, c, ch in BAYER_PATTERNS[pattern]:
        planes[ch] = np.asarray(raw[r::2, c::2], dtype=np.float64)

    g = (planes["G1"] + planes["G2"]) / 2.0
    return planes["R"], g, planes["B"]


# ---------------------------------------------------------------- 分块与累积


def center_crop(img: np.ndarray, size: int) -> np.ndarray:
    """居中裁剪为 size x size。若原图已等于 size 则原样返回。"""
    h, w = img.shape[:2]
    if (h, w) == (size, size):
        return img
    if size > h or size > w:
        raise ValueError(f"裁剪尺寸 {size} 超过原图 {img.shape[:2]}")
    top = (h - size) // 2
    left = (w - size) // 2
    return img[top:top + size, left:left + size]


def block_mean(channel: np.ndarray, block: int) -> np.ndarray:
    """对单通道平面做 block x block 分块平均。

    实现说明：使用 reshape 的视图切分，禁止把二维索引展平后跨行分块——
    这会导致分块跨越行边界，是预处理里最常见的错误之一（对应测试
    test_block_mean_does_not_cross_rows）。
    """
    if channel.ndim != 2:
        raise ValueError(f"channel 必须为二维数组，得到 {channel.shape}")
    h, w = channel.shape
    if h % block or w % block:
        raise ValueError(f"平面 {channel.shape} 不能被 block={block} 整除")
    bh, bw = h // block, w // block
    return channel.reshape(bh, block, bw, block).mean(axis=(1, 3))


def block_sum(channel: np.ndarray, block: int) -> np.ndarray:
    """分块求和（用于核对定点实现的位宽预算）。"""
    if channel.ndim != 2:
        raise ValueError(f"channel 必须为二维数组，得到 {channel.shape}")
    h, w = channel.shape
    if h % block or w % block:
        raise ValueError(f"平面 {channel.shape} 不能被 block={block} 整除")
    bh, bw = h // block, w // block
    return channel.reshape(bh, block, bw, block).sum(axis=(1, 3))


def accumulate(frames: np.ndarray) -> np.ndarray:
    """帧累积：直接求和，不除以帧数（规则 R3，已由官方数据验证）。

    参数 frames: shape (n, H, W) 或 (H, W)
    """
    a = np.asarray(frames, dtype=np.float64)
    if a.ndim == 2:
        return a.copy()
    if a.ndim != 3:
        raise ValueError(f"frames 应为 (n,H,W) 或 (H,W)，得到 {a.shape}")
    return a.sum(axis=0)


def vertical_diff(meas: np.ndarray) -> np.ndarray:
    """垂直相邻差分（论文式(7) 的 D 算子）。

    返回 shape (H-1, W, C)。差分为「下一行减上一行」。
    """
    a = np.asarray(meas, dtype=np.float64)
    if a.ndim != 3:
        raise ValueError(f"meas 应为 (H,W,C)，得到 {a.shape}")
    return a[1:, :, :] - a[:-1, :, :]


def horizontal_diff(meas: np.ndarray) -> np.ndarray:
    """水平相邻差分（作为消融对照，主链路不使用）。"""
    a = np.asarray(meas, dtype=np.float64)
    if a.ndim != 3:
        raise ValueError(f"meas 应为 (H,W,C)，得到 {a.shape}")
    return a[:, 1:, :] - a[:, :-1, :]


# ---------------------------------------------------------------- 主流程


def preprocess_raw(
    raw: np.ndarray,
    profile: PreprocProfile,
    camera: Optional[CameraConfig] = None,
    dark_level: float = 0.0,
) -> Measurement:
    """对单帧（或已累积）RAW 图执行完整预处理。

    参数：
        raw        : (N, N) RAW 图；N 应 >= profile.crop
        profile    : 预处理配置
        camera     : 相机配置（提供 Bayer 相位）；None 时用 RGGB
        dark_level : 暗电平偏置，先减后处理

    返回：
        Measurement(rgb, diff)
    """
    profile.validate()
    cam = camera or CameraConfig()
    a = np.asarray(raw, dtype=np.float64)
    if a.ndim != 2:
        raise ValueError(f"raw 必须为二维数组，得到 {a.shape}")

    a = a - float(dark_level)
    a = center_crop(a, profile.crop)

    r, g, b = deinterleave_bayer(a, cam.bayer_pattern)
    rgb = np.dstack([
        block_mean(r, profile.block),
        block_mean(g, profile.block),
        block_mean(b, profile.block),
    ])
    return Measurement(rgb=rgb, diff=vertical_diff(rgb))


def preprocess_frames(
    frames: Sequence[np.ndarray] | np.ndarray,
    profile: PreprocProfile,
    camera: Optional[CameraConfig] = None,
    dark_level: float = 0.0,
) -> Measurement:
    """对帧序列先累积再做预处理（等价于逐帧处理后再求和）。"""
    acc = accumulate(np.asarray(frames, dtype=np.float64) if not isinstance(frames, np.ndarray) else frames)
    return preprocess_raw(acc, profile, camera=camera, dark_level=dark_level)


def preprocess_mat(path: str, profile: PreprocProfile,
                   camera: Optional[CameraConfig] = None,
                   dark_level: float = 0.0) -> Measurement:
    """从官方 .mat 文件直接预处理。"""
    from .io_mat import load_image  # 延迟导入，避免循环依赖
    return preprocess_raw(load_image(path), profile, camera=camera, dark_level=dark_level)


# ---------------------------------------------------------------- 归一化


def normalize(meas: np.ndarray, profile: PreprocProfile,
              camera: Optional[CameraConfig] = None,
              frames: Optional[int] = None) -> Tuple[np.ndarray, float]:
    """把累积和归一化到 [0,1] 附近，返回 (归一化数组, 所用除数)。

    除数 = full_scale * frames。这是定点设计的推荐归一化口径：
    归一化后每个样本落在 [0,1]，块内平均与帧累积带来的位宽增长
    由定点配置单独核算（见 FixedPointConfig）。

    注意：定点流水必须在归一化之后再进入矩阵运算，否则 16~23 bit 的
    动态范围会在 MAC 累加中溢出。
    """
    cam = camera or CameraConfig()
    n = frames if frames is not None else profile.frames
    divisor = float(cam.full_scale) * float(max(n, 1))
    return np.asarray(meas, dtype=np.float64) / divisor, divisor
