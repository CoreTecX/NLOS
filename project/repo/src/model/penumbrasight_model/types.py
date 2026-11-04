# -*- coding: utf-8 -*-
"""配置数据类。

所有配置一经构造即不可变（frozen dataclass），避免在流水线中途被悄悄改动。
派生量（如测量维数、候选数）一律由字段计算得出，不允许手工填写。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple


@dataclass(frozen=True)
class PreprocProfile:
    """预处理配置。

    数据链路：RAW Bayer 方图
        -> 中心裁剪 (crop) -> Bayer 解交织 (plane = crop/2)
        -> block x block 分块平均 (out = plane/block)
        -> 帧累积（求和）-> 垂直差分

    属性：
        name        profile 名称
        crop        中心裁剪后的边长（原图像素）
        block       分块尺寸（解交织后平面内的像素数）
        frames      默认帧累积数（1 表示单帧）

    说明：等效原图像素块 = block * 2。
        官方配置等效 16x16 原图像素块；本方案 B 档等效 20x20。
    """

    name: str
    crop: int
    block: int
    frames: int = 1

    @property
    def plane(self) -> int:
        """解交织后单通道平面的边长。"""
        return self.crop // 2

    @property
    def out_size(self) -> int:
        """分块平均后测量图的边长。"""
        return self.plane // self.block

    @property
    def measurement_shape(self) -> Tuple[int, int, int]:
        """三通道测量的形状 (H, W, C)。"""
        return (self.out_size, self.out_size, 3)

    @property
    def diff_shape(self) -> Tuple[int, int, int]:
        """垂直差分后的形状 (H-1, W, C)。"""
        return (self.out_size - 1, self.out_size, 3)

    @property
    def equivalent_original_block(self) -> int:
        """等效的原图像素块边长。"""
        return self.block * 2

    @property
    def diff_measurements(self) -> int:
        """差分测量总数 M。"""
        return self.diff_shape[0] * self.diff_shape[1]

    def validate(self) -> None:
        if self.crop % 2 != 0:
            raise ValueError(f"crop 必须为偶数，得到 {self.crop}")
        if self.plane % self.block != 0:
            raise ValueError(
                f"平面边长 {self.plane} 不能被 block {self.block} 整除"
            )
        if self.frames < 1:
            raise ValueError(f"frames 必须 >= 1，得到 {self.frames}")


#: 官方配置：2016 -> 1008 -> 8x8 分块 -> 126x126
#: 参数依据见 data/inventory/data_contract.md（numPixels=1008, walln_points=126）
PROFILE_OFFICIAL = PreprocProfile(name="official", crop=2016, block=8, frames=1)

#: 本方案 B 档：中心裁剪 1920 -> 960 -> 10x10 分块 -> 96x96
#: 等效原图像素块 20x20，大于官方 16x16，噪声抑制更好
PROFILE_B = PreprocProfile(name="b", crop=1920, block=10, frames=4)

#: 本方案 A 档保底：中心裁剪 1280 -> 640 -> 10x10 分块 -> 64x64
PROFILE_A = PreprocProfile(name="a", crop=1280, block=10, frames=8)


@dataclass(frozen=True)
class CameraConfig:
    """相机与采集配置。"""

    bit_depth: int = 12
    full_scale: int = 65535          # 官方数据的实际满量程容器
    bayer_pattern: str = "RGGB"
    raw_shape: Tuple[int, int] = (2016, 2016)
    dark_level: float = 0.0


@dataclass(frozen=True)
class FixedPointConfig:
    """定点格式配置（与设计文档第 6.3 节对应）。

    位宽链依据 data_contract.md 第 5.3 节重新核算：
    源数据按 16 bit 满量程参与累加，故块和需 16 + ceil(log2(block^2)) 位。
    """

    raw_bits: int = 16               # 源容器位宽（非 12，见数据契约）
    block_bits: int = 0              # 由 block 计算，0 表示自动
    frame_acc_bits: int = 0          # 由 frames 计算，0 表示自动
    diff_bits: int = 24
    coeff_bits: int = 16
    mac_acc_bits: int = 48

    def block_sum_bits(self, block: int) -> int:
        """块累加所需位宽：源位宽 + ceil(log2(block^2))。

        **口径警告（2026-09-27 补）**：本方法返回的是**单通道**（R 或 B）口径。
        真实最坏通道是**双绿 G1+G2** —— 原则 P2 规定双绿按「和」进累加、不做 0.5
        舍入，所以它比单通道**再多 1 bit**。

            >>> FixedPointConfig(raw_bits=16).block_sum_bits(10)   # 单通道
            23
            >>> FixedPointConfig(raw_bits=16).block_sum_bits_dual_green(10)  # 容器
            24

        两者不是一回事：**容器**必须按最坏通道取宽（12 bit 源 → 20；16 bit 源 → 24）。
        硬件侧 `src/hardware/preprocess.hpp` 的 `PS_BS_BITS` 已按容器口径实现，
        回归由 `test_preprocess.py::test_fixed_point_bit_budget` 与
        `sim/results/x29_config_consistency.md §1` 覆盖。

        本方法**刻意保持单通道口径不改** —— 它的 docstring 与既有判据都是按此写的，
        改它会动到既有断言。需要容器口径请用 `block_sum_bits_dual_green()`。
        """
        import math
        return self.raw_bits + math.ceil(math.log2(block * block))

    def block_sum_bits_dual_green(self, block: int) -> int:
        """块和**容器**所需位宽（最坏通道 = 双绿 G1+G2，比单通道多 1 bit）。

        12 bit 源、10x10 分块 → 20；16 bit 源、10x10 分块 → 24。
        这是配置与硬件位宽表应当采用的口径；`block_sum_bits()` 是它的单通道对照。
        """
        return self.block_sum_bits(block) + 1

    def frame_sum_bits(self, block: int, frames: int) -> int:
        """帧累积后的位宽：块和位宽 + ceil(log2(frames))。"""
        import math
        return self.block_sum_bits(block) + math.ceil(math.log2(max(frames, 1)))
