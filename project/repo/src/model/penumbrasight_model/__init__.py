# -*- coding: utf-8 -*-
"""PenumbraSight 参考模型包。

本包是 PC 侧的离线参考实现：几何前向模型、预处理、定位、重建与定点量化的
浮点/整数基准。FPGA 端的 RTL 与 HLS 实现以本包的输出作为对照标准。

子模块：
    types            配置数据类（几何、相机、预处理、定点）
    io_mat           官方 .mat 数据的读取与帧序列管理
    preprocess       Bayer 解交织、分块、帧累积、垂直差分
    forward          几何前向模型、可见性、光传输矩阵、投影能量
    localization     差分框架下的粗到细遮挡物定位
    reconstruction   TV 正则化 PDHG 重建与质量度量

许可与来源边界见仓库根目录 third_party/NOTICE.md。
"""

from .types import (
    CameraConfig,
    FixedPointConfig,
    PreprocProfile,
    PROFILE_A,
    PROFILE_B,
    PROFILE_OFFICIAL,
)

__all__ = [
    "PreprocProfile",
    "PROFILE_A",
    "PROFILE_B",
    "PROFILE_OFFICIAL",
    "CameraConfig",
    "FixedPointConfig",
    "io_mat",
    "preprocess",
    "forward",
    "localization",
    "reconstruction",
]
