# -*- coding: utf-8 -*-
"""官方 .mat 数据的读取与帧序列管理。

只读。不修改、不复制原始数据。
许可边界：官方数据集无 LICENSE，不可再分发，仅作算法基线来源（见 third_party/NOTICE.md）。
"""

from __future__ import annotations

import os
import re
import warnings
from typing import Dict, List, Sequence

import numpy as np
import scipy.io as sio

#: 官方 .mat 已知的结构瑕疵：部分文件含重复变量名，scipy 会给出 MatReadWarning。
#: 这是官方文件自身的问题，不影响正常字段读取。
try:
    from scipy.io.matlab import MatReadWarning
except ImportError:  # Older supported development installations.
    from scipy.io.matlab.miobase import MatReadWarning
warnings.filterwarnings("ignore", category=MatReadWarning)


def load_mat(path: str) -> Dict[str, np.ndarray]:
    """读取一个 .mat 文件，返回去掉元数据的变量字典。"""
    d = sio.loadmat(path)
    return {k: v for k, v in d.items() if not k.startswith("__")}


def load_image(path: str) -> np.ndarray:
    """读取测量文件中的 RAW 图像数组（通常为 2016x2016）。"""
    d = load_mat(path)
    if "image" not in d:
        raise KeyError(f"{os.path.basename(path)} 中不存在 image 字段，"
                       f"现有字段：{sorted(d)}")
    return np.asarray(d["image"], dtype=np.float64)


def load_metadata(path: str) -> Dict[str, object]:
    """读取随文件携带的元数据（pixel_size、shutter_speed、iterations 等）。"""
    d = load_mat(path)
    out: Dict[str, object] = {}
    for k in ("pixel_size", "shutter_speed", "iterations", "ground_truth"):
        if k in d:
            a = np.asarray(d[k])
            out[k] = a.reshape(-1)[0].item() if a.size == 1 else a
    return out


def _frame_index(name: str) -> int:
    """从形如 image_test_it12.mat 的文件名中提取帧序号。"""
    m = re.search(r"it(\d+)", name)
    if m:
        return int(m.group(1))
    m = re.search(r"frame_?(\d+)", name)
    if m:
        return int(m.group(1))
    m = re.search(r"(\d+)", name)
    return int(m.group(1)) if m else 0


def list_frame_series(directory: str) -> List[str]:
    """列出目录中按帧序号排序的 .mat 帧序列（完整路径）。"""
    names = [f for f in os.listdir(directory) if f.lower().endswith(".mat")]
    names.sort(key=_frame_index)
    return [os.path.join(directory, n) for n in names]


def load_frame_series(directory: str) -> np.ndarray:
    """读取一个帧序列，返回 shape=(n_frames, H, W) 的数组。

    内存提示：每帧 2016x2016 float64 约 32 MB；35 帧约 1.1 GB。
    大序列建议改用 iter_frames() 逐帧处理。
    """
    paths = list_frame_series(directory)
    if not paths:
        raise FileNotFoundError(f"目录中没有 .mat 帧：{directory}")
    return np.stack([load_image(p) for p in paths], axis=0)


def iter_frames(directory: str, dtype=np.float64):
    """逐帧迭代，避免一次性载入整个序列。"""
    for p in list_frame_series(directory):
        yield os.path.basename(p), np.asarray(load_image(p), dtype=dtype)


def sum_frames(directory: str, dtype=np.float64) -> np.ndarray:
    """对一个帧序列求和（官方规则：累积为求和，不除以帧数）。"""
    acc = None
    n = 0
    for _name, img in iter_frames(directory, dtype=dtype):
        acc = img.copy() if acc is None else acc + img
        n += 1
    if acc is None:
        raise FileNotFoundError(f"目录中没有 .mat 帧：{directory}")
    return acc


def frame_count(directory: str) -> int:
    return len(list_frame_series(directory))


def load_sequence(paths: Sequence[str]) -> np.ndarray:
    return np.stack([load_image(p) for p in paths], axis=0)
