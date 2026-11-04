# -*- coding: utf-8 -*-
"""快速探查 gt / meas 的数值结构。"""
import os
import numpy as np
import scipy.io as sio

# VEC 由本文件位置推导（见 docs/environment_ascii_build.md）；MAT 指向仓库外的论文
# 数据集，那是**另一个**树、与本次 ASCII 整改无关，故仍保留绝对路径。
VEC = os.path.join(os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir, os.pardir)), "sim", "vectors")
MAT = (r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master"
       r"\Data\TestPosD11\image_test_bur20.mat")

gt = np.load(os.path.join(VEC, "paper_gt.npy"))
meas = np.load(os.path.join(VEC, "paper_meas.npy"))
print(f"gt {gt.shape} dtype {gt.dtype}")
for c, n in enumerate("RGB"):
    g = gt[:, :, c].astype(float)
    print(f"  {n}: min {g.min():.1f} max {g.max():.1f} mean {g.mean():.1f} "
          f"std {g.std():.3f} 唯一值 {len(np.unique(g))}")
print(f"meas {meas.shape} dtype {meas.dtype}")
for c, n in enumerate("RGB"):
    m = meas[:, :, c]
    print(f"  {n}: min {m.min():.1f} max {m.max():.1f} mean {m.mean():.1f}")

print("\n.mat 原始键：")
d = sio.loadmat(MAT)
for k, v in d.items():
    if k.startswith("__"):
        continue
    a = np.asarray(v)
    print(f"  {k}: shape {a.shape} dtype {a.dtype} "
          f"范围 [{a.min() if a.size else 0}, {a.max() if a.size else 0}]")

g0 = d["ground_truth"]
print(f"\nground_truth 原始: {g0.shape} {g0.dtype}")
print(g0[:, :, 0] if g0.ndim == 3 else g0)
