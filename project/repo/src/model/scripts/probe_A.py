# -*- coding: utf-8 -*-
"""数值探查 paper_A_va1.npy 的单列结构：是否有明显的遮挡阴影缺口。"""
import os
import numpy as np

# 由本文件位置推导（见 docs/environment_ascii_build.md）：同一份源码要能在工作区检出
# 与 ASCII 构建区 D:\psight\penumbrasight 两处都跑，故不写死绝对路径。
VEC = os.path.join(os.path.abspath(os.path.join(
    os.path.dirname(__file__), os.pardir, os.pardir, os.pardir)), "sim", "vectors")
A = np.load(os.path.join(VEC, "paper_A_va1.npy"))
print(f"A {A.shape}  范围 [{A.min():.4f}, {A.max():.4f}]  非零 {np.count_nonzero(A)}")
print(f"列和范围 [{A.sum(0).min():.2f}, {A.sum(0).max():.2f}]")

# 列索引 k = (35-mc)*29 + mr（mc 为显示器列，mr 为显示器行）
for k in (0, 14, 29, 500, 750, 1043):
    mc = 35 - k // 29
    mr = k % 29
    c = A[:, k].reshape(126, 126)          # C 序 → [i, j]，i 为翻转过后的行
    print(f"\n--- 列 {k} (显示器 mc={mc}, mr={mr})  范围 [{c.min():.4f}, {c.max():.4f}]"
          f"  均值 {c.mean():.4f} ---")
    ds = c.reshape(9, 14, 9, 14).mean(axis=(1, 3))     # 粗看 9x14（图像整体转置后显示）
    for r in range(9):
        print(f"  {r:2d} " + "".join(f"{v:8.3f}" for v in ds[r]))

# 阴影深度：同一列内 min/max
print("\n各列 min/max 比值（越小说明缺口越深）")
ratios = np.array([A[:, k].max() and A[:, k].min() / A[:, k].max()
                   for k in range(0, 1044, 60)])
print(np.round(ratios, 4))
