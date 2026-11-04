# -*- coding: utf-8 -*-
"""检验：模型 y=A·x_gt 与测量之间是否存在整体像素级错位。

若 A 与测量只是「整体错开几个像素」，那么把模型平移 (dz,dx) 后再做差分、
并与测量的差分做最小二乘标度匹配，残差会大幅下降。
同时对「垂直差分(F 序)」和「水平差分(C 序)」两种情况做对比。
"""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N, ROWS, COLS = 126, 29, 36


def dvec_C(M):
    """C 序展平后做相邻差分（水平差分）。"""
    v = np.ascontiguousarray(M).ravel(order="C")
    out = np.empty_like(v)
    out[:-1] = v[:-1] - v[1:]
    out[-1] = v[-1]
    return out


def dvec_F(M):
    """F 序展平后做相邻差分（垂直差分）。"""
    v = np.ascontiguousarray(M).ravel(order="F")
    out = np.empty_like(v)
    out[:-1] = v[:-1] - v[1:]
    out[-1] = v[-1]
    return out


def resid_after_gain(dy, dm):
    """给定模型差分 dy 与测量差分 dm，先估最佳标度，再报相对残差。"""
    den = float(dy @ dy)
    if den <= 0:
        return float("inf")
    a = float(dy @ dm) / den
    return float(np.linalg.norm(a * dy - dm) / np.linalg.norm(dm))


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))

    x_gt = gt[:, :, 0].astype(float).ravel(order="F")
    y = (A0 @ x_gt).reshape(N, N)            # 模型预测的墙图（C 序 = 翻转后的像）

    for ci, cn in enumerate("RGB"):
        m = meas[:, :, ci]
        print(f"\n===== 通道 {cn} =====")
        for tag, dfun in (("水平差分(C)", dvec_C), ("垂直差分(F)", dvec_F)):
            dm = dfun(m)
            best = (float("inf"), 0, 0)
            for dz in range(-4, 5):
                for dx in range(-4, 5):
                    ys = np.roll(np.roll(y, dz, axis=0), dx, axis=1)
                    r = resid_after_gain(dfun(ys), dm)
                    if r < best[0]:
                        best = (r, dz, dx)
            r0 = resid_after_gain(dfun(y), dm)
            print(f"  {tag}: 零平移残差 {r0:.4f} →  最佳 ({best[1]:+d},{best[2]:+d}) "
                  f"残差 {best[0]:.4f}")


if __name__ == "__main__":
    main()
