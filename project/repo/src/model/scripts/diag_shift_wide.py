# -*- coding: utf-8 -*-
"""大范围（±40 px）平移搜索：模型 y=A·x_gt 相对测量是否存在整体错位。

若存在较大的整体错位，说明某个几何量（遮挡物位置 / 墙面 z 范围 / D）偏了。
"""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N = 126
RANGE = 40


def dvec_C(M):
    v = np.ascontiguousarray(M).ravel(order="C")
    o = np.empty_like(v); o[:-1] = v[:-1] - v[1:]; o[-1] = v[-1]
    return o


def dvec_F(M):
    v = np.ascontiguousarray(M).ravel(order="F")
    o = np.empty_like(v); o[:-1] = v[:-1] - v[1:]; o[-1] = v[-1]
    return o


def resid(dy, dm):
    den = float(dy @ dy)
    if den <= 0:
        return float("inf")
    a = float(dy @ dm) / den
    return float(np.linalg.norm(a * dy - dm) / np.linalg.norm(dm))


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    m = meas[:, :, 0]

    for tag, (xf, y) in {"x=真值(不翻转)": (None, None)}.items():
        pass

    y0 = (A0 @ gt[:, :, 0].astype(float).ravel(order="F")).reshape(N, N)
    xf0 = (A0 @ gt[:, :, 0].astype(float)[::-1, :].ravel(order="F")).reshape(N, N)

    dmC, dmF = dvec_C(m), dvec_F(m)
    for yname, y in (("y 原样", y0), ("y 上下翻转", xf0)):
        for tag, dfun in (("水平C", dvec_C), ("垂直F", dvec_F)):
            r0 = resid(dfun(y), dmC if tag == "水平C" else dmF)
            best = (r0, 0, 0)
            for dz in range(-RANGE, RANGE + 1, 2):
                for dx in range(-RANGE, RANGE + 1, 2):
                    ys = np.roll(np.roll(y, dz, 0), dx, 1)
                    r = resid(dfun(ys), dmC if tag == "水平C" else dmF)
                    if r < best[0]:
                        best = (r, dz, dx)
            print(f"{yname:<12}{tag:<6} 零平移 {r0:.4f} → 最佳 ({best[1]:+4d},{best[2]:+4d}) "
                  f"{best[0]:.4f}")


if __name__ == "__main__":
    main()
