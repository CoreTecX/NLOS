# -*- coding: utf-8 -*-
"""对测量图做 8 种二面体变换，找与 A 最匹配的那一种。

A 固定为 C 序行 + 不翻转（paper_forward.py 的原样输出），只变换测量图。
判据：差分最小二乘残差，以及 LS 解与真值的相关系数。
"""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N, ROWS, COLS = 126, 29, 36


def dvec(v):
    out = np.empty_like(v)
    out[:-1] = v[:-1] - v[1:]
    out[-1] = v[-1]
    return out


def drows(M):
    out = np.empty_like(M)
    out[:-1, :] = M[:-1, :] - M[1:, :]
    out[-1, :] = M[-1, :]
    return out


def pearson(a, b):
    a = np.asarray(a, float).ravel() - np.mean(a)
    b = np.asarray(b, float).ravel() - np.mean(b)
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d > 0 else float("nan")


TRANSFORMS = {
    "恒等": lambda m: m,
    "上下翻转": lambda m: m[::-1, :],
    "左右翻转": lambda m: m[:, ::-1],
    "旋转180": lambda m: m[::-1, ::-1],
    "转置": lambda m: m.T,
    "转置+上下": lambda m: m.T[::-1, :],
    "转置+左右": lambda m: m.T[:, ::-1],
    "转置+180": lambda m: m.T[::-1, ::-1],
}


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    gC = gt[:, :, 0].astype(float).ravel(order="C")
    gF = gt[:, :, 0].astype(float).ravel(order="F")

    dA = drows(A0)
    AT = dA.T
    ATA = AT @ dA

    print(f"{'测量变换':<12}{'R残差':>10}{'r(gt,C)':>12}{'r(gt,F)':>12}"
          f"{'G残差':>10}{'B残差':>10}")
    rows = []
    for name, T in TRANSFORMS.items():
        res, rs = [], []
        for ci in range(3):
            dm = dvec(np.ascontiguousarray(T(meas[:, :, ci])).ravel(order="C"))
            x = np.linalg.solve(ATA, AT @ dm)
            res.append(float(np.linalg.norm(dA @ x - dm) / np.linalg.norm(dm)))
            if ci == 0:
                rs = [pearson(x, gC), pearson(x, gF)]
        rows.append((res[0], name, rs, res))
        print(f"{name:<12}{res[0]:>10.4f}{rs[0]:>12.4f}{rs[1]:>12.4f}"
              f"{res[1]:>10.4f}{res[2]:>10.4f}")

    rows.sort()
    print(f"\nR 残差最小：{rows[0][1]}  残差 {rows[0][0]:.4f}  "
          f"r(gt,C)={rows[0][2][0]:+.4f}  r(gt,F)={rows[0][2][1]:+.4f}")
    print("（当前代码用的是「恒等」）")


if __name__ == "__main__":
    main()
