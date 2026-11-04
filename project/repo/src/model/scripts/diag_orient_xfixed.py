# -*- coding: utf-8 -*-
"""在「x 固定为真值」的前提下，比较模型预测与测量的 8 种朝向组合。

比 diag_meas_transform 更严格：那里 x 是自由的最小二乘解，可以「迁就」错的 A；
这里把 x 钉死在真值上，所以残差直接反映 A 本身对不对。
"""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N = 126

TF = {
    "恒等": lambda m: m,
    "上下翻转": lambda m: m[::-1, :],
    "左右翻转": lambda m: m[:, ::-1],
    "旋转180": lambda m: m[::-1, ::-1],
    "转置": lambda m: m.T,
    "转置+上下": lambda m: m.T[::-1, :],
    "转置+左右": lambda m: m.T[:, ::-1],
    "转置+180": lambda m: m.T[::-1, ::-1],
}


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

    x_gt = gt[:, :, 0].astype(float).ravel(order="F")
    y = (A0 @ x_gt).reshape(N, N)

    m = meas[:, :, 0]
    dmC, dmF = dvec_C(m), dvec_F(m)

    print(f"{'模型变换':<12}{'C 残差':>12}{'F 残差':>12}{'均值':>12}")
    rows = []
    for name, T in TF.items():
        yt = T(y)
        rc = resid(dvec_C(yt), dmC)
        rf = resid(dvec_F(yt), dmF)
        rows.append(((rc + rf) / 2, name, rc, rf))
        print(f"{name:<12}{rc:>12.4f}{rf:>12.4f}{(rc+rf)/2:>12.4f}")

    # 也试试把模型当作行序不变、只用 F 序展平测量的那种（等价于对 y 取转置）
    rows.sort()
    print(f"\n最优：{rows[0][1]}  C={rows[0][2]:.4f}  F={rows[0][3]:.4f}")
    print(f"当前实现（x 列为 F 序、A 行为 C 序）对应「恒等」："
          f"C={[r for r in rows if r[1]=='恒等'][0][2]:.4f}  "
          f"F={[r for r in rows if r[1]=='恒等'][0][3]:.4f}")


if __name__ == "__main__":
    main()
