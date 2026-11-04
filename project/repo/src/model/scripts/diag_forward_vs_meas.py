# -*- coding: utf-8 -*-
"""决定性检验：把真值图像代进前向模型 A@x_gt，看它像不像测量。

若几何/朝向正确，y_sim = A @ x_gt 应当与测量图（同一行序）高度相关，
并且能看出 BU 形状的阴影调制。

枚举：
  A 行序 {C, F} × 撤 flipud {否, 是}
  x_gt 展平序 {F, F + 列翻转}
  m 展平序与 A 保持一致
"""
import itertools
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N, ROWS, COLS = 126, 29, 36


def rows_A(A, unflip, order):
    A3 = A.reshape(N, N, A.shape[1])
    if unflip:
        A3 = A3[::-1, :, :]
    if order == "F":
        A3 = A3.transpose(1, 0, 2)
    return np.ascontiguousarray(A3).reshape(N * N, A.shape[1])


def rows_m(m, unflip, order):
    if unflip:
        m = m[::-1, :]
    return np.ascontiguousarray(m).ravel(order=order)


def pearson(a, b):
    a = a.ravel() - a.mean()
    b = b.ravel() - b.mean()
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d > 0 else float("nan")


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    g = gt[:, :, 0].astype(float)

    xvars = {
        "F": g.ravel(order="F"),
        "F+列翻转": g[:, ::-1].ravel(order="F"),
        "C": g.ravel(order="C"),
        "C+列翻转": g[:, ::-1].ravel(order="C"),
    }

    print(f"{'A序':<4}{'撤flip':<8}{'x展平':<10}{'R':>10}{'G':>10}{'B':>10}")
    rows = []
    for order, unflip in itertools.product(("C", "F"), (False, True)):
        Av = rows_A(A0, unflip, order)
        for xn, x in xvars.items():
            y = Av @ x
            cs = []
            for ci in range(3):
                mv = rows_m(meas[:, :, ci], unflip, order)
                cs.append(pearson(y, mv))
            rows.append((np.mean(cs), order, unflip, xn, cs))
            print(f"{order:<4}{str(unflip):<8}{xn:<10}"
                  f"{cs[0]:>10.4f}{cs[1]:>10.4f}{cs[2]:>10.4f}")

    rows.sort(reverse=True)
    print("\n按平均相关排序（前 4）：")
    for r in rows[:4]:
        print(f"  A序={r[1]} 撤flip={r[2]}  x={r[3]}  平均 r={r[0]:+.4f}")

    # 目视：最佳组合下 y_sim 与测量
    _, order, unflip, xn, _ = rows[0]
    Av = rows_A(A0, unflip, order)
    y = Av @ xvars[xn]
    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 4, figsize=(16, 8))
        for ci, cn in enumerate("RGB"):
            mv = rows_m(meas[:, :, ci], unflip, order)
            axes[0, ci].imshow(mv.reshape(N, N) if order == "C"
                               else mv.reshape(N, N, order="F"), cmap="gray")
            axes[0, ci].set_title(f"测量 {cn}")
            axes[0, ci].axis("off")
        ys = y.reshape(N, N) if order == "C" else y.reshape(N, N, order="F")
        for ci in range(3):
            axes[1, ci].imshow(ys, cmap="gray")
            axes[1, ci].set_title(f"模型 y=A·x_gt（同图）")
            axes[1, ci].axis("off")
        # 归一化叠加比较
        mR = rows_m(meas[:, :, 0], unflip, order).reshape(ys.shape)
        a = (mR - mR.min()) / (mR.max() - mR.min())
        b = (ys - ys.min()) / (ys.max() - ys.min())
        axes[1, 3].imshow(a - b, cmap="bwr", vmin=-0.5, vmax=0.5)
        axes[1, 3].set_title("测量R − 模型（归一化）")
        axes[1, 3].axis("off")
        fig.suptitle(f"最佳组合：A序={order} 撤flip={unflip} x={xn}")
        fig.tight_layout()
        p = os.path.join(VEC, "forward_vs_meas.png")
        fig.savefig(p, dpi=110)
        print(f"已保存 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
