# -*- coding: utf-8 -*-
"""用论文口径的 A（修正行序/列序约定）+ 已验证的 TV-PDHG，重跑差分重建。

修正的两个约定
--------------
1. **行序**：A 的行与测量向量必须用同一种展平序。diag_row_order.py 实测：
   A 的行是「C 序 + 不翻转」，测量也应是「C 序 + 不翻转」才匹配
   （相对残差 0.173，而 paper_recon.py 实际用的 A=C / m=F 是 0.647，最差组合）。
2. **列序**：A 的列（隐藏场景块）按 MATLAB 的 `reshape(x,29,36)` 是 F 序，
   而 tv_pdhg 内部按 C 序 reshape，故需对 A 的列做一次置换。

λ 标定
------
论文目标 0.5||cs*dA*x - mr||^2 + reg*TV(x)，cs = cscale。
令 u = cs*x，等价于 0.5||dA*u - mr||^2 + (reg/cs)*TV(u)，最终 x = u/cs。
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

from penumbrasight_model.reconstruction import pearson, tv_pdhg  # noqa: E402

VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
ROWS, COLS = 29, 36
SHAPE = (ROWS, COLS)
CSCALE = np.array([312.5, 425.34722222, 496.54222222])
REG = 1e07 * np.array([5.0, 25.0, 25.0])


def drows(M):
    out = np.empty_like(M)
    out[:-1, :] = M[:-1, :] - M[1:, :]
    out[-1, :] = M[-1, :]
    return out


def dvec(v):
    out = np.empty_like(v)
    out[:-1] = v[:-1] - v[1:]
    out[-1] = v[-1]
    return out


def col_perm():
    """perm[k_C] = k_F：把「F 序列」的 A 换成「C 序列」。"""
    return np.arange(ROWS * COLS).reshape(ROWS, COLS, order="F").ravel(order="C")


def main():
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    print(f"A {A0.shape} | 测量 {meas.shape} | 真值 {gt.shape}")

    A = np.ascontiguousarray(A0[:, col_perm()])
    dA = drows(A)
    AT = dA.T
    ATA = AT @ dA

    # 精确算子范数（1044x1044 特征分解），用于把问题归一化到 ||dA'||_2 = 1
    ev = np.linalg.eigvalsh(ATA)
    lam_max = float(ev[-1])
    print(f"dA 算子范数平方 = {lam_max:.6e}   条件数 = {ev[-1]/max(ev[0],1e-30):.3e}")

    s = np.sqrt(lam_max)                      # 归一化因子
    dAs = dA / s

    out = np.zeros(SHAPE + (3,))
    summary = []
    for ci, cn in enumerate(("R", "G", "B")):
        m3 = meas[:, :, ci]
        dm = dvec(np.ascontiguousarray(m3).ravel(order="C"))
        g = gt[:, :, ci].astype(float)
        cs = CSCALE[ci]
        reg = REG[ci]

        # ---- 1) 最小二乘解（λ=0），最干净的一步检验
        t0 = time.time()
        u_ls = np.linalg.solve(ATA, AT @ dm)
        r_ls = pearson(u_ls, g.ravel(order="C"))
        # 相关系数对尺度不变，但报一下物理量级（x = u/cs）
        x_ls = u_ls / cs
        print(f"\n通道 {cn}: 最小二乘 r={r_ls:+.4f}  "
              f"u 范围 [{u_ls.min():.3e},{u_ls.max():.3e}]  "
              f"x 范围 [{x_ls.min():.3e},{x_ls.max():.3e}]  ({time.time()-t0:.1f}s)")

        # ---- 2) PDHG（从最小二乘解热启动），λ 扫描
        lam_base = reg / cs
        print(f"  论文 λ = reg/cs = {lam_base:.4e}  （归一化后 λ' = λ/s² = "
              f"{lam_base/s**2:.4e}）")
        best = (None, abs(r_ls), u_ls / cs, "LS")
        for factor in (1e-3, 1e-2, 0.1, 1.0, 10.0, 100.0):
            lam = lam_base / (s ** 2) * factor
            t0 = time.time()
            res = tv_pdhg(dAs, dm / s, SHAPE, lam=lam, n_iter=800,
                          nonneg=False, L2=None, record_every=200)
            u = np.asarray(res.image).reshape(-1) * s
            r = pearson(u, g.ravel(order="C"))
            dt = time.time() - t0
            obj = res.objective[-1]
            print(f"    λ'={lam:.3e} (×{factor:<7g}) r={r:+.4f}  "
                  f"obj={obj['obj']:.4e} (data {obj['data']:.4e}, tv {obj['tv']:.4e})"
                  f"  {dt:.1f}s")
            if abs(r) > best[1]:
                best = (factor, abs(r), u / cs, "PDHG")

        out[:, :, ci] = best[2].reshape(SHAPE, order="C")
        summary.append((cn, r_ls, best[0], best[3], best[1]))
        print(f"  通道 {cn} 最佳：{best[3]} λ 因子 {best[0]}  r={best[1]:+.4f}")

    print("\n" + "=" * 72)
    print("汇总")
    print(f"{'通道':<6}{'最小二乘 r':>14}{'最佳 λ 因子':>14}{'方案':>8}{'最佳 r':>12}")
    for cn, r_ls, fac, tag, r in summary:
        print(f"{cn:<6}{r_ls:>14.4f}{str(fac):>14}{tag:>8}{r:>12.4f}")

    np.save(os.path.join(VEC, "paper_recon_pdhg.npy"), out)
    print(f"\n已保存 {os.path.join(VEC, 'paper_recon_pdhg.npy')}")

    # 目视对照图
    try:
        import matplotlib
        matplotlib.use("Agg")
        matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK SC",
                                                  "WenQuanYi Micro Hei",
                                                  "Microsoft YaHei", "SimHei"]
        matplotlib.rcParams["axes.unicode_minus"] = False
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 3, figsize=(13, 8))
        for ci, cn in enumerate(("R", "G", "B")):
            g = gt[:, :, ci].astype(float)
            x = out[:, :, ci]
            axes[0, ci].imshow(g, cmap="gray")
            axes[0, ci].set_title(f"真值 {cn}")
            axes[0, ci].axis("off")
            axes[1, ci].imshow(x, cmap="gray")
            axes[1, ci].set_title(f"重建 {cn}  r={pearson(x, g):+.3f}")
            axes[1, ci].axis("off")
        fig.suptitle("论文口径 A + TV-PDHG 重建 vs 真值")
        fig.tight_layout()
        p = os.path.join(VEC, "paper_recon_pdhg_fig.png")
        fig.savefig(p, dpi=120)
        print(f"已保存对照图 {p}")
    except Exception as e:
        print(f"绘图跳过：{e}")


if __name__ == "__main__":
    main()
