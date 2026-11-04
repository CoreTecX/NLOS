# -*- coding: utf-8 -*-
"""诊断：A 的行序 与 测量展平序 是否匹配。

背景
----
paper_forward.py 里 A 的行是用 `image[::-1,:].ravel()` 产生的（C 序，即 Fortran
索引 p = i*126 + j），而 paper_recon.py 里测量用的是 `m.ravel(order="F")`
（p = i + 126*j）。两者若不一致，重建问题本身就是错配的 —— 这才是 FISTA
「数据项不降」的真正候选原因，而不是求解器差。

做法
----
把 A 的行序枚举为 4 种：{C 序, F 序} × {保持 flipud, 撤掉 flipud}；
测量展平枚举为同样的 4 种（flipud 由载入端决定）。
对每种组合，取论文口径的最小二乘解
        x = argmin || dA x - dm ||_2
并报告相对残差 ||dA x - dm|| / ||dm||。
匹配正确的组合残差应显著更低。
"""
import itertools
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
N = 126
SHAPE = (29, 36)


def rows_A(A, unflip, order):
    """把 A 的行重排到指定 (是否撤 flipud, 展平序)。"""
    A3 = A.reshape(N, N, A.shape[1])          # 第 0/1 轴 = C 序下的 (i, j)
    if unflip:
        A3 = A3[::-1, :, :]
    if order == "F":
        A3 = A3.transpose(1, 0, 2)            # 交换两轴 → 等价 F 序展平
    return np.ascontiguousarray(A3).reshape(N * N, A.shape[1])


def rows_m(m, unflip, order):
    if unflip:
        m = m[::-1, :]
    return np.ascontiguousarray(m).ravel(order=order)


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


def main():
    A = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    print(f"A {A.shape} | 测量 {meas.shape} | 真值 {gt.shape}")

    avars = list(itertools.product(("C", "F"), (False, True)))
    mvars = list(itertools.product(("C", "F"), (False, True)))

    table = {}
    for oa, ua in avars:
        t0 = time.time()
        Ar = rows_A(A, ua, oa)
        dA = drows(Ar)
        AT = dA.T
        ATA = AT @ dA
        print(f"\nA 变体 序={oa} 撤flip={ua}  →  ATA 用时 {time.time()-t0:.1f}s")
        for ci, cn in enumerate(("R", "G", "B")):
            m3 = meas[:, :, ci]
            for om, um in mvars:
                dm = dvec(rows_m(m3, um, om))
                x = np.linalg.solve(ATA, AT @ dm)
                r = float(np.linalg.norm(dA @ x - dm) / np.linalg.norm(dm))
                table[(oa, ua, om, um, cn)] = r

    print("\n" + "=" * 78)
    print("相对残差 ||dA x - dm|| / ||dm||   （越小越匹配；按三通道平均排序）")
    print("=" * 78)
    print(f"{'A序':<4}{'A撤flip':<9}{'m序':<4}{'m撤flip':<9}"
          f"{'R':>12}{'G':>12}{'B':>12}{'均值':>12}")
    rows = []
    for oa, ua in avars:
        for om, um in mvars:
            vals = [table[(oa, ua, om, um, cn)] for cn in ("R", "G", "B")]
            rows.append((float(np.mean(vals)), oa, ua, om, um, vals))
    rows.sort()
    for mean, oa, ua, om, um, vals in rows:
        flag = "  <== 一致" if ua == um else ""
        print(f"{oa:<4}{str(ua):<9}{om:<4}{str(um):<9}"
              f"{vals[0]:>12.5f}{vals[1]:>12.5f}{vals[2]:>12.5f}{mean:>12.5f}{flag}")

    best = rows[0]
    print(f"\n最佳组合：A序={best[1]} 撤flip={best[2]}  |  "
          f"m序={best[3]} 撤flip={best[4]}   平均残差 {best[0]:.5f}")
    print("当前代码实际使用：A序=C 撤flip=False  |  m序=F 撤flip=False")


if __name__ == "__main__":
    main()
