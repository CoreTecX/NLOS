# -*- coding: utf-8 -*-
"""x4j —— 定位判据原型（1 人日时间盒，只做 Python，不碰 FPGA）。

背景（见 x4i 报告）
------------------
论文的判据 `||U(:,1:K)' y||`（K = min(#{s_k/s_1 > sigma_th}, 10)）在 σ=0.75 下
K ≡ 1，因为 A 近秩 1（s2/s1 ≈ 0.18，相邻列 cos ≈ 0.99）。论文**自己保存的**能量面
top-1/top-2 只差 3.96e-4，而模型误差约 1% —— 差 25 倍，所以 argmax 由噪声决定。
论文自己的 Chair 结果 `best_pos = (0.5471, 0.6500, 0.2000)` 对真值
`(0.506, 0.594, 0.242)` 偏 6.3 cm，且 y 压在网格上界。

物理根源：遮挡物在显示器到墙的 55% 处（D/occ_y ≈ 1.81），阴影边缘的模糊宽度
`|1 - D/occ_y| × 显示器尺寸 ≈ 0.32 m` 比阴影本身投影宽度 `occ_size × D/occ_y ≈ 0.14 m`
还宽 2.3 倍，**本影被完全洗掉**，墙上没有可匹配的暗区，只剩平滑亮场上的弱调制。

本轮试几个替代判据。

预登记（先写下来，再看结果）
--------------------------
    C0_rgb  论文原判据，K=1，三通道逐通道 argmax 再平均（= 论文的做法）  预测：失败
    C0_r    同上但只用 r 通道（bu 场景真值只在 r 通道有内容）            预测：略好，仍失败
    C1_r    论文判据但 σ=0.001（K 放开）                                预测：可能改善，不稳
    C2_r    截断最小二乘解的负值占比 ||neg(x̂)||₁/||x̂||₁（吃非负先验）    预测：明显好于 C0/C1
    C3_r    非负最小二乘残差 ||y - A x̂||/||y||（投影梯度 200 步）        预测：最好
    C4_r    全秩子空间能量 ||P_A y||/||y||                              预测：失败（x4i 已测 K=全谱）

成功线：argmax 到真值 ≤ 0.03 m；边缘 ≤ 0.05 m；> 0.05 m 记失败。
若全部失败 → 按方案退到 C（定位降级），沉没成本封在本文件。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, os.pardir))
if _MODEL_DIR not in sys.path:
    sys.path.insert(0, _MODEL_DIR)
from penumbrasight_model import forward_loc as FL   # noqa: E402

REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")

DEFAULT_MAT = (r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master"
               r"\Data\TestPosD11\image_test_bur20.mat")


# ----------------------------------------------------------------------
def _nnls(G, b, y_norm2, eta, iters=200):
    """投影梯度非负最小二乘：min_{x>=0} x'Gx - 2x'b + |y|^2。

    返回 (x, 残差)。每步只花一次 G@x（1044²），所以很便宜。
    G 是 A'A（半正定），步长 eta = 1/lam_max 保证收敛。
    """
    x = np.zeros(G.shape[0], dtype=np.float64)
    for _ in range(iters):
        x = np.maximum(0.0, x - eta * (G @ x - b))
    resid2 = float(x @ (G @ x) - 2.0 * (x @ b) + y_norm2)
    return x, max(resid2, 0.0) ** 0.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mat", default=DEFAULT_MAT)
    ap.add_argument("--nx", type=int, default=9)
    ap.add_argument("--ny", type=int, default=7)
    ap.add_argument("--nz", type=int, default=5)
    ap.add_argument("--xlo", type=float, default=0.33)
    ap.add_argument("--xhi", type=float, default=0.62)
    ap.add_argument("--ylo", type=float, default=0.42)
    ap.add_argument("--yhi", type=float, default=0.70)
    ap.add_argument("--zlo", type=float, default=0.13)
    ap.add_argument("--zhi", type=float, default=0.30)
    ap.add_argument("--downsample", type=int, default=4)
    ap.add_argument("--json", default=os.path.join(RESULTS, "x4j_criterion.json"))
    args = ap.parse_args()

    st = FL.d11_loc_setup()
    truth = np.asarray(st.occ_ll_true, dtype=float)
    meas, gt = FL.load_measurement(args.mat, args.downsample)
    ds = args.downsample

    gx = np.linspace(args.xlo, args.xhi, args.nx)
    gy = np.linspace(args.ylo, args.yhi, args.ny)
    gz = np.linspace(args.zlo, args.zhi, args.nz)

    print("=" * 76)
    print("x4j 定位判据原型")
    print("=" * 76)
    print("测量  : %s  -> %s（downsamp=%d）" % (os.path.basename(args.mat), meas.shape, ds))
    print("真值  : [%.4f %.4f %.4f]（搜索坐标）" % tuple(truth))
    print("网格  : x %d × y %d × z %d = %d 个候选"
          % (args.nx, args.ny, args.nz, args.nx * args.ny * args.nz))
    print("        x [%.4f..%.4f] step %.4f" % (gx[0], gx[-1], gx[1] - gx[0]))
    print("        y [%.4f..%.4f] step %.4f" % (gy[0], gy[-1], gy[1] - gy[0]))
    print("        z [%.4f..%.4f] step %.4f" % (gz[0], gz[-1], gz[1] - gz[0]))
    # 网格本身能给到的最好结果
    best_pt = np.array([gx[np.argmin(abs(gx - truth[0]))],
                        gy[np.argmin(abs(gy - truth[1]))],
                        gz[np.argmin(abs(gz - truth[2]))]])
    print("        网格上离真值最近的点 = [%.4f %.4f %.4f]，距离 %.6f m"
          % (best_pt[0], best_pt[1], best_pt[2], np.linalg.norm(best_pt - truth)))

    for c, nm in enumerate("rgb"):
        y = meas[:, :, c].reshape(-1, order="F")
        y = y / np.linalg.norm(y)
        sm = FL.build_A_from_plan(FL.make_plan(st, 2, downsamp_factor=ds), truth)
        yp = sm @ np.asarray(gt[:, :, c], dtype=float).reshape(-1, order="F")
        cc = np.corrcoef(yp, y)[0, 1] if np.linalg.norm(yp) > 0 else float("nan")
        print("  通道 %s 与真值模型的相关系数 = %.6f  （判断该通道有无场景信息）" % (nm, cc))

    plan = FL.make_plan(st, 2, downsamp_factor=ds)
    print("\nplan: npx=%d blocks=%d chunk=%d" % (plan.npx, plan.n_blocks, plan.block_chunk))

    shapes = (args.nx, args.ny, args.nz)
    crit = {k: np.zeros(shapes + (3,)) for k in
            ("C0_K1", "C1_s0001", "C2_negfrac", "C3_nnls", "C4_fullrank")}
    K1_all = np.zeros(shapes, dtype=int)

    ys = [meas[:, :, c].reshape(-1, order="F") for c in range(3)]
    y2 = [float(y @ y) for y in ys]

    t0 = time.time()
    n = 0
    for ix in range(args.nx):
        for iy in range(args.ny):
            for iz in range(args.nz):
                p = [gx[ix], gy[iy], gz[iz]]
                A = FL.build_A_from_plan(plan, p)
                G = A.T @ A
                G = 0.5 * (G + G.T)
                lam, V = np.linalg.eigh(G)          # 升序
                lam = np.maximum(lam, 0.0)
                s = np.sqrt(lam)
                s_max = s[-1]
                b_all = [A.T @ y for y in ys]

                # ---- 论文的 K 规则（sigma=0.75）----
                K1 = int(np.sum(s / s_max > 0.75))
                K1_all[ix, iy, iz] = K1

                for c in range(3):
                    b = b_all[c]
                    coef = V.T @ b                  # 长度 1044，顺序与 lam 一致
                    g2 = (coef ** 2) / np.where(lam > 0, lam, np.inf)
                    cum = np.cumsum(g2)             # 升序累加

                    # C0/C1：||U(:,1:K)'y||，即**最大的 K 个** g2 之和再开方。
                    # cum 是按升序 λ 累加的，所以最大 K 个的和 = cum[-1] - cum[n-K-1]。
                    nlam = len(cum)
                    for name, sg in (("C0_K1", 0.75), ("C1_s0001", 1e-3)):
                        K = int(np.sum(s / s_max > sg))
                        if K <= 0:
                            crit[name][ix, iy, iz, c] = 0.0
                        elif K >= nlam:
                            crit[name][ix, iy, iz, c] = np.sqrt(cum[-1])
                        else:
                            crit[name][ix, iy, iz, c] = np.sqrt(cum[-1] - cum[nlam - K - 1])

                    # C4：全秩
                    crit["C4_fullrank"][ix, iy, iz, c] = np.sqrt(cum[-1])

                    # C2：截断最小二乘解的负值占比。Kt 取 s_k/s_max > 0.02
                    Kt = int(np.sum(s / s_max > 0.02))
                    if Kt <= 0:
                        crit["C2_negfrac"][ix, iy, iz, c] = 1.0
                    else:
                        idx = slice(len(lam) - Kt, len(lam))
                        xh = V[:, idx] @ (coef[idx] / lam[idx])
                        nrm = np.abs(xh).sum()
                        crit["C2_negfrac"][ix, iy, iz, c] = \
                            float(np.abs(np.minimum(xh, 0.0)).sum() / nrm) if nrm > 0 else 1.0

                # C3：非负最小二乘残差（只对 r 通道做，bu 场景信息都在 r）
                r = 0
                eta = 1.0 / lam[-1] if lam[-1] > 0 else 1.0
                xx, rr = _nnls(G, b_all[r], y2[r], eta, iters=200)
                crit["C3_nnls"][ix, iy, iz, :] = rr / np.sqrt(y2[r])

                n += 1
                if n % 30 == 0:
                    print("   %d/%d  %.1f s" % (n, args.nx * args.ny * args.nz, time.time() - t0))

    print("全部完成，%d 个候选，用时 %.1f s\n" % (n, time.time() - t0))

    # ---------------------------------------------------------------- 汇总
    def _report(name, surf, maximize=True):
        i, j, k = np.unravel_index(int(np.argmax(surf) if maximize else np.argmin(surf)),
                                   surf.shape)
        p = np.array([gx[i], gy[j], gz[k]])
        d = float(np.linalg.norm(p - truth))
        f = surf.ravel()
        o = np.argsort(f)[::-1] if maximize else np.argsort(f)
        gap = (f[o[0]] - f[o[1]]) / abs(f[o[0]]) if f[o[0]] != 0 else float("nan")
        return p, d, gap

    print("=" * 76)
    print("结果（成功线 ≤ 0.03 m；边缘 ≤ 0.05 m；> 0.05 m 记失败）")
    print("=" * 76)
    hdr = "%-12s %-24s %-12s %-12s %s" % ("判据", "argmax (搜索坐标)", "到真值(m)", "top1/top2 gap", "判定")
    print(hdr)
    print("-" * len(hdr))

    rows = []
    # C0 按论文做法：三通道各自的 argmax 再平均
    p_rgb = np.zeros((3, 3))
    for c in range(3):
        i, j, k = np.unravel_index(int(np.argmax(crit["C0_K1"][..., c])), shapes)
        p_rgb[c] = [gx[i], gy[j], gz[k]]
    p_mean = p_rgb.mean(axis=0)
    d_mean = float(np.linalg.norm(p_mean - truth))
    print("%-12s [%.4f %.4f %.4f]     %-12.6f %-12s %s"
          % ("C0_rgb", p_mean[0], p_mean[1], p_mean[2], d_mean, "-",
             "失败" if d_mean > 0.05 else ("边缘" if d_mean > 0.03 else "成功")))
    rows.append(dict(name="C0_rgb", p=p_mean.tolist(), d=d_mean,
                     per_channel=p_rgb.tolist()))

    for name, maximize, ch in (("C0_K1", True, 0), ("C1_s0001", True, 0),
                               ("C2_negfrac", False, 0), ("C3_nnls", False, 0),
                               ("C4_fullrank", True, 0)):
        surf = crit[name][..., ch]
        p, d, gap = _report(name, surf, maximize)
        verdict = "失败" if d > 0.05 else ("边缘" if d > 0.03 else "成功")
        print("%-12s [%.4f %.4f %.4f]     %-12.6f %-12.5f %s"
              % (name + "_r", p[0], p[1], p[2], d, gap, verdict))
        rows.append(dict(name=name + "_r", p=p.tolist(), d=d, gap=gap, verdict=verdict,
                         maximize=maximize))

    print("\n论文 K 规则的 K 取值分布: %s" % np.unique(K1_all).tolist())

    with open(args.json, "w", encoding="utf-8") as f:
        json.dump(dict(truth=truth.tolist(), grid=dict(
            gx=gx.tolist(), gy=gy.tolist(), gz=gz.tolist()), rows=rows,
            K1_unique=np.unique(K1_all).tolist()), f, ensure_ascii=False, indent=2)
    np.savez_compressed(args.json.replace(".json", "_surf.npz"),
                        **{k: v for k, v in crit.items()}, K1=K1_all)
    print("\n结果: %s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
