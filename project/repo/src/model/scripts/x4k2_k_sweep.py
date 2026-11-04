# -*- coding: utf-8 -*-
"""x4k2 —— 用 x4k 留下的投影缓存做 K 扫描与加权扫描（**不再建任何 A**）。

为什么可以这样省
----------------
`x4k` 在跑那 315 个候选时把每个候选的
    `lamv[op, ix, iy, iz, k]` = `λ_k`（`G = BᵀB` 的特征值，降序）
    `proj[op, ix, iy, iz, c, k]` = `v_kᵀ b_c`（`b_c = Aᵀ·(c 通道的差分测量)`）
全存了下来（各 64 阶，`op ∈ {none, z, x}`）。于是

    `||U(:,1:K)ᵀ d||² = Σ_{k≤K} (v_kᵀ b)² / λ_k`

对**任意 K、任意通道合成、任意奇异值加权**都是同一批缓存上的**线性组合**，
事后重算一次不到一秒。这正是 X11 立的规矩「早停轨迹 = 长轨迹的前缀」的用法：
**不必为每个 K 重跑一遍 502 秒。**

要回答的问题
------------
`x4k` 的主结果：方案口径 `T1a_diff_K8` 落在 **0.1081 m**，比论文原口径还差。
而差分后的谱是**一片高台**（`s_k/s_1` 前 12 个都 > 0.77，σ=0.75 规则的 K 涨到 13）。
所以这一轮问三件事：

1. **有没有任何一个 K 能定位？**（K ∈ 1…64）
   —— 若没有，说明「位置信息不在前 K 个左奇异方向的张成里」，与 K 取值无关。
2. **是 λ 加权的问题吗？** 换掉 `1/λ`（改成不加权、或反向加权 `λ`）后 argmax 动没动。
3. **方向选对了吗？** 垂直（`z`）vs 水平（`x`）vs 不差分（`none`），同一张表比。

预登记
------
- 若 `K` 扫描在某个 K 上给出 ≤ 0.03 m → 方案口径**只是 K 取错了**，改常数即可；
- 若整条曲线都在 0.08 m 以上乱跳 → 问题不在 K，**在「位置信息是否落在 B 的前 K 个左奇异子空间里」**；
- 换加权若把 argmax 拉回真值附近 → 说明病根是 `1/λ` 对小奇异值的放大（这恰好是 x4j 在
  未差分域量到过的「A_c(p) 的 1044 维列空间过拟合」）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, os.pardir))
REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")

OP_NAME = {"none": "未差分 y", "z": "垂直差分 D_y y", "x": "水平差分 D_x y"}


def sub_energy(proj, lam, K, weight):
    """`Σ_{k<K} f(λ_k)·(v_kᵀb)²`（对通道求和），按 `weight` 取 f。

    weight='inv'  -> 1/λ   （子空间能量 / 截断最小二乘的补充量，**方案口径**）
    weight='one'  -> 1     （不做奇异值加权）
    weight='lam'  -> λ     （反向加权）

    `proj` 形状 `shp+(3,K)`、`lam` 形状 `shp+(K,)`；返回 `shp+(3,)`。
    """
    p = proj[..., :K]                        # shp + (3, K)
    l = lam[..., :K][..., None, :]           # shp + (1, K)
    if weight == "inv":
        return (p ** 2 / np.maximum(l, 1e-300)).sum(axis=-1)
    if weight == "one":
        return (p ** 2).sum(axis=-1)
    return (p ** 2 * l).sum(axis=-1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", default=os.path.join(RESULTS, "x4k_diff_bu_surf.npz"))
    ap.add_argument("--out", default="")
    ap.add_argument("--ks",
                    default="1,2,3,4,5,6,8,10,13,16,20,24,32,48,64")
    args = ap.parse_args()

    d = np.load(args.npz, allow_pickle=False)
    ops = str(d["ops"]).split(",")
    gx, gy, gz = d["gx"], d["gy"], d["gz"]
    truth = d["truth"]
    shp = (len(gx), len(gy), len(gz))
    lamv, proj = d["lamv"], d["proj"]            # (3,)+shp+（64,) / (3,)+shp+(3,64)
    Ks = [int(v) for v in args.ks.split(",")]

    ti = tuple(int(np.argmin(abs(g - t))) for g, t in zip((gx, gy, gz), truth))
    floor = float(np.linalg.norm(
        np.array([gx[ti[0]], gy[ti[1]], gz[ti[2]]]) - truth))

    print("=" * 84)
    print("x4k2  K 扫描与加权扫描   缓存 = %s" % os.path.basename(args.npz))
    print("=" * 84)
    print("真值 [%.4f %.4f %.4f]   物理地板 %.6f m   网格 %d×%d×%d"
          % (truth[0], truth[1], truth[2], floor, *shp))
    print("算子：%s" % " / ".join("%s=%s" % (o, OP_NAME.get(o, o)) for o in ops))
    print()

    WEIGHTS = [("inv", "1/λ（方案口径）"), ("one", "不加权"), ("lam", "λ（反加权）")]
    res = {}
    for oi, op in enumerate(ops):
        lam = lamv[oi]
        # 谱高台有多宽：s_k/s_1 > 0.75 的个数（在缓存 64 阶内）
        s = np.sqrt(np.maximum(lam, 0))
        kil = (s / np.maximum(s[..., :1], 1e-300) > 0.75).sum(axis=-1)
        print("-" * 84)
        print("算子 %-14s   σ=0.75 规则的 K：中位 %d  范围 %d~%d"
              % (OP_NAME.get(op, op), int(np.median(kil)), int(kil.min()), int(kil.max())))
        print("-" * 84)
        print("%-22s %s" % ("K", "  ".join("%-16s" % w[1] for w in WEIGHTS)))
        for K in Ks:
            cell = []
            for wn, _ in WEIGHTS:
                surf = sub_energy(proj[oi], lam, K, wn)
                surf_ch = surf.sum(axis=-1)               # 三通道求和（方案 §2 口径）
                i = np.unravel_index(int(np.argmax(surf_ch)), shp)
                p = np.array([gx[i[0]], gy[i[1]], gz[i[2]]])
                dist = float(np.linalg.norm(p - truth))
                cell.append("%.4f m [%.2f,%.2f,%.2f]" % (dist, p[0], p[1], p[2]))
                res.setdefault(op, {}).setdefault(wn, {})[str(K)] = dict(
                    dist=dist, p=p.tolist())
            print("%-22s %s" % ("K=%d" % K, "  ".join("%-16s" % c for c in cell)))
        print()

    # ---- 三个算子在同一 K 上的横向对比（方案口径 1/λ）----
    print("=" * 84)
    print("横向对比（1/λ 加权、三通道求和）—— 方向选对了吗")
    print("=" * 84)
    print("%-8s %s" % ("K", "  ".join("%-16s" % OP_NAME.get(o, o) for o in ops)))
    for K in Ks:
        cell = []
        for oi, op in enumerate(ops):
            surf = sub_energy(proj[oi], lamv[oi], K, "inv").sum(axis=-1)
            i = np.unravel_index(int(np.argmax(surf)), shp)
            p = np.array([gx[i[0]], gy[i[1]], gz[i[2]]])
            cell.append("%.4f m" % np.linalg.norm(p - truth))
        print("%-8s %s" % ("K=%d" % K, "  ".join("%-16s" % c for c in cell)))

    out = args.out or args.npz.replace("_surf.npz", "_ksweep.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(dict(npz=os.path.basename(args.npz), truth=truth.tolist(),
                       floor=floor, ks=Ks, results=res), f,
                  ensure_ascii=False, indent=2)
    print("\n结果: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
