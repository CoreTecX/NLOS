# -*- coding: utf-8 -*-
"""x4l —— 模型开关消融：−43% 真值落后量是从哪来的？

T1（x4k）留下的问题
------------------
方案口径 `D_y` + K=8 下，真值点的能量**稳定落后全网格最优 42~44%**，跨场景几乎不变。
两个可能的来源：

  (i)  **模型失配**：`forward_loc` 里那些「刻意保留原代码特征」的某个开关让差分域
       的预测与实测系统性对不上；
  (ii) **估计器病态**：模型其实没问题，是「场景未知」这件事让判据失效。

`x4k` 的 oracle 已经把这两者分开了，本轮要把它钉死：

| | bu | mushroom |
|---|---|---|
| oracle（**已知**场景真值）差分残差在真值点是否最优 | **是**（0% 落后） | **不是**（落后 18.27%） |
| 方案口径（**未知**场景） | 落后 43.63% | 落后 42.02% |

**所以预登记是这样的**：bu 上 oracle 已经最优，**任何开关都不可能把 bu 的 oracle 变得更好**
（只能变差）；bu 那 43% 只能是 (ii)。mushroom 才是 (i) 的目标 —— 那里 oracle 本身就没命中。

若实测与此不符（比如某个开关把 bu 的 oracle 落后量从 0% 变成负的、或把 mushroom 的
18.27% 拉回 0%），那说明我对 x4k 的读法是错的，报告要改。

本轮要测的开关
--------------
| 开关 | 忠实值 | 翻转后 | 为什么怀疑它 |
|---|---|---|---|
| `mz_mirrored` | True | False | 视角因子内部用**降序**的 FOV 网格乘**升序**的墙网格，等效把视角因子当成 z 的镜像在用（x4h §3 第 6 条） |
| `stick` | True | False | 另有一根独立立柱，与矩形阴影用 **OR** 合并；重建链那边是逐元素乘积 |
| `view_angle` | True | False | 整个视角因子项开/关 |
| `x_reversed` | True | False | 块→显示器 x 序号反向（源码注释承认是文件命名约定造成的） |
| `Ndiscr_mon` | 2 | 1 / 10 | 整屏全局离散的采样密度；方案 `table_S1` 全局轮用 2，y 精细轮用 10 |

度量
----
每个配置 × 每个场景，在同一组参考点上算三样：

  1. **未差分 oracle 残差**（对照，应当一直很好；变差 = 该开关是错的）
  2. **差分 oracle 残差** ← 模型失配的直接读数，越小越好
  3. **方案口径子空间能量**（K=8、垂直差分、1/λ、三通道求和）← 交付指标

参考点 = 真值最近网格点 + 它的 6 个面邻居 + x4k 里各判据的 argmax（去重）。
这样既能看「模型在真值处贴得多近」，也能看「真值还算不算最好」。

成本控制：**不跑满 315 网格**。先在 ~10 个参考点上扫；只有当某个配置明显动了指针，
才升级到满网格复核。
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
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from penumbrasight_model import forward_loc as FL          # noqa: E402
import x4k_diff_localization as X4K                        # noqa: E402

REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")
DATA_ROOT = r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master\Data\TestPosD11"
SCENES = {"bu": "image_test_bur20.mat", "mushroom": "image_test_mushroom20.mat"}

NX, NY, NZ = 9, 7, 5
XLO, XHI = 0.33, 0.62
YLO, YHI = 0.42, 0.70
ZLO, ZHI = 0.13, 0.30
K = 8
_TOPK = 16

# (名字, make_plan 关键字, build_A 关键字)
CONFIGS = [
    ("base",            {},                                          {}),
    ("no_mz_mirror",    dict(mz_mirrored=False),                     {}),
    ("no_stick",        {},                                          dict(stick=False)),
    ("no_view_angle",   {},                                          dict(view_angle=False)),
    ("no_x_rev",        dict(x_reversed=False),                      {}),
    ("N1",              dict(nmon=1),                                {}),
    ("N10",             dict(nmon=10),                               {}),
    ("no_mz_no_stick",  dict(mz_mirrored=False),                     dict(stick=False)),
    ("bare",            dict(mz_mirrored=False, x_reversed=False),   dict(stick=False, view_angle=False)),
]


def _relres(a, b):
    """最优尺度下的相对残差 `min_α ||b − αa|| / ||b||`（闭式）。"""
    na2, nb2 = float(a @ a), float(b @ b)
    if nb2 <= 0 or na2 <= 0:
        return 1.0
    return float(np.sqrt(max(0.0, 1.0 - (a @ b) ** 2 / (na2 * nb2))))


def _corr(a, b):
    na2, nb2 = float(a @ a), float(b @ b)
    if na2 <= 0 or nb2 <= 0:
        return -1.0
    return float((a @ b) / np.sqrt(na2 * nb2))


def build_refs(gx, gy, gz, truth, x4k_npz):
    """真值最近点 + 6 个面邻居 + x4k 各判据的 argmax（去重）。返回 [(label, idx)]。"""
    ti = tuple(int(np.argmin(abs(g - t))) for g, t in zip((gx, gy, gz), truth))
    out, seen = [], set()

    def add(lab, idx):
        if all(0 <= idx[a] < (len(gx), len(gy), len(gz))[a] for a in range(3)) \
                and idx not in seen:
            seen.add(idx)
            out.append((lab, idx))

    add("truth", ti)
    for a, nm in enumerate("xyz"):
        for d in (-1, +1):
            j = list(ti)
            j[a] += d
            add("truth%+d%s" % (d, nm), tuple(j))
    for key in ("surf_T1a_diff_K8", "surf_T1f_diffx_K8", "surf_B0_paper_K1"):
        if key in x4k_npz:
            s = x4k_npz[key]
            if s.ndim == 4:
                s = s.sum(axis=-1)
            add(key.replace("surf_", "") + "_argmax", np.unravel_index(int(np.argmax(s)), s.shape))
    return out, ti


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="bu", choices=sorted(SCENES))
    ap.add_argument("--data-root", default=DATA_ROOT)
    ap.add_argument("--downsample", type=int, default=4)
    ap.add_argument("--only", default="", help="只跑这些配置（逗号分隔）")
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    sc = args.scene
    ds = args.downsample
    st = FL.d11_loc_setup()
    truth = np.asarray(st.occ_ll_true, dtype=float)
    meas, gt = FL.load_measurement(os.path.join(args.data_root, SCENES[sc]), ds)
    npx = FL.num_pixels_for(st, ds)
    nb = st.num_blocks[0] * st.num_blocks[1]

    gx = np.linspace(XLO, XHI, NX)
    gy = np.linspace(YLO, YHI, NY)
    gz = np.linspace(ZLO, ZHI, NZ)

    x4k_npz = dict(np.load(os.path.join(RESULTS, "x4k_diff_%s_surf.npz" % sc),
                           allow_pickle=False))
    refs, ti = build_refs(gx, gy, gz, truth, x4k_npz)
    truth_pt = (gx[ti[0]], gy[ti[1]], gz[ti[2]])
    floor = float(np.linalg.norm(np.array(truth_pt) - truth))

    Lz = X4K.diff_gram_1d(npx)
    Y3 = np.stack([np.ascontiguousarray(meas[:, :, c].T) for c in range(3)], axis=-1)
    YZ = np.ascontiguousarray(np.einsum("ibc,ab->iac", Y3, Lz)).reshape(-1, 3)
    Y0 = Y3.reshape(-1, 3)
    Y3r = Y3[:, :, 0]
    dnorm2 = np.array([float(Y0[:, c] @ YZ[:, c]) for c in range(3)])
    xgt = np.stack([np.asarray(gt[:, :, c], dtype=float).reshape(-1, order="F")
                    for c in range(3)], axis=-1)

    print("=" * 92)
    print("x4l  模型开关消融   场景 = %s" % sc)
    print("=" * 92)
    print("真值 [%.4f %.4f %.4f]   真值最近网格点 [%.4f %.4f %.4f]  物理地板 %.6f m"
          % (truth[0], truth[1], truth[2], truth_pt[0], truth_pt[1], truth_pt[2], floor))
    print("参考点 %d 个：%s" % (len(refs), ", ".join(lab for lab, _ in refs)))
    print()

    cfgs = CONFIGS
    if args.only:
        want = set(args.only.split(","))
        cfgs = [c for c in CONFIGS if c[0] in want]

    ALL = {}
    for name, plan_kw, A_kw in cfgs:
        nmon = plan_kw.get("nmon", 2)
        pk = {k: v for k, v in plan_kw.items() if k != "nmon"}
        t0 = time.time()
        plan = FL.make_plan(st, nmon, downsamp_factor=ds, **pk)
        rows = {}
        for lab, idx in refs:
            p = [gx[idx[0]], gy[idx[1]], gz[idx[2]]]
            A = FL.build_A_from_plan(plan, p, **A_kw)
            A3 = A.reshape(npx, npx, nb)
            Ar = A3.reshape(npx * npx, nb)
            yp0 = Ar @ xgt[:, 0]

            # 1) 未差分 oracle
            r0 = _relres(yp0, Y0[:, 0])
            c0 = _corr(yp0, Y0[:, 0])
            # 2) 差分 oracle
            dp = np.diff(yp0.reshape(npx, npx), axis=1).ravel()
            dm = np.diff(Y3r, axis=1).ravel()
            r1 = _relres(dp, dm)
            c1 = _corr(dp, dm)
            # 3) 方案口径子空间能量
            Az = np.ascontiguousarray(
                np.einsum("ibj,ab->iaj", A3, Lz)).reshape(npx * npx, nb)
            Gz = Az.T @ Ar
            Gz = 0.5 * (Gz + Gz.T)
            bz = Ar.T @ YZ
            lam, V = X4K.topk_eigh(Gz, _TOPK)
            sE = sum(X4K.sub_energy(lam, V, bz[:, c], K) for c in range(3))
            rows[lab] = dict(p=p, r0=r0, c0=c0, r1=r1, c1=c1, sE=sE,
                             dist=float(np.linalg.norm(np.array(p) - truth)))
        ALL[name] = rows
        print("  %-16s %5.1f s（nmon=%d）" % (name, time.time() - t0, nmon))

    # ==================================================================
    print()
    print("=" * 92)
    print("度量 1：模型在真值处贴得多近（**差分 oracle 相对残差**，越小 = 模型越对）")
    print("=" * 92)
    print("%-16s %-16s %-18s %-14s %s"
          % ("配置", "未差分残差@真值", "差分残差最小(参考点)", "差分残差@真值", "对照 x4k"))
    print("-" * 92)
    x4k_truth_r1 = float(x4k_npz["or_relres_diff"][ti]) if "or_relres_diff" in x4k_npz else float("nan")
    for name, _, _ in cfgs:
        rows = ALL[name]
        r0t, r1t = rows["truth"]["r0"], rows["truth"]["r1"]
        tag = ""
        if name == "base":
            tag = "x4k 存值 %.6f" % x4k_truth_r1
        print("%-16s %-12.6f %-12.6f %-14.6f %s"
              % (name, r0t, min(v["r1"] for v in rows.values()), r1t, tag))

    print()
    print("=" * 92)
    print("度量 2：真值在这组参考点里排第几")
    print("=" * 92)
    print("%-16s %-34s %-34s" % ("配置",
                                 "差分 oracle 残差（小=好）", "方案口径能量（大=好）"))
    print("%-16s %-34s %-34s" % ("", "真值排名 / 落后量", "真值排名 / 落后量"))
    print("-" * 92)
    for name, _, _ in cfgs:
        rows = ALL[name]
        out = []
        for key, minimize in (("r1", True), ("sE", False)):
            vals = sorted(((v[key], k) for k, v in rows.items()),
                          reverse=not minimize)
            best = vals[0][0]
            rank = [k for _, k in vals].index("truth") + 1
            lag = (rows["truth"][key] - best) / abs(best) if best else float("nan")
            out.append("%d / %d   %+7.2f%%" % (rank, len(rows), 100 * lag))
        print("%-16s %-34s %-34s" % (name, out[0], out[1]))

    print()
    print("=" * 92)
    print("回归校验：base 配置必须与 x4k 存值一致")
    print("=" * 92)
    if "or_relres_diff" in x4k_npz and "base" in ALL:
        got = ALL["base"]["truth"]["r1"]
        print("  x4k 存的真值点差分 oracle 残差 = %.6f" % x4k_truth_r1)
        print("  本轮 base 重算                = %.6f" % got)
        print("  -> 相对差 %.3e  %s"
              % (abs(got - x4k_truth_r1) / max(abs(x4k_truth_r1), 1e-300),
                 "✅ 默认路径未被改动" if abs(got - x4k_truth_r1) < 1e-9
                 else "⚠️ 不一致，先查开关默认值"))

    out = os.path.join(RESULTS, args.tag or ("x4l_switch_%s.json" % sc))
    with open(out, "w", encoding="utf-8") as f:
        json.dump(dict(scene=sc, truth=truth.tolist(), truth_pt=list(truth_pt),
                       floor=floor, refs=[l for l, _ in refs],
                       results={k: {kk: vv for kk, vv in v.items()}
                                for k, v in ALL.items()}), f,
                  ensure_ascii=False, indent=2)
    print("\n结果: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
