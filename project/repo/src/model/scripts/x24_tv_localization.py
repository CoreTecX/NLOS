# -*- coding: utf-8 -*-
"""X24 —— 定位判据能不能用上「空间排布」：TV 耦合判据的第一轮判定

## 为什么试这个

x4k / x4l 已把问题推到「差分域的前向模型保真度」，最好 0.0438 m（43.8 mm），
方案 §9 的目标是 ≤ 20 mm。但在此之前有一条**结构性**结论一直没被利用：

> 论文的判据 Σ_c ‖U_{p,K}ᵀ d_c‖² 对隐藏场景的**空间排布完全不敏感**。
> 任何像素索引重排都是置换 P，`B → BP = UΣ(PᵀV)ᵀ` 仍是合法 SVD，
> **左奇异向量与奇异值逐点不变**，判据逐点不变。

实测佐证：x4l 的 `x_reversed` 开关给出**逐位相同**的结果。

后果：该判据在原理上只能利用「被点亮的那一组块」的**集合**，
用不到任何一块的**位置**。而遮挡物位置信息恰恰编码在场景的空间结构里，
这条通路被整条丢掉了 —— 这就是「oracle 行、判据不行」的另一种说法。

**突破口**：方案 §2 自己指定的正则项是 `λ·TV(f)`。`TV(f) = Σ|∇f|`
**在空间上耦合相邻像素**，所以它**不是**置换不变的。也就是说：
只要把定位判据从「子空间投影」换成「带 TV 的拟合目标」，
判据就重新获得了使用位置信息的能力。

这一点可证、可测，而且**正是老师方案里的东西**（不是自研附加）。

## 本轮的预登记（跑之前写死）

判据分两段，先验机制、再看能不能定位：

- **M1（机制）**：存在一个块标号置换 P，使
  (a) 子空间判据 `E_sub(B)` 与 `E_sub(BP)` 相对差 **< 1e-12**（证明不变性）；
  (b) TV 目标 `J_tv(B)` 与 `J_tv(BP)` 相对差 **> 1e-3**（证明它确实不一样）。
- **M2（判别力）**：在真值网格点 + 6 个由近及远的诱饵上，
  `J_tv` 的最小值落在**离真值一个网格步长之内**。
- **M3（对照）**：同一批候选上，子空间判据 `E_sub` 的真值落后量仍然很差
  （预期 −40% 量级，复现 x4k 的结论）。

**若 M1 成立而 M2 不成立**：TV 打破了不变性但没用上可用信息（可能是 λ 标度不对，
或差分域信噪比太低）—— 那就不该继续在判据上投入，应转 X4（模型保真度）。

**若 M1 自身不成立**：说明我对置换不变性的推导有错，要回去查推导。

λ 取 4 档（1e-4 / 1e-3 / 1e-2 / 1e-1，B 与 d 已按 Frobenius 范数归一），
**跑之前不定哪档"应该"最好** —— 四档全报。

## 用法

    cd src/model && python scripts/x24_tv_localization.py --scene bu
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

from penumbrasight_model import forward_loc as FL          # noqa: E402
from penumbrasight_model.reconstruction import tv_pdhg      # noqa: E402

DATA_ROOT = r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master\Data\TestPosD11"
SCENES = {
    "bu":       "image_test_bur20.mat",
    "mushroom": "image_test_mushroom20.mat",
}

# 与 x4k 完全相同的候选网格（保证结论可比）
GRID = dict(nx=9, ny=7, nz=5, xlo=0.33, xhi=0.62,
            ylo=0.42, yhi=0.70, zlo=0.13, zhi=0.30)
NMON = 2
DS   = 4
K    = 8              # 方案冻结值
LAM_LIST = (1e-4, 1e-3, 1e-2, 1e-1)
TV_ITERS = 250


def diff_gram_1d(n: int) -> np.ndarray:
    """DᵀD，D 是 n 阶一阶相邻差分（与 x4k 同一实现）。"""
    G = np.zeros((n, n))
    idx = np.arange(n)
    G[idx, idx] = 1.0
    G[idx[:-1], idx[:-1]] += 1.0
    G[idx[:-1] - 1, idx[:-1]] = -1.0
    G[idx[:-1], idx[:-1] - 1] = -1.0
    return G


def fc_perm(nrow: int, ncol: int) -> np.ndarray:
    """把「A 的列序（F 序）」换成「tv_pdhg 期望的 C 序」。

    x4k 用 `gt.reshape(-1, order="F")` 对齐 A 的列，且 oracle 命中，
    所以列序 = F 序已被实测确定。tv_pdhg 内部按 C 序 reshape，
    故需把 B 的列按 B[:, perm] 重排，使 `B_C @ x_C == B @ x_F`。
    """
    return np.arange(nrow * ncol).reshape(nrow, ncol, order="F").ravel(order="C")


def subspace_scores(B: np.ndarray, D3: np.ndarray, k: int = K) -> np.ndarray:
    """Σ_c ‖U_{:,1:k}ᵀ d_c‖²，用右奇异向量等价式算（避免对 3906×1044 做 SVD）。

    B = UΣVᵀ  =>  U_k = B V_k Σ_k⁻¹  =>  ‖U_kᵀ d‖² = Σ_{j<=k} (v_jᵀ Bᵀd)² / σ_j²
    """
    G = B.T @ B
    lam, V = np.linalg.eigh(G)
    lam, V = lam[::-1], V[:, ::-1]
    s2 = np.maximum(lam, 0.0)
    out = np.zeros(D3.shape[1])
    keep = min(k, s2.size)
    for c in range(D3.shape[1]):
        bd = B.T @ D3[:, c]
        p = V[:, :keep].T @ bd
        out[c] = float(np.sum(np.where(s2[:keep] > 0, p * p / np.maximum(s2[:keep], 1e-300), 0.0)))
    return out


def tv_objective(B: np.ndarray, d3: np.ndarray, shp, lam: float):
    """Σ_c [ ½‖B f_c − d_c‖² + λ TV(f_c) ]，f ≥ 0。

    B 与 d 已归一化，故 λ 是无量纲的。返回 (总目标, 逐通道目标, 逐通道残差)。
    """
    tot, per, res = 0.0, [], []
    for c in range(d3.shape[1]):
        r = tv_pdhg(B, d3[:, c], shp, lam=lam, n_iter=TV_ITERS,
                    nonneg=True, L2=None, record_every=TV_ITERS)
        o = r.objective[-1]
        per.append(float(o["obj"]))
        res.append(float(o["data"]))
        tot += float(o["obj"])
    return tot, per, res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="bu", choices=sorted(SCENES))
    ap.add_argument("--data-root", default=DATA_ROOT)
    ap.add_argument("--mz-mirror", action=argparse.BooleanOptionalAction, default=False,
                    help="默认 False（X4l 实测拟合更好的一侧）")
    ap.add_argument("--n-decoys", type=int, default=6)
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    st = FL.d11_loc_setup()
    truth = np.asarray(st.occ_ll_true, dtype=float)
    mat = os.path.join(args.data_root, SCENES[args.scene])
    meas, gt = FL.load_measurement(mat, DS)

    npx = FL.num_pixels_for(st, DS)
    nrow, ncol = st.num_blocks
    nb = nrow * ncol
    shp = (nrow, ncol)

    gx = np.linspace(GRID["xlo"], GRID["xhi"], GRID["nx"])
    gy = np.linspace(GRID["ylo"], GRID["yhi"], GRID["ny"])
    gz = np.linspace(GRID["zlo"], GRID["zhi"], GRID["nz"])
    bp = np.array([gx[np.argmin(abs(gx - truth[0]))],
                   gy[np.argmin(abs(gy - truth[1]))],
                   gz[np.argmin(abs(gz - truth[2]))]])
    floor = float(np.linalg.norm(bp - truth))

    print("=" * 78)
    print("X24  TV 耦合判据的第一轮判定    场景 = %s" % args.scene)
    print("=" * 78)
    print("真值      : [%.4f %.4f %.4f]  最近网格点 [%.4f %.4f %.4f]  物理地板 %.5f m"
          % (*truth, *bp, floor))
    print("网格步长  : %.4f / %.4f / %.4f" % (gx[1] - gx[0], gy[1] - gy[0], gz[1] - gz[0]))
    print("口径      : 差分 D_y（方案 §2）  K=%d  Ndiscr_mon=%d  ds=%d  mz_mirrored=%s"
          % (K, NMON, DS, args.mz_mirror))
    print()

    # ---- 测量侧：d = L_z y -------------------------------------------
    Lz = diff_gram_1d(npx)
    Y3 = np.stack([np.ascontiguousarray(meas[:, :, c].T) for c in range(3)], axis=-1)
    DZ = np.ascontiguousarray(np.einsum("ibc,ab->iac", Y3, Lz)).reshape(-1, 3)

    # ---- 候选表：真值 + 由近及远的诱饵 ---------------------------------
    cand = [("truth", bp)]
    rng = np.random.default_rng(20260926)
    steps = np.linspace(0.6, 6.0, args.n_decoys)     # 以网格步长为单位
    base = np.array([gx[1] - gx[0], gy[1] - gy[0], gz[1] - gz[0]])
    for i, s in enumerate(steps):
        for _ in range(40):
            dirv = rng.normal(size=3)
            dirv /= np.linalg.norm(dirv)
            q = bp + dirv * s * base
            if (gx[0] <= q[0] <= gx[-1] and gy[0] <= q[1] <= gy[-1] and gz[0] <= q[2] <= gz[-1]):
                cand.append(("decoy%d_%.1fstep" % (i, s), q))
                break

    plan = FL.make_plan(st, NMON, downsamp_factor=DS, mz_mirrored=args.mz_mirror)
    perm = fc_perm(nrow, ncol)
    print("plan: npx=%d blocks=%d  mz_mirrored=%s" % (plan.npx, plan.n_blocks, plan.mz_mirrored))
    print()

    # ==================================================================
    # M1 机制：置换是否改变判据
    # ==================================================================
    print("=" * 78)
    print("M1 机制判定：块标号置换下，两个判据各自变不变")
    print("=" * 78)
    A0 = FL.build_A_from_plan(plan, bp)
    B0 = np.ascontiguousarray(
        np.einsum("ibk,ab->iak", A0.reshape(npx, npx, nb), Lz)).reshape(npx * npx, nb)
    Bn = B0 / (np.linalg.norm(B0) + 1e-300)
    Dn = DZ / (np.linalg.norm(DZ) + 1e-300)

    e_sub0 = subspace_scores(Bn, Dn, K).sum()
    j_tv0 = tv_objective(Bn[:, perm], Dn, shp, LAM_LIST[1])[0]

    rp = np.random.default_rng(7).permutation(nb)
    Bp = np.ascontiguousarray(Bn[:, rp])
    e_sub1 = subspace_scores(Bp, Dn, K).sum()
    j_tv1 = tv_objective(Bp[:, perm], Dn, shp, LAM_LIST[1])[0]

    d_sub = abs(e_sub1 - e_sub0) / max(abs(e_sub0), 1e-300)
    d_tv = abs(j_tv1 - j_tv0) / max(abs(j_tv0), 1e-300)
    print("  子空间判据 E_sub : %.6e → %.6e   相对差 %.3e" % (e_sub0, e_sub1, d_sub))
    print("  TV 目标    J_tv  : %.6e → %.6e   相对差 %.3e" % (j_tv0, j_tv1, d_tv))
    m1a = d_sub < 1e-12
    m1b = d_tv > 1e-3
    print("  M1a 子空间判据置换不变（<1e-12）      : %s" % ("成立 ✓" if m1a else "不成立 ✗"))
    print("  M1b TV 目标置换敏感   （>1e-3）      : %s" % ("成立 ✓" if m1b else "不成立 ✗"))

    # ---- 收敛性检查：负结果的强度取决于此 ---------------------------------
    # λ≥1e-3 时 TV 项占主导，若 250 次迭代没收敛，那几个目标值就只是「迭代轨
    # 迹上的一个点」，不能用来断言判据的判别力。先把这件事量清楚。
    print()
    print("  收敛性检查（真值位置，λ=%.0e，目标值随迭代）：" % LAM_LIST[1])
    for lam_ck in (LAM_LIST[0], LAM_LIST[1]):
        rr = tv_pdhg(Bn[:, perm], Dn[:, 0], shp, lam=lam_ck, n_iter=TV_ITERS,
                     nonneg=True, L2=None, record_every=50)
        hist = [float(o["obj"]) for o in rr.objective]
        rel = abs(hist[-1] - hist[-2]) / max(abs(hist[-1]), 1e-300) if len(hist) > 1 else 1.0
        print("    λ=%.0e  " % lam_ck + "  ".join("%.4e" % v for v in hist)
              + "   末两次变化 %.1e  %s"
              % (rel, "已收敛" if rel < 1e-4 else "未收敛"))
    print()

    # ==================================================================
    # M2 / M3 判别力
    # ==================================================================
    print("=" * 78)
    print("M2/M3 判别力：真值 + %d 个诱饵" % (len(cand) - 1))
    print("=" * 78)
    rows = []
    for name, p in cand:
        A = FL.build_A_from_plan(plan, p)
        B = np.ascontiguousarray(
            np.einsum("ibk,ab->iak", A.reshape(npx, npx, nb), Lz)).reshape(npx * npx, nb)
        Bnn = B / (np.linalg.norm(B) + 1e-300)

        e_sub = subspace_scores(Bnn, Dn, K)
        lam_tot = {}
        for lam in LAM_LIST:
            lam_tot[lam] = tv_objective(Bnn[:, perm], Dn, shp, lam)[0]

        dist = float(np.linalg.norm(p - truth))
        rows.append(dict(name=name, dist=dist, p=[float(v) for v in p],
                         e_sub=float(e_sub.sum()),
                         esub_rgb=[float(v) for v in e_sub],
                         tv={("%.0e" % l): v for l, v in lam_tot.items()}))
        print("  %-16s 距真值 %6.4f m  E_sub %.6e  J_tv %s"
              % (name, dist, e_sub.sum(),
                 "  ".join("%.0e:%.5f" % (l, v) for l, v in lam_tot.items())))

    # 真值落后量：真值点比全候选最优差多少（负 = 真值更差）
    e_arr = np.array([r["e_sub"] for r in rows])
    lag_sub = (e_arr[0] - e_arr.max()) / abs(e_arr.max()) if e_arr.max() != 0 else 0.0
    print()
    print("  子空间判据真值落后量 = %.2f pct（负 = 真值不是最优；x4k 同网格量到 -43 pct）"
          % (100.0 * lag_sub))

    m2 = {}
    for lam in LAM_LIST:
        j = np.array([r["tv"]["%.0e" % lam] for r in rows])
        arg = int(np.argmin(j))
        best_in_grid = min(cand, key=lambda cp: np.linalg.norm(cp[1] - truth))
        lag = (j[0] - j.min()) / abs(j.min()) if j.min() != 0 else 0.0
        ok = abs(rows[arg]["dist"] - floor) <= (base.max() * 1.01)
        m2[lam] = dict(argmin=rows[arg]["name"], argmin_dist=rows[arg]["dist"],
                       truth_lag=float(lag), within_one_step=bool(ok))
        print("  λ=%.0e  argmin = %-16s 距真值 %6.4f m   真值落后量 %+7.2f%%   %s"
              % (lam, rows[arg]["name"], rows[arg]["dist"], 100.0 * lag,
                 "✓ 落在地板内" if ok else "✗"))

    print()
    print("=" * 78)
    print("判定汇总")
    print("=" * 78)
    print("  M1a 置换不变性（子空间）  : %s" % ("✓" if m1a else "✗"))
    print("  M1b 置换敏感性（TV）      : %s" % ("✓" if m1b else "✗"))
    print("  M2  TV 能定位到地板       : %s" % ("✓" if any(v["within_one_step"] for v in m2.values()) else "✗"))
    print("  M3  子空间判据仍差        : %s（落后 %.1f%%）"
          % ("✓" if lag_sub < -0.05 else "✗", 100.0 * lag_sub))

    out = dict(scene=args.scene, mz_mirrored=args.mz_mirror, floor=floor,
               grid=GRID, K=K, lam_list=list(LAM_LIST), tv_iters=TV_ITERS,
               m1=dict(e_sub_0=e_sub0, e_sub_1=e_sub1, d_sub=d_sub,
                       j_tv_0=j_tv0, j_tv_1=j_tv1, d_tv=d_tv),
               m2={("%.0e" % l): v for l, v in m2.items()},
               truth_lag_sub=float(lag_sub), rows=rows)
    tag = args.tag or ("x24_tv_%s" % args.scene)
    with open(os.path.join(HERE, "..", "..", "..", "sim", "results", tag + ".json"), "w",
              encoding="utf-8") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print("\n已保存 sim/results/%s.json" % tag)


if __name__ == "__main__":
    t0 = time.time()
    main()
    print("用时 %.1f s" % (time.time() - t0))
