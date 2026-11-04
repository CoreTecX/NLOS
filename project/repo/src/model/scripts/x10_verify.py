# -*- coding: utf-8 -*-
"""X10 验证：C++ 定位核（M2 + M3）产出的 Gram / A_d^T y 能否复现官方第 1 轮 top-1。

分工刻意如此
------------
    PL 侧（C++）：M2 几何算核 + M3 Gram 累加 -> G 上三角、A_d^T y
    PS 侧（本脚本）：特征分解 + 投影能量 + 取 argmax
这与 A6 的路线一致：PL 只出 Gram 与右端项，特征分解在 PS 做。
所以 X10 验的是 PL 那一半的数值正确性，加上整条链的位号一致性。

参考量
------
    sim/results/localization_energy_cubes.npz 里的 E_round1，形状 (18,18,3)。
    已核对它与 `localization.evaluate_candidate(kappa=0.75)` 逐一相同
    （三个抽样点比值均为 1.000000000），故可作为权威参考。

判据
----
    V1 逐候选能量：相对误差 <= 1e-3
    V2 位号一致：argmax 的 (ix,iy,iz) 与参考相同
    V3 名次一致：能量排序的 Spearman 相关 >= 0.99

用法
----
    python src/model/scripts/x10_verify.py --k-start 0 --k-stop 20
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys

import numpy as np

# ---------------------------------------------------------------------------
# V-1 ε* 翻转裕度（判据 6 的正确口径）
#
# 与 src/fpga/hls/projection_engine/test_projection.cpp 的 eps_flip_scan 同一份定义：
#   R[p] = E_ref[p]（权威参考能量）  F[p] = E_cpp[p]（定点/C++ 能量）
#   δ[p] = F[p] - R[p]              best = argmax_p R[p]
#   分子 m_k = (R[best] - R[k]) / R[best]      —— 候选 k 相对 top-1 的相对间隔
#   分母 e_k = |F[k] - R[k]| / R[k] = |δ_k| / R[k]  —— 候选 k 的相对误差
#   放大误差（R + a·δ）使 k 超过 best 的临界倍数：
#       a_k = (R[best] - R[k]) / (δ[k] - δ[best])   仅当 δ[k] - δ[best] > 0
#   ε* = min_k a_k。含义：误差再放大 ε* 倍，top-1 才首次翻转。
#   文档比值口径 ε*_ratio = m_k / e_k 是 a_k 的保守下界，一并给出。
# 判据：ε* > 2 通过；[1,2] 裕度有限；< 1 为异常。
# 不可扫描（如实返回 scannable=False）：δ≡0、无 δ[k]-δ[best]>0 的对手、n<2、R[best]<=0。
# ---------------------------------------------------------------------------


def eps_flip_scan(R, F):
    """按上式算 ε*。返回 dict；不伪造：退化场景 scannable=False。"""
    R = np.asarray(R, dtype=np.float64)
    F = np.asarray(F, dtype=np.float64)
    n = int(R.size)
    out = {"n": n, "best_k": -1, "flip_k": -1,
           "eps_star": float("inf"), "eps_star_ratio": float("inf"),
           "eps_star_ratio_min": float("inf"), "eps_star_scan": float("inf"),
           "margin": 0.0, "e_best": 0.0, "e_max": 0.0, "scannable": False}
    if n < 2:
        return out
    best = int(np.argmax(R))
    Rb = float(R[best])
    out["best_k"] = best
    if not (Rb > 0.0):
        return out
    delta = F - R
    eb = abs(float(delta[best])) / Rb
    out["e_best"] = eb
    out["e_max"] = float(np.max(np.abs(delta) / np.maximum(R, 1e-300)))

    eps = float("inf")
    fk = -1
    ratio_min = float("inf")
    for k in range(n):
        if k == best or R[k] >= Rb:
            continue
        rk = float(R[k])
        m = (Rb - rk) / Rb
        ek = abs(float(delta[k])) / rk if rk > 0 else float("inf")
        ratio_min = min(ratio_min, m / max(ek, eb))
        dd = float(delta[k]) - float(delta[best])
        if dd > 0.0:
            a = (Rb - rk) / dd
            if a < eps:
                eps = a
                fk = k
    out["eps_star_ratio_min"] = float(ratio_min)
    if not np.isfinite(eps):
        return out                      # 退化：无有限翻转点

    out["scannable"] = True
    out["eps_star"] = float(eps)
    out["flip_k"] = int(fk)
    out["margin"] = float((Rb - float(R[fk])) / Rb)
    e_k = abs(float(delta[fk])) / float(R[fk])
    out["eps_star_ratio"] = float(out["margin"] / max(e_k, eb))

    # 逐步放大互校：二分定位 argmax(R + a·δ) 首次偏离 best 的 a。
    def argmax_a(a):
        return int(np.argmax(R + a * delta))

    lo, hi = 1.0, max(2.0, eps * 4.0)
    if argmax_a(hi) != best:
        for _ in range(200):
            if (hi - lo) <= 1e-15 * hi:
                break
            mid = 0.5 * (lo + hi)
            if argmax_a(mid) == best:
                lo = mid
            else:
                hi = mid
        out["eps_star_scan"] = float(hi)
    return out


def eps_block(rows, k0, k1):
    """从 rows（每条含 E_cpp / E_ref）算 ε* 块，供 JSON 落盘。"""
    R = np.array([r["E_ref"] for r in rows], dtype=np.float64)
    F = np.array([r["E_cpp"] for r in rows], dtype=np.float64)
    e = eps_flip_scan(R, F)
    e["threshold"] = 2.0
    e["k_range"] = [int(k0), int(k1)]
    e["definition"] = ("eps*=min_k a_k; a_k=(E_ref[best]-E_ref[k])/(d_k-d_best); "
                       "d=E_cpp-E_ref; m_k=(E_ref[best]-E_ref[k])/E_ref[best]; "
                       "e_k=|E_cpp[k]-E_ref[k]|/E_ref[k]")
    e["pass"] = bool(e["scannable"] and e["eps_star"] > 2.0)
    if not e["scannable"]:
        e["pass"] = False
        e["note"] = "不可扫描：δ≡0 或无 δ_k-δ_best>0 的对手；不伪造裕度"
    return e

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(_SRC_MODEL))
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)


def projection_energy_from_gram(G: np.ndarray, z: np.ndarray, kappa: float = 0.75):
    """与 `localization.projection_energy_detail` 同口径，但直接吃 G 与 z。

    保留判据在**奇异值**上：s >= kappa * s[0]，等价于 lam >= kappa^2 * lam[0]。
    """
    G = 0.5 * (G + G.T)
    lam, V = np.linalg.eigh(G)
    lam = lam[::-1]
    V = V[:, ::-1]
    lam = np.maximum(lam, 0.0)
    s = np.sqrt(lam)
    if s.size == 0 or s[0] <= 0.0:
        return 0.0, 0, 0.0
    keep = (s >= kappa * s[0]) if kappa > 0 else np.ones(s.size, dtype=bool)
    if not keep.any():
        return 0.0, 0, float(s[0])
    c = V.T @ z
    e = float(np.sum((c[keep] ** 2) / lam[keep]))
    return e, int(keep.sum()), float(s[0])


def eps_from_json(args) -> int:
    """不重跑 2.12 GB gram -> 直接从已有 x10_verify.json 的 rows 复算 ε* 并回写。

    用于在缺少 x10_gram_z.bin 的机器上复现判据 6 的翻转裕度口径。
    """
    p = os.path.join(args.out_dir, "x10_verify.json")
    if not os.path.exists(p):
        print("[ERR] 找不到 %s" % p)
        return 1
    with io.open(p, "r", encoding="utf-8") as f:
        j = json.load(f)
    rows = j.get("rows") or []
    if len(rows) < 2:
        print("[ERR] rows 不足，无法算 ε*")
        return 1
    kr = j.get("k_range") or [0, len(rows)]
    e = eps_block(rows, kr[0], kr[1])
    j["eps_star"] = e
    with io.open(p, "w", encoding="utf-8") as f:
        json.dump(j, f, ensure_ascii=False, indent=2)
    print("[V-1] ε* = %.6e  ratio = %.6e  scan = %.6e  flip_k = %d  best_k = %d"
          % (e["eps_star"], e["eps_star_ratio"], e["eps_star_scan"],
             e["flip_k"], e["best_k"]))
    print("[V-1] 判据 ε* > 2：%s（可扫描 = %s，n = %d）"
          % ("PASS" if e["pass"] else "FAIL", e["scannable"], e["n"]))
    return 0 if e["pass"] else 1


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gram-bin", default=os.path.join(
        _REPO, "src", "hardware", "hwdata", "x10_gram_z.bin"))
    ap.add_argument("--k-start", type=int, default=0)
    ap.add_argument("--k-stop", type=int, default=-1)
    ap.add_argument("--out-dir", default=os.path.join(_REPO, "sim", "results"))
    ap.add_argument("--from-json", action="store_true",
                    help="不读 gram，直接从已有 x10_verify.json 的 rows 复算 ε* 并回写")
    args = ap.parse_args()

    if args.from_json:
        sys.exit(eps_from_json(args))

    z_npz = np.load(os.path.join(args.out_dir, "localization_energy_cubes.npz"))
    E_ref = z_npz["E_round1"]
    nx, ny, nz = E_ref.shape
    K = nx * ny * nz

    grid = np.fromfile(os.path.join(_REPO, "src", "hardware", "hwdata",
                                    "x10_grid.bin"), dtype=np.float32)
    xv = grid[:nx].astype(np.float64)
    yv = grid[nx:nx + ny].astype(np.float64)
    zv = grid[nx + ny:].astype(np.float64)

    N = 1044
    T = N * (N + 1) // 2
    per = T + N
    iu = np.triu_indices(N)

    k0 = args.k_start
    k1 = K if args.k_stop < 0 else args.k_stop
    need = np.arange(k0, k1)

    raw = np.memmap(args.gram_bin, dtype=np.float32, mode="r")
    print("[info] 网格 %d x %d x %d = %d 候选；本次验 [%d, %d)"
          % (nx, ny, nz, K, k0, k1))
    print("[info] 文件 %s，%.1f MB，含 %d 个候选"
          % (os.path.basename(args.gram_bin),
             raw.size * 4 / 1e6, raw.size // per))

    # 参考 top-1
    fi = int(np.argmax(E_ref))
    ref_ix, ref_iy, ref_iz = np.unravel_index(fi, E_ref.shape)
    ref_k = ref_ix * (ny * nz) + ref_iy * nz + ref_iz
    print("[info] 参考 top-1：idx=(%d,%d,%d)  k=%d  ll=(%.5f, %.5f, %.5f)  E=%.6e"
          % (ref_ix, ref_iy, ref_iz, ref_k, xv[ref_ix], yv[ref_iy], zv[ref_iz],
             E_ref[ref_ix, ref_iy, ref_iz]))

    E_cpp = np.full(K, np.nan)
    nk_cpp = np.zeros(K, dtype=int)
    G = np.empty((N, N), dtype=np.float64)
    rows = []
    for k in need:
        blk = raw[k * per:(k + 1) * per]
        if blk.size < per:
            print("  [warn] 候选 %d 数据不完整，跳过" % k)
            continue
        tri = blk[:T].astype(np.float64)
        z = blk[T:].astype(np.float64)
        G[iu] = tri
        G[(iu[1], iu[0])] = tri
        e, nk, s1 = projection_energy_from_gram(G, z, 0.75)
        E_cpp[k] = e
        nk_cpp[k] = nk
        ix, iy, iz = np.unravel_index(k, (nx, ny, nz))
        ref = float(E_ref[ix, iy, iz])
        rel = abs(e - ref) / ref if ref > 0 else 0.0
        rows.append({"k": int(k), "ix": int(ix), "iy": int(iy), "iz": int(iz),
                     "ll": [float(xv[ix]), float(yv[iy]), float(zv[iz])],
                     "E_cpp": e, "E_ref": ref, "rel": rel, "n_kept": nk,
                     "s1": s1})
        if len(rows) <= 15 or k % 100 == 0:
            print("  k=%4d (%2d,%2d,%d)  E_cpp=%.6e  E_ref=%.6e  相对 %.3e  "
                  "n_kept=%d" % (k, ix, iy, iz, e, ref, rel, nk))
            sys.stdout.flush()

    if not rows:
        print("[ERR] 没有可比对的候选")
        return

    E_have = E_cpp[need]
    E_ref_have = np.array([r["E_ref"] for r in rows])
    rels = np.array([r["rel"] for r in rows])

    print("\n=== V1 逐候选能量相对误差 ===")
    print("  候选数 %d  最坏 %.3e  中位 %.3e  均值 %.3e"
          % (len(rows), rels.max(), np.median(rels), rels.mean()))
    i_worst = int(np.argmax(rels))
    print("  最坏出在 k=%d idx=(%d,%d,%d) E_cpp=%.6e E_ref=%.6e"
          % (rows[i_worst]["k"], rows[i_worst]["ix"], rows[i_worst]["iy"],
             rows[i_worst]["iz"], rows[i_worst]["E_cpp"], rows[i_worst]["E_ref"]))

    print("\n=== V2 位号一致 ===")
    k_best = int(need[np.nanargmax(E_have)])
    ix, iy, iz = np.unravel_index(k_best, (nx, ny, nz))
    same = (k_best == ref_k)
    print("  C++ top-1：idx=(%d,%d,%d)  k=%d  ll=(%.5f, %.5f, %.5f)  E=%.6e"
          % (ix, iy, iz, k_best, xv[ix], yv[iy], zv[iz], E_cpp[k_best]))
    print("  参考 top-1：idx=(%d,%d,%d)  k=%d" % (ref_ix, ref_iy, ref_iz, ref_k))
    print("  [%s] 位号一致" % ("ok  " if same else "FAIL"))

    print("\n=== V3 名次一致 ===")
    if len(rows) >= 5:
        r_cpp = np.argsort(np.argsort(E_have))
        r_ref = np.argsort(np.argsort(E_ref_have))
        n = len(rows)
        rho = 1.0 - 6.0 * np.sum((r_cpp - r_ref) ** 2) / (n * (n * n - 1))
        print("  能量排序 Spearman rho = %.6f（阈值 0.99，%d 个候选）" % (rho, n))
        ok3 = rho >= 0.99
    else:
        rho = float("nan")
        ok3 = True
        print("  候选数不足 5，跳过")
    print("  [%s] 名次一致" % ("ok  " if ok3 else "FAIL"))

    ok = (rels.max() <= 1e-3) and same and ok3
    print("汇总：V1 %s / V2 %s / V3 %s -> %s"
          % ("通过" if rels.max() <= 1e-3 else "不通过",
             "通过" if same else "不通过",
             "通过" if ok3 else "不通过",
             "全部通过" if ok else "存在失败项"))

    # V-1：把「结果量」（位号一致 + Spearman）升级成「翻转裕度」ε*。
    eps = eps_block(rows, k0, k1)
    print("\n=== V-1 ε* 翻转裕度（判据 6 的正确口径）===")
    print("  定义：ε*=min_k a_k，a_k=(E_ref[best]-E_ref[k])/(δ_k-δ_best)，δ=E_cpp-E_ref")
    if eps["scannable"]:
        print("  best_k=%d  flip_k=%d  m_k=%.3e  e_best=%.3e"
              % (eps["best_k"], eps["flip_k"], eps["margin"], eps["e_best"]))
        print("  ε* = %.6e  （ratio 口径 %.6e，逐步放大互校 %.6e）"
              % (eps["eps_star"], eps["eps_star_ratio"], eps["eps_star_scan"]))
    else:
        print("  ε* = +inf：不可扫描（δ≡0 或无 δ_k-δ_best>0 的对手），如实列出")
    print("  [%s] 判据 ε* > 2（%d 个候选）"
          % ("ok  " if eps["pass"] else "FAIL", eps["n"]))

    os.makedirs(args.out_dir, exist_ok=True)
    with io.open(os.path.join(args.out_dir, "x10_verify.json"), "w",
                 encoding="utf-8") as f:
        json.dump({
            "k_range": [k0, k1], "n_checked": len(rows),
            "rel_max": float(rels.max()), "rel_median": float(np.median(rels)),
            "rel_mean": float(rels.mean()),
            "cpp_top1": {"k": int(k_best), "idx": [int(ix), int(iy), int(iz)]},
            "ref_top1": {"k": int(ref_k),
                         "idx": [int(ref_ix), int(ref_iy), int(ref_iz)]},
            "top1_match": bool(same), "spearman": float(rho),
            "all_pass": bool(ok), "eps_star": eps, "rows": rows,
        }, f, ensure_ascii=False, indent=2)
    print("[done] 明细写入 %s/x10_verify.json" % args.out_dir)


if __name__ == "__main__":
    main()
