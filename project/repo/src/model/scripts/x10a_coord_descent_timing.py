# -*- coding: utf-8 -*-
"""X10a 坐标下降版定位耗时重测。

背景
----
A1 基线（`sim/results/timing_baseline.md`）测的是**官方第 1 轮 18x18x3 = 972 候选
的穷举网格**：单候选 1.0939 s，外推全网格 1007.6 s 约 16.8 分钟。

C0（`sim/results/c0_candidate_reduction.md`）已证明「对 972 网格做间隔采样」不可行：
能量峰极窄，只有 2 个候选达到 top-1 的 90%，任何间隔采样都可能漏峰。

队友在 MATLAB 侧实际采用的不是穷举，而是**坐标下降**：

    第 1 轮：5 x 5 x 5 = 125 个候选（粗网格穷举）
    第 2~4 轮：每轮 30 个点，按 x -> z -> y 依次做一维搜索（各 10 点），
               搜索范围逐轮缩小 4 倍
    合计 125 + 3 x 30 = 215 次候选评估

本脚本把这套调度在主机侧实测一遍，回答三个问题：

  Q1 耗时：215 次评估要多久，相对 972 候选的 16.8 分钟是多少倍。
  Q2 覆盖：5x5x5 粗网格会不会像 C0 担心的那样漏掉真峰——
          即坐标下降最终落点与穷举 top-1、官方 best_pos 各差多少。
  Q3 构成：单候选耗时里 `build_transport_matrix` 占多少，
          即 S0-1 的前缀和路径能消掉多少。

口径声明
--------
1. **单进程、默认 BLAS 线程数**，与 A1 基线同口径，便于直接比较。
   （C0 已实测「10 进程 + BLAS 单线程」可再快 5.9 倍，那是并行口径，
   不用于替换 A1 的串行基线。）
2. 定位通道为 **G 通道**（`meas_g`），与 T0-05 的 `coarse_to_fine` 一致。
3. 候选评估一律走**差分框架**（`differenced=True`），与 T0-04 的结论一致。
4. 调度参数（首轮 5^3、每轮每轴 10 点、缩小 4 倍）取自队友的口头描述，
   脚本内做成参数，若要改调度只需改 CLI。

用法
----
    python src/model/scripts/x10a_coord_descent_timing.py \
        --data-root "D:\\学习\\26秋嵌入式\\论文数据集\\Ordinary-Camera-master\\Data"
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC_MODEL = os.path.dirname(_HERE)
_REPO = os.path.dirname(os.path.dirname(_SRC_MODEL))
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F  # noqa: E402
from penumbrasight_model import io_mat  # noqa: E402
from penumbrasight_model import localization as L  # noqa: E402


# ------------------------------------------------------------------ 单项评估

def evaluate(ll, setup, y2d, kappa):
    """单候选评估，返回 (energy, n_kept, elapsed_s)。"""
    t0 = time.perf_counter()
    r = L.evaluate_candidate(setup, y2d, ll, differenced=True, kappa=kappa)
    return r.energy, r.n_kept, time.perf_counter() - t0


# ------------------------------------------------------------------ 坐标下降

def coord_descent(setup, y2d, box, n_first=5, n_axis=10, n_rounds=4,
                  kappas=(0.75, 0.5, 0.05, 0.05), shrink=4.0,
                  verbose=True):
    """队友口径的四轮坐标下降。

    参数
    ----
    box      : (x0, x1, y0, y1, z0, z1)，各轴搜索区间，取自官方第 1 轮网格的外框
    n_first  : 首轮每轴点数（5 -> 125 个候选）
    n_axis   : 后续每轮每轴点数（10 -> 每轮 30 个候选）
    n_rounds : 总轮数
    shrink   : 每轮范围缩小倍数
    kappas   : 各轮截断阈值

    轴顺序按队友描述为 x -> z -> y（y 为深度），即内部索引 (0, 2, 1)。
    """
    x0, x1, y0, y1, z0, z1 = box
    span1 = np.array([x1 - x0, y1 - y0, z1 - z0], dtype=np.float64)
    n_eval = 0
    trace = []
    t_all = time.perf_counter()

    # ---- 第 1 轮：粗网格穷举 ----
    xv = np.linspace(x0, x1, n_first)
    yv = np.linspace(y0, y1, n_first)
    zv = np.linspace(z0, z1, n_first)
    t0 = time.perf_counter()
    best_ll = None
    best_e = -np.inf
    best_nk = 0
    per_cand = []
    for x in xv:
        for yd in yv:
            for z in zv:
                e, nk, dt = evaluate([x, yd, z], setup, y2d, kappas[0])
                per_cand.append(dt)
                n_eval += 1
                if e > best_e:
                    best_e, best_ll, best_nk = e, np.array([x, yd, z]), nk
    r1 = {"round": 1, "mode": "grid", "n": n_first ** 3,
          "best_ll": best_ll.copy(), "best_energy": best_e,
          "elapsed": time.perf_counter() - t0,
          "cand_mean": float(np.mean(per_cand)),
          "cand_std": float(np.std(per_cand))}
    trace.append(r1)
    if verbose:
        print(f"  [1] 粗网格 {n_first}^3={r1['n']:3d}  最优 "
              f"{np.round(best_ll, 5).tolist()}  E={best_e:.6g}  "
              f"{r1['elapsed']:.1f}s  (单候选 {r1['cand_mean']:.4f}s)", flush=True)

    # ---- 第 2~n_rounds 轮：x -> z -> y 逐轴一维搜索 ----
    order = (0, 2, 1)          # x -> z -> y
    axis_name = {0: "x", 1: "y深度", 2: "z"}
    for k in range(2, n_rounds + 1):
        span = span1 / (shrink ** (k - 1))
        half = span / 2.0
        kap = kappas[min(k - 1, len(kappas) - 1)]
        t0 = time.perf_counter()
        detail = []
        per_cand = []
        for ax in order:
            lo = best_ll[ax] - half[ax]
            hi = best_ll[ax] + half[ax]
            vals = np.linspace(lo, hi, n_axis)
            ax_best_v = best_ll[ax]
            ax_best_e = best_e
            for v in vals:
                ll = best_ll.copy()
                ll[ax] = v
                e, nk, dt = evaluate(ll, setup, y2d, kap)
                per_cand.append(dt)
                n_eval += 1
                if e > ax_best_e:
                    ax_best_e, ax_best_v, best_nk = e, v, nk
            best_ll[ax] = ax_best_v
            best_e = ax_best_e
            detail.append({"axis": axis_name[ax], "step": float(span[ax] / (n_axis - 1)),
                           "picked": float(ax_best_v)})
        rk = {"round": k, "mode": "coordinate", "n": 3 * n_axis,
              "kappa": kap, "span": span.tolist(),
              "best_ll": best_ll.copy(), "best_energy": best_e,
              "elapsed": time.perf_counter() - t0,
              "cand_mean": float(np.mean(per_cand)),
              "detail": detail}
        trace.append(rk)
        if verbose:
            sp = np.round(span, 5).tolist()
            print(f"  [{k}] 坐标下降 {3 * n_axis} 点  范围 {sp}  "
                  f"最优 {np.round(best_ll, 5).tolist()}  E={best_e:.6g}  "
                  f"{rk['elapsed']:.1f}s", flush=True)

    total = time.perf_counter() - t_all
    return {"n_eval": n_eval, "total": total, "best_ll": best_ll,
            "best_energy": best_e, "best_n_kept": best_nk, "trace": trace}


# ------------------------------------------------------------------ 单候选分节

def stage_breakdown(setup, y2d, ll, n=5):
    """复测单候选耗时的构成，用于给出「消掉几何重建后」的估计。

    分节口径与 A1-3 一致：build_transport_matrix / 差分 A_d / A_d^T y /
    A_d^T A_d / 特征分解 / 能量求和。
    """
    rows = []
    for k in range(n):
        # 轻微扰动位置，避免缓存命中
        l = np.asarray(ll, dtype=np.float64) + (k - n // 2) * 1e-4
        s2 = setup.with_occluder_ll(l)
        t = time.perf_counter()
        A = F.build_transport_matrix(s2)
        t_build = time.perf_counter() - t

        t = time.perf_counter()
        Ad = L.apply_row_difference_A(A, setup.wall.n)
        t_diff = time.perf_counter() - t

        t = time.perf_counter()
        y = L.apply_row_difference_img(y2d)
        z = Ad.T @ y
        t_aty = time.perf_counter() - t

        t = time.perf_counter()
        G = Ad.T @ Ad
        t_gram = time.perf_counter() - t

        t = time.perf_counter()
        lam, V = np.linalg.eigh(0.5 * (G + G.T))
        t_eigh = time.perf_counter() - t

        t = time.perf_counter()
        lam = np.maximum(lam[::-1], 0.0)
        s = np.sqrt(lam)
        keep = s >= 0.75 * s[0]
        c = V[:, ::-1].T @ z
        _ = float(np.sum((c[keep] ** 2) / lam[keep]))
        t_sum = time.perf_counter() - t

        rows.append({"build": t_build, "diff": t_diff, "aty": t_aty,
                     "gram": t_gram, "eigh": t_eigh, "sum": t_sum,
                     "total": t_build + t_diff + t_aty + t_gram + t_eigh + t_sum})
        del A, Ad, G, V
    return rows


# ------------------------------------------------------------------ 主流程

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default=os.path.join(_REPO, "sim", "results"))
    ap.add_argument("--vectors", default=os.path.join(_REPO, "sim", "vectors"))
    ap.add_argument("--n-first", type=int, default=5)
    ap.add_argument("--n-axis", type=int, default=10)
    ap.add_argument("--n-rounds", type=int, default=4)
    ap.add_argument("--shrink", type=float, default=4.0)
    ap.add_argument("--brk-n", type=int, default=5)
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    print(f"[info] 配置：{cfg}", flush=True)
    print(f"[info] numpy BLAS 线程：OMP={os.environ.get('OMP_NUM_THREADS', '(默认)')} "
          f"MKL={os.environ.get('MKL_NUM_THREADS', '(默认)')}", flush=True)

    setup = F.load_setup(cfg)
    d = io_mat.load_mat(cfg)
    y2d = np.asarray(d["meas_g"], dtype=np.float64)
    print(f"[info] 测量 G 通道形状 {y2d.shape}  墙网格 n={setup.wall.n}", flush=True)

    # 官方第 1 轮搜索外框
    grids = L.load_search_grids(cfg)
    r1x, r1y, r1z = grids["r1_x"], grids["r1_y"], grids["r1_z"]
    box = (float(r1x.min()), float(r1x.max()),
           float(r1y.min()), float(r1y.max()),
           float(r1z.min()), float(r1z.max()))
    print(f"[info] 官方第 1 轮网格 {r1x.size}x{r1y.size}x{r1z.size}，"
          f"外框 x[{box[0]},{box[1]}] y[{box[2]},{box[3]}] z[{box[4]},{box[5]}]",
          flush=True)

    # 对照量
    official_best = np.asarray(d["best_pos"], dtype=np.float64).reshape(-1)
    gt = np.asarray(d["Occluder"], dtype=np.float64).reshape(-1)[:3]
    ref = None
    vpath = os.path.join(args.vectors, "c0_field_r1.npz")
    if os.path.isfile(vpath):
        cv = np.load(vpath)
        pos, val = cv["pos"], cv["val"]
        i = int(np.argmax(val))
        ref = {"top1": pos[i].copy(), "energy": float(val[i]),
               "n_candidates": int(pos.shape[0])}
        print(f"[info] 穷举基准（C0 存档）：top-1 {np.round(ref['top1'], 5).tolist()}  "
              f"E={ref['energy']:.6g}", flush=True)

    # ---- Q1/Q2：坐标下降实测 ----
    print("\n=== 坐标下降定位（单进程，默认 BLAS） ===", flush=True)
    res = coord_descent(setup, y2d, box, n_first=args.n_first,
                        n_axis=args.n_axis, n_rounds=args.n_rounds,
                        shrink=args.shrink)

    dist_ref = float(np.linalg.norm(res["best_ll"] - ref["top1"])) if ref else None
    dist_off = float(np.linalg.norm(res["best_ll"] - official_best))
    dist_gt = float(np.linalg.norm(res["best_ll"] - gt))

    print(f"\n  评估次数 {res['n_eval']}  总耗时 {res['total']:.1f}s "
          f"({res['total']/60:.2f} min)", flush=True)
    print(f"  落点 {np.round(res['best_ll'], 5).tolist()}  E={res['best_energy']:.6g}", flush=True)
    if dist_ref is not None:
        print(f"  与穷举 top-1 距离 {dist_ref:.5f} m", flush=True)
    print(f"  与官方 best_pos 距离 {dist_off:.5f} m   与真值距离 {dist_gt:.5f} m", flush=True)

    # ---- Q3：单候选分节 ----
    print("\n=== 单候选分节复测（Q3） ===", flush=True)
    rows = stage_breakdown(setup, y2d, res["best_ll"], n=args.brk_n)
    agg = {k: float(np.mean([r[k] for r in rows]))
           for k in ("build", "diff", "aty", "gram", "eigh", "sum", "total")}
    for k, label in (("build", "build_transport_matrix"), ("diff", "差分 A_d"),
                     ("aty", "A_d^T y"), ("gram", "A_d^T A_d"),
                     ("eigh", "特征分解 eigh"), ("sum", "能量求和")):
        print(f"  {label:24s} {agg[k]*1e3:8.1f} ms  {agg[k]/agg['total']*100:5.1f}%",
              flush=True)
    print(f"  {'合计':24s} {agg['total']*1e3:8.1f} ms", flush=True)

    # ---- 汇总 ----
    A1_cand = 1.0939                 # A1 基线单候选
    A1_full = 1007.6                 # A1 基线 972 候选外推
    out = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"),
        "box": box,
        "official_round1_grid": [int(r1x.size), int(r1y.size), int(r1z.size)],
        "schedule": {"n_first": args.n_first, "n_axis": args.n_axis,
                     "n_rounds": args.n_rounds, "shrink": args.shrink,
                     "axis_order": ["x", "z", "y"]},
        "coord_descent": {
            "n_eval": res["n_eval"], "total_s": res["total"],
            "best_ll": res["best_ll"].tolist(), "best_energy": res["best_energy"],
            "best_n_kept": int(res["best_n_kept"]),
            "dist_to_exhaustive_top1_m": dist_ref,
            "dist_to_official_best_pos_m": dist_off,
            "dist_to_ground_truth_m": dist_gt,
            "trace": [{**{k: v for k, v in r.items() if k != "best_ll"},
                       "best_ll": r["best_ll"].tolist()} for r in res["trace"]],
        },
        "exhaustive_reference": ({**{k: v for k, v in ref.items() if k != "top1"},
                                  "top1": ref["top1"].tolist()} if ref else None),
        "stage_breakdown_ms": {k: v * 1e3 for k, v in agg.items()},
        "a1_baseline": {"single_candidate_s": A1_cand, "grid972_s": A1_full},
        "speedup_vs_grid972": A1_full / res["total"],
        "optimized_estimate": {
            "note": "消掉 build_transport_matrix（S0-1 前缀和路径）后的估计",
            "no_build_s_per_candidate": agg["total"] - agg["build"],
            "coord_descent_s": res["n_eval"] * (agg["total"] - agg["build"]),
        },
    }

    os.makedirs(args.out_dir, exist_ok=True)
    with io.open(os.path.join(args.out_dir, "x10a_coord_descent_timing.json"),
                 "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2, default=float)
    np.savez_compressed(
        os.path.join(args.out_dir, "x10a_coord_descent_trace.npz"),
        **{f"r{r['round']}_ll": r["best_ll"] for r in res["trace"]},
        **{f"r{r['round']}_E": np.array(r["best_energy"]) for r in res["trace"]})
    print(f"\n[done] 总耗时 {res['total']:.1f}s，结果已写入 "
          f"{args.out_dir}/x10a_coord_descent_timing.json", flush=True)


if __name__ == "__main__":
    main()
