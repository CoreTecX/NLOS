# -*- coding: utf-8 -*-
"""X30 / T7 —— 去掉「z 镜像」后，定位链前向模型与**论文自己保存的能量数组**差多少。

任务（台账 T7）
--------------
用同一套口径，把 `mz_mirrored=True`（忠实）与 `mz_mirrored=False`（去镜像）两套定位链
前向模型，分别与论文保存的定位中间结果逐值比对，量化偏差，据此决定设计报告里写
「忠实复现」还是「修正 + 分栏单列」。

数据（论文自己的存档，不依赖任何测量加载猜测）
--------------------------------------------
`Data/TestPosChair/bur_opt_{1st,2nd,3rd}_it_pos.mat` 是官方算法运行的完整中间状态存档：
  * `meas_r/g/b`（126x126）—— 官方那次运行实际吃进去的三通道测量
  * `xhatvals/yhatvals/zhatvals`（+`2`/`3` 后缀）—— 各轮搜索网格
  * `Ndiscr_mon` / `downsamp_factor` / `FOV_cord` / `Mon_Offset` /
    `IlluminationBlock_Size` / `Occ_size` / `NumBlocks_sim` —— 官方前向模型参数
  * `threshold` + `thresh`/`thresh_2`/`thresh_3` —— 每轮实际用的归一化奇异值阈值
  * `proj_yOnRangeA{,_g,_b}`（18,18,3,2）、`proj_yOnRangeA_2/_3`（6,6,6,11）
    —— 官方各候选取值 `||U(:,1:K)' y||`，最后一维是阈值扫描下标
  * `best_pos` / `best_pos_2` / `best_pos_3` —— 官方各轮位置估计

于是「前向模型对不对」可以完全脱离重建链、脱离测量加载器来判：**同一测量、同一网格、
同一 Ndiscr_mon、同一 sigma 规则**，逐候选比 `||U(:,1:K)' y||`。

预登记成功线（跑之前写死，跑完不许改）
--------------------------------------
P1  忠实（mz_mirrored=True）与论文数组：最大相对偏差应 <= 2%
    （x4i §2.2 记录 12 点抽样落在 0.986~1.009，即 <= 1.4%）。
P2  去镜像（mz_mirrored=False）与论文数组：
      * 最大相对偏差 <= 2%  -> 「仍是同一模型」，可与论文数值逐值对齐；
      * 最大相对偏差 >= 10% -> 必须按方案 §2 末条把「去镜像」作为**修正**分栏单列；
      * 2% ~ 10%            -> 中间地带，按中位偏差与符号是否系统来定，并如实标注。
P3  去镜像若改变 `s2/s1`（x4l 记 D11/ds=4 上 0.185 -> 0.283），一并列出，作为「换了模型」
    的旁证。

判定口径
--------
相对偏差 = |模型能量 / 论文能量 - 1|，逐候选、逐通道算，再取最大值与中位数。
「最大相对偏差」对离群点敏感，所以同时给中位数；两者一起看。

约束
----
* 只使用 `forward_loc.py` 的既有开关（`mz_mirrored`），**不改其数学定义**。
* 不碰 `src/fpga/`、`src/software/`、`tests/vectors/`、`configs/`、
  `run_localization_reconstruction.py`。

用法
----
    python x30_t7_mirror_deviation.py                 # 全网格三轮 + 两变体
    python x30_t7_mirror_deviation.py --workers 12
    python x30_t7_mirror_deviation.py --limit 24      # 自检（每轮每变体抽样 24 点）
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np
import scipy.io as sio

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, os.pardir))
if _MODEL_DIR not in sys.path:
    sys.path.insert(0, _MODEL_DIR)
from penumbrasight_model import forward_loc as FL   # noqa: E402

REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")
DEFAULT_DATA = (r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master"
                r"\Data\TestPosChair")

VARIANTS = {"faithful": True, "no_mirror": False}     # 变体名 -> mz_mirrored
ROUND_SPECS = [
    # (轮次, 网格后缀, 阈值下标属性名, 论文能量数组后缀)
    (1, "", "thresh", ""),
    (2, "2", "thresh_2", "_2"),
    (3, "3", "thresh_3", "_3"),
]

_LOG = []


def say(msg=""):
    print(msg, flush=True)
    _LOG.append(msg)


# ----------------------------------------------------------------------
def load_paper(data_dir):
    """读三轮存档，拼出每轮的网格 / sigma / 论文能量 / best_pos。"""
    rounds = []
    for r, suf, thr_key, e_suf in ROUND_SPECS:
        mat = os.path.join(data_dir, "bur_opt_%s_it_pos.mat"
                           % {1: "1st", 2: "2nd", 3: "3rd"}[r])
        d = sio.loadmat(mat)
        thr_list = np.asarray(d["threshold"], dtype=float).ravel()
        ti = int(np.asarray(d[thr_key]).ravel()[0])
        sigma = float(thr_list[ti - 1])
        grids = [np.asarray(d["xhatvals" + suf], float).ravel(),
                 np.asarray(d["yhatvals" + suf], float).ravel(),
                 np.asarray(d["zhatvals" + suf], float).ravel()]
        E = np.stack([np.asarray(d["proj_yOnRangeA" + e_suf], float),
                      np.asarray(d["proj_yOnRangeA_g" + e_suf], float),
                      np.asarray(d["proj_yOnRangeA_b" + e_suf], float)], axis=-1)
        rounds.append(dict(
            round=r, mat=os.path.basename(mat), sigma=sigma, thr_index=ti - 1,
            grids=grids, E_paper=E[..., ti - 1, :] if E.ndim == 5 else E,
            best_pos=np.asarray(
                d["best_pos" + ("_" + suf if suf else "")], float).ravel(),
        ))
    d1 = sio.loadmat(os.path.join(data_dir, "bur_opt_1st_it_pos.mat"))
    try:
        npr = int(np.asarray(d1["simuParams"]["numPixels"][0, 0]).ravel()[0])
    except Exception:                                     # noqa: BLE001
        npr = 1008
    setup = FL.LocSetup(
        num_blocks=tuple(int(v) for v in np.asarray(d1["NumBlocks_sim"], float).ravel()),
        num_pixels_raw=npr,
        D=float(np.asarray(d1["D"], float).ravel()[0]),
        fov_cord=np.asarray(d1["FOV_cord"], float).reshape(2, 2),
        illum_block_size=np.asarray(d1["IlluminationBlock_Size"], float).ravel(),
        mon_offset=np.asarray(d1["Mon_Offset"], float).ravel(),
        occ_size=np.asarray(d1["Occ_size"], float).ravel(),
        occ_ll_true=np.asarray(d1["true"], float).ravel(),
    )
    meas = np.stack([np.asarray(d1["meas_r"], float),
                     np.asarray(d1["meas_g"], float),
                     np.asarray(d1["meas_b"], float)], axis=-1)
    n_discr = int(np.asarray(d1["Ndiscr_mon"], float).ravel()[0])
    ds = int(np.asarray(d1["downsamp_factor"], float).ravel()[0])
    meta = dict(num_blocks=setup.num_blocks, num_pixels_raw=setup.num_pixels_raw,
                D=setup.D, fov_cord=setup.fov_cord.tolist(),
                illum_block_size=setup.illum_block_size.tolist(),
                mon_offset=setup.mon_offset.tolist(),
                occ_size=setup.occ_size.tolist(), n_discr_mon=n_discr,
                downsamp_factor=ds, npx=FL.num_pixels_for(setup, ds),
                meas_mean=float(meas.mean()))
    return setup, meas, meta, rounds


# ---- 并行 worker 状态 -------------------------------------------------
_G = {}


def _init_worker(setup, meas, n_discr, ds, npx):
    _G["setup"] = setup
    _G["meas"] = meas
    _G["npx"] = npx
    _G["ch"] = [meas[..., c].reshape(-1) for c in range(3)]
    _G["plan"] = {}
    for name, mir in VARIANTS.items():
        _G["plan"][name] = FL.make_plan(setup, n_discr, downsamp_factor=ds,
                                        mz_mirrored=mir)


def _task(args):
    """返回 (轮次, 变体, i, j, k, e_r, e_g, e_b, K, s2_over_s1)。"""
    rnd, vname, i, j, k, x, y, z, sigma = args
    plan = _G["plan"][vname]
    A = FL.build_A_from_plan(plan, [x, y, z], view_angle=True, stick=True)
    G = A.T @ A
    lam, V = np.linalg.eigh(0.5 * (G + G.T))
    s = np.sqrt(np.maximum(lam[::-1], 0.0))
    Vd = V[:, ::-1]
    npass = int(np.sum(s / s[0] > sigma))
    K = min(npass, 10)
    e = [0.0, 0.0, 0.0]
    if K > 0:
        for c in range(3):
            e[c] = float(np.linalg.norm(Vd[:, :K].T @ (A.T @ _G["ch"][c]) / s[:K]))
    s21 = float(s[1] / s[0]) if s.size > 1 else 0.0
    return (rnd, vname, i, j, k, e[0], e[1], e[2], K, s21)


# ----------------------------------------------------------------------
def run(workers, limit, data_dir, log_path, json_path, npz_path):
    t0 = time.time()
    setup, meas, meta, rounds = load_paper(data_dir)
    say("=" * 84)
    say("X30 / T7  去镜像 vs 论文保存能量数组    %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    say("=" * 84)
    say("前向模型 : src/model/penumbrasight_model/forward_loc.py（未改数学定义，只用开关）")
    say("论文存档 : %s" % data_dir)
    say("Chair 档 : NumBlocks=%s  npx=%d（ds=%d, numPixels_raw=%d）  Ndiscr_mon=%d"
        % (meta["num_blocks"], meta["npx"], meta["downsamp_factor"],
           meta["num_pixels_raw"], meta["n_discr_mon"]))
    say("           FOV_cord=%s  Mon_Offset=%s  illum=%s  Occ_size=%s"
        % (meta["fov_cord"], meta["mon_offset"], meta["illum_block_size"], meta["occ_size"]))
    say("测量     : 论文存档 meas_r/g/b（126x126x3），mean=%.6e" % meta["meas_mean"])
    say("并行     : %d workers  %s" % (workers, "（自检 limit=%d）" % limit if limit else ""))
    say("")
    say("预登记成功线（跑前写死）：")
    say("  P1 忠实 vs 论文数组 最大相对偏差 <= 2%")
    say("  P2 去镜像 vs 论文数组 最大相对偏差 <=2% 记『同一模型』；>=10% 记『必须分栏单列』")
    say("  P3 列出两种变体的 s2/s1")
    say("")

    tasks = []
    for spec in rounds:
        gx, gy, gz = spec["grids"]
        sigma = spec["sigma"]
        idx = [(i, j, k) for i in range(len(gx))
               for j in range(len(gy)) for k in range(len(gz))]
        if limit:
            step = max(1, len(idx) // limit)
            idx = idx[::step][:limit]
        for vname in VARIANTS:
            for (i, j, k) in idx:
                tasks.append((spec["round"], vname, i, j, k,
                              float(gx[i]), float(gy[j]), float(gz[k]), sigma))
    say("总候选-变体数 : %d" % len(tasks))

    # 输出缓冲
    res = {sp["round"]: {v: np.full(sp["E_paper"].shape, np.nan) for v in VARIANTS}
           for sp in rounds}
    ks = {sp["round"]: {v: np.zeros(sp["E_paper"].shape[:3], dtype=int) for v in VARIANTS}
          for sp in rounds}
    s21s = {sp["round"]: {v: np.zeros(sp["E_paper"].shape[:3]) for v in VARIANTS}
            for sp in rounds}

    t1 = time.time()
    done = 0
    ctx = mp.get_context("spawn")
    with ctx.Pool(processes=workers, initializer=_init_worker,
                  initargs=(setup, meas, meta["n_discr_mon"], meta["downsamp_factor"],
                            meta["npx"])) as pool:
        for out in pool.imap_unordered(_task, tasks, chunksize=4):
            rnd, vname, i, j, k, er, eg, eb, K, s21 = out
            res[rnd][vname][i, j, k, :] = (er, eg, eb)
            ks[rnd][vname][i, j, k] = K
            s21s[rnd][vname][i, j, k] = s21
            done += 1
            if done % 200 == 0:
                el = time.time() - t1
                say("  进度 %d/%d  %.1f s  预计剩 %.1f s"
                    % (done, len(tasks), el, el / done * (len(tasks) - done)))
    say("计算完成，用时 %.1f s" % (time.time() - t1))
    say("")

    # ---- 统计 ----
    summary = {"meta": meta, "pre_registration": {
        "P1_faithful_max_rel_dev_le_2pct": None,
        "P2_no_mirror_same_model_le_2pct": None,
        "P2_no_mirror_must_split_ge_10pct": None},
        "rounds": []}

    for sp in rounds:
        r = sp["round"]
        Ep = np.asarray(sp["E_paper"], float)
        say("=" * 84)
        say("第 %d 轮  存档 %s  网格 %s  sigma=%.6g（threshold[%d]）  Ndiscr_mon=%d"
            % (r, sp["mat"], tuple(len(g) for g in sp["grids"]), sp["sigma"],
               sp["thr_index"], meta["n_discr_mon"]))
        say("  论文能量范围 : %.6e ~ %.6e" % (Ep.min(), Ep.max()))
        say("  论文 best_pos : %s" % np.round(sp["best_pos"], 6).tolist())
        say("")
        rd = {"round": r, "mat": sp["mat"], "sigma": sp["sigma"],
              "thr_index": sp["thr_index"],
              "grid_shape": [int(len(g)) for g in sp["grids"]],
              "paper_min": float(Ep.min()), "paper_max": float(Ep.max()),
              "paper_argmax_ijk": [int(v) for v in np.unravel_index(
                  int(np.argmax(Ep[..., 0])), Ep.shape[:3])],
              "best_pos": sp["best_pos"].tolist(), "variants": {}}
        say("  %-10s %-9s %-9s %-9s %-9s %-10s %-8s" %
            ("变体", "最大偏差%", "中位偏差%", "均值偏差%", "p95偏差%", "偏低占比", "K分布"))
        for vname in VARIANTS:
            M = res[r][vname]
            ok = np.isfinite(Ep) & np.isfinite(M) & (Ep != 0)
            ok3 = np.isfinite(Ep[..., 0]) & np.isfinite(M[..., 0]) & (Ep[..., 0] != 0)
            dev = np.abs(M[ok] / Ep[ok] - 1.0) * 100.0
            signed = M[ok] / Ep[ok] - 1.0
            ku = np.unique(ks[r][vname][ok3]).tolist()
            s21 = s21s[r][vname][ok3]
            rd["variants"][vname] = {
                "max_rel_dev": float(dev.max()) / 100.0,
                "median_rel_dev": float(np.median(dev)) / 100.0,
                "mean_rel_dev": float(dev.mean()) / 100.0,
                "p95_rel_dev": float(np.percentile(dev, 95)) / 100.0,
                "frac_model_lower": float(np.mean(signed < 0)),
                "K_unique": ku, "s2_over_s1_median": float(np.median(s21)),
                "s2_over_s1_min": float(s21.min()), "s2_over_s1_max": float(s21.max()),
                "n_candidates": int(ok.sum()),
            }
            say("  %-10s %-9.4f %-9.4f %-9.4f %-9.4f %-10.4f %-8s"
                % (vname, dev.max(), np.median(dev), dev.mean(),
                   np.percentile(dev, 95), np.mean(signed < 0), ku))
        # best_pos 邻域
        bp = sp["best_pos"]
        gx, gy, gz = sp["grids"]
        bi = [int(np.argmin(np.abs(gx - bp[0]))), int(np.argmin(np.abs(gy - bp[1]))),
              int(np.argmin(np.abs(gz - bp[2])))]
        say("")
        say("  paper best_pos 最近网格点 (i,j,k)=%s  位置=%s"
            % (bi, np.round([gx[bi[0]], gy[bi[1]], gz[bi[2]]], 6).tolist()))
        say("  %-16s %-13s %-13s %-9s %-13s %-9s" %
            ("网格点(i,j,k)", "论文(r)", "忠实(r)", "忠实比值", "去镜像(r)", "去镜像比值"))
        nb = [tuple(bi)]
        for ax in range(3):
            for dd in (-1, 1):
                t = list(bi)
                t[ax] += dd
                if 0 <= t[ax] < len(sp["grids"][ax]):
                    nb.append(tuple(t))
        nb_blocks = []
        for (i, j, k) in nb:
            pap = float(Ep[i, j, k, 0])
            f = float(res[r]["faithful"][i, j, k, 0])
            n = float(res[r]["no_mirror"][i, j, k, 0])
            nb_blocks.append({"ijk": [i, j, k], "paper_r": pap,
                              "faithful_r": f, "faithful_ratio": f / pap,
                              "no_mirror_r": n, "no_mirror_ratio": n / pap})
            say("  %-16s %-13.6e %-13.6e %-9.6f %-13.6e %-9.6f"
                % (str((i, j, k)), pap, f, f / pap, n, n / pap))
        rd["best_pos_neighborhood"] = nb_blocks
        say("")
        summary["rounds"].append(rd)

    # ---- 逐值对照表（红通道，全网格取样） ----
    say("=" * 84)
    say("逐值对照表（红通道，每轮按论文能量降序取前 12 个候选）")
    say("=" * 84)
    table = []
    for sp in rounds:
        r = sp["round"]
        Ep = np.asarray(sp["E_paper"], float)
        order = np.argsort(Ep[..., 0].ravel())[::-1][:12]
        say("--- 第 %d 轮 ---" % r)
        say("  %-12s %-13s %-13s %-9s %-13s %-9s" %
            ("(i,j,k)", "论文(r)", "忠实(r)", "忠实比值", "去镜像(r)", "去镜像比值"))
        for o in order:
            i, j, k = np.unravel_index(int(o), Ep.shape[:3])
            pap = float(Ep[i, j, k, 0])
            f = float(res[r]["faithful"][i, j, k, 0])
            n = float(res[r]["no_mirror"][i, j, k, 0])
            table.append({"round": r, "ijk": [int(i), int(j), int(k)],
                          "paper_r": pap, "faithful_r": f, "faithful_ratio": f / pap,
                          "no_mirror_r": n, "no_mirror_ratio": n / pap})
            say("  %-12s %-13.6e %-13.6e %-9.6f %-13.6e %-9.6f"
                % (str((i, j, k)), pap, f, f / pap, n, n / pap))
        say("")
    summary["top12_table"] = table

    # ---- 预登记判定 ----
    allmax = {v: max(sp["variants"][v]["max_rel_dev"] for sp in summary["rounds"])
              for v in VARIANTS}
    allmed = {v: max(sp["variants"][v]["median_rel_dev"] for sp in summary["rounds"])
              for v in VARIANTS}
    summary["overall"] = {
        "faithful_max_rel_dev_over_rounds": allmax["faithful"],
        "no_mirror_max_rel_dev_over_rounds": allmax["no_mirror"],
        "faithful_median_rel_dev_over_rounds": allmed["faithful"],
        "no_mirror_median_rel_dev_over_rounds": allmed["no_mirror"],
    }
    say("=" * 84)
    say("预登记判定")
    say("=" * 84)
    say("  P1 忠实 vs 论文：三轮最大相对偏差 = %.4f%%  -> %s"
        % (allmax["faithful"] * 100,
           "成立（<=2%）" if allmax["faithful"] <= 0.02 else "不成立（>2%）"))
    say("  P2 去镜像 vs 论文：三轮最大相对偏差 = %.4f%%  -> %s"
        % (allmax["no_mirror"] * 100,
           "『同一模型』（<=2%）" if allmax["no_mirror"] <= 0.02
           else ("『必须分栏单列』（>=10%）" if allmax["no_mirror"] >= 0.10
                 else "中间地带（2%~10%）")))
    say("  中位相对偏差：忠实 = %.4f%%  去镜像 = %.4f%%"
        % (allmed["faithful"] * 100, allmed["no_mirror"] * 100))
    for r in (1, 2, 3):
        sp = next(s for s in summary["rounds"] if s["round"] == r)
        say("  第 %d 轮 s2/s1（中位）：忠实 = %.6f  去镜像 = %.6f"
            % (r, sp["variants"]["faithful"]["s2_over_s1_median"],
               sp["variants"]["no_mirror"]["s2_over_s1_median"]))
    summary["pre_registration"]["P1_faithful_max_rel_dev_le_2pct"] = bool(allmax["faithful"] <= 0.02)
    summary["pre_registration"]["P2_no_mirror_same_model_le_2pct"] = bool(allmax["no_mirror"] <= 0.02)
    summary["pre_registration"]["P2_no_mirror_must_split_ge_10pct"] = bool(allmax["no_mirror"] >= 0.10)
    say("")
    say("总用时 %.1f s" % (time.time() - t0))

    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    with open(log_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(_LOG) + "\n")
    with open(json_path, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=2)
    np.savez_compressed(npz_path,
                        **{("E_%s_r%d" % (v, s["round"])): res[s["round"]][v]
                           for v in VARIANTS for s in rounds},
                        **{("K_%s_r%d" % (v, s["round"])): ks[s["round"]][v]
                           for v in VARIANTS for s in rounds},
                        **{("s21_%s_r%d" % (v, s["round"])): s21s[s["round"]][v]
                           for v in VARIANTS for s in rounds})
    say("日志 : %s" % log_path)
    say("JSON : %s" % json_path)
    say("NPZ  : %s" % npz_path)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default=DEFAULT_DATA)
    ap.add_argument("--workers", type=int, default=max(2, (os.cpu_count() or 4) - 4))
    ap.add_argument("--limit", type=int, default=0, help="每轮每变体抽样点数（0=全网格）")
    ap.add_argument("--log", default=os.path.join(RESULTS, "x30_t7_run.log"))
    ap.add_argument("--json", default=os.path.join(RESULTS, "x30_t7_result.json"))
    ap.add_argument("--npz", default=os.path.join(RESULTS, "x30_t7_energy.npz"))
    args = ap.parse_args()
    return run(args.workers, args.limit, args.data_dir, args.log, args.json, args.npz)


if __name__ == "__main__":
    sys.exit(main())
