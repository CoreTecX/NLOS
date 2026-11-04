# -*- coding: utf-8 -*-
"""x4i —— 复现论文 Table S1 的 D11 定位结果（**纯 Python**，不依赖 MATLAB）。

背景
----
x4e（全分辨率 972 候选）与 x4g（63²、125 候选）两次都退化：K 恒为 1、能量要么逐位
相同、要么只是 y 的纯函数。x4f 查明是参数档用错，x4h 进一步查明**定位链用的是另一套
前向模型**（`SimulateForwardModelPerBlock`，解析阴影；不是重建链的 `simulate_block`）。

本脚本把那一套模型在 Python 里实现出来，然后照 `table_S1.m` 的流程跑完整三轮，
看能不能复现论文自己的定位结果。

靶子（`load_experiment_config_data_localization.m:16-24` 里注释掉的
"Estimated occluder positions for the different scenes" 就是论文的定位估计，
第 2 位已按 `(D - p_y)` 口径写好）::

    scene        x        y_stored   z        y_manuscript(D-y)
    mushroom     0.4583   0.5408     0.2026   0.4892
    tommy        0.4569   0.5744     0.2080   0.4556
    bu           0.4733   0.5661     0.2072   0.4639
    rgb          0.4693   0.5626     0.2080   0.4674

真值（`load_experiment_config_data_localization.m:23`，`useEstimatedOccPos = false`）::

    x = 0.4700   y_stored = D-0.460 = 0.5700   z = 0.2040

数据
----
数据集是 **MAT v5/v7**（不是 v7.3），`scipy.io.loadmat` 可直接读，
所以本脚本不需要 MATLAB 也不需要 h5py。

用法::

    python x4i_table_s1_repro.py --variant faithful --max-round 3
    python x4i_table_s1_repro.py --variants faithful,no_mirror,no_stick,no_va --max-round 1
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

REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))   # …\penumbrasight
RESULTS = os.path.join(REPO, "sim", "results")
DEFAULT_DATA = (r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master"
                r"\Data\TestPosD11")

D = 1.03

# table_S1.m:49-72 的场景分支
SCENES = {
    "rgb":      dict(mat="image_test_colbar20.mat",   n_iter=4, sigma_th1=0.75, sigma_th2=0.2),
    "bu":       dict(mat="image_test_bur20.mat",      n_iter=3, sigma_th1=0.75, sigma_th2=0.5),
    "mushroom": dict(mat="image_test_mushroom20.mat", n_iter=3, sigma_th1=0.75, sigma_th2=0.2),
    "tommy":    dict(mat="image_test_tommy20.mat",    n_iter=4, sigma_th1=0.75, sigma_th2=0.5),
}

PAPER_EST = {
    "mushroom": [0.4583, D - 0.4892, 0.2026],
    "tommy":    [0.4569, D - 0.4556, 0.2080],
    "bu":       [0.4733, D - 0.4639, 0.2072],
    "rgb":      [0.4693, D - 0.4674, 0.2080],
}

# 模型变体：用来判断「照抄原代码」与「技术上更自洽」差多少
VARIANTS = {
    "faithful":  dict(view_angle=True,  mz_mirrored=True,  stick=True),
    "no_mirror": dict(view_angle=True,  mz_mirrored=False, stick=True),
    "no_stick":  dict(view_angle=True,  mz_mirrored=True,  stick=False),
    "no_va":     dict(view_angle=False, mz_mirrored=True,  stick=True),
}

N_ONEPARAM = 30
RANGE_VALS = [v * 4 for v in (0.025, 0.0125, 0.00625, 0.003125)]   # table_S1.m:86


# ----------------------------------------------------------------------
def _argmax_rgb(E: np.ndarray, gx, gy, gz):
    """`occluderposgridsearch.m:95-111`：逐通道 argmax，再取三通道平均。"""
    p_rgb = np.zeros((3, 3), dtype=float)
    for c in range(3):
        i, j, k = np.unravel_index(int(np.argmax(E[..., c])), E.shape[:3])
        p_rgb[c] = [gx[i], gy[j], gz[k]]
    return p_rgb, p_rgb.mean(axis=0)


def _axis_spread(E: np.ndarray):
    """各轴上的边缘极差：先在其他两轴上取最大，再求差。

    这是判断「能量是否真的看见了某个轴」最简单的量：若某轴极差为 0，
    说明在该轴上能量完全不变，argmax 在该轴上没有信息（x4g 就是这种情形）。
    """
    out = {}
    for name, ax in (("x", 0), ("y", 1), ("z", 2)):
        m = E.max(axis=tuple(a for a in (0, 1, 2) if a != ax))
        out[name] = float(m.max() - m.min())
    return out


def _log(fh, msg=""):
    print(msg, flush=True)
    if fh is not None:
        fh.write(msg + "\n")
        fh.flush()


# ----------------------------------------------------------------------
def run_variant(setup, meas, scene, variant, max_round, fh):
    cfg = SCENES[scene]
    kw = VARIANTS[variant]
    n_iter = cfg["n_iter"]
    if max_round and max_round > 0:
        n_iter = min(n_iter, max_round)

    akw = dict(view_angle=kw["view_angle"], stick=kw["stick"])
    plans = {}

    def plan_for(N):
        if N not in plans:
            plans[N] = FL.make_plan(setup, N, downsamp_factor=4,
                                    mz_mirrored=kw["mz_mirrored"])
        return plans[N]

    def search(gx, gy, gz, N, sigma_th):
        pl = plan_for(N)
        A_of = lambda x, y, z: FL.build_A_from_plan(pl, [x, y, z], **akw)   # noqa: E731
        return FL.loc_energy_grid(A_of, meas, gx, gy, gz, sigma_th)

    _log(fh, "-" * 68)
    _log(fh, "变体 %-10s 场景 %-8s n_iter=%d  模型: va=%s mirror=%s stick=%s"
         % (variant, scene, n_iter, kw["view_angle"], kw["mz_mirrored"], kw["stick"]))
    _log(fh, "-" * 68)

    rounds = []
    p_est = np.zeros(3)
    for ii in range(1, n_iter + 1):
        t_ii = time.time()
        if ii == 1:
            N, sigma_th = 2, cfg["sigma_th1"]
            gx = np.linspace(0.1, 0.6, 5)
            gy = np.linspace(0.1, 0.6, 5)
            gz = np.linspace(0.0, 0.3, 5)
            E, Kp = search(gx, gy, gz, N, sigma_th)
            prgb, p_est = _argmax_rgb(E, gx, gy, gz)
            spread = _axis_spread(E)
            uniq = [int(len(np.unique(E[..., c]))) for c in range(3)]
            kuniq = np.unique(Kp)

            _log(fh, "第 1 轮（全局 5x5x5，Ndiscr_mon=%d，sigma_th=%.2f）" % (N, sigma_th))
            _log(fh, "  K 的取值分布 : %s" % np.array2string(kuniq, max_line_width=120))
            _log(fh, "  K 全为 1 吗  : %s" % ("是（仍然退化）" if kuniq.tolist() == [1] else "否"))
            _log(fh, "  能量互异值数 : r/g/b = %s （共 125 个候选）" % uniq)
            _log(fh, "  各轴边缘极差 : x=%.6e  y=%.6e  z=%.6e"
                 % (spread["x"], spread["y"], spread["z"]))
            _log(fh, "  逐通道 argmax: r=%s g=%s b=%s"
                 % (np.round(prgb[0], 4).tolist(), np.round(prgb[1], 4).tolist(),
                    np.round(prgb[2], 4).tolist()))
            rounds.append(dict(round=1, kind="global", N=N, sigma_th=sigma_th,
                               p_est=p_est.tolist(), p_rgb=prgb.tolist(),
                               K_unique=kuniq.tolist(), E_unique=uniq,
                               spread=spread, secs=time.time() - t_ii))
            E_r1 = E
        else:
            sigma_th = cfg["sigma_th2"] / (10 ** (ii - 2))
            ar = RANGE_VALS[ii - 2]
            sub = []

            # 顺序照 table_S1.m:110-130：x → z（用更新后的 p_est）→ y（Ndiscr_mon=10）
            for axis, N in ((0, 1), (2, 1), (1, 10)):
                gx = np.array([p_est[0]])
                gy = np.array([p_est[1]])
                gz = np.array([p_est[2]])
                if axis == 0:
                    gx = np.linspace(p_est[0] - ar, p_est[0] + ar, N_ONEPARAM)
                elif axis == 2:
                    gz = np.linspace(p_est[2] - ar, p_est[2] + ar, N_ONEPARAM)
                else:
                    gy = np.linspace(p_est[1] - ar, p_est[1] + ar, N_ONEPARAM)
                E, Kp = search(gx, gy, gz, N, sigma_th)
                prgb, p_est = _argmax_rgb(E, gx, gy, gz)
                sub.append(dict(axis="xyz"[axis], N=N, p_est=p_est.tolist(),
                                K_unique=np.unique(Kp).tolist(),
                                E_unique=[int(len(np.unique(E[..., c]))) for c in range(3)]))
            _log(fh, "第 %d 轮（x/z/y 逐个精化，arange=%.6g，sigma_th=%.4g）" % (ii, ar, sigma_th))
            for s in sub:
                _log(fh, "  %s 轴  Ndiscr_mon=%2d  K取值=%-14s 能量互异=%-10s -> p=[%.4f %.4f %.4f]"
                     % (s["axis"], s["N"], str(s["K_unique"]), str(s["E_unique"]),
                        s["p_est"][0], s["p_est"][1], s["p_est"][2]))
            rounds.append(dict(round=ii, kind="refine", arange=ar, sigma_th=sigma_th,
                               sub=sub, p_est=p_est.tolist(), secs=time.time() - t_ii))

        _log(fh, "  => 第 %d 轮结束 p_est(搜索坐标) = [%.6f %.6f %.6f]   用时 %.1f s"
             % (ii, p_est[0], p_est[1], p_est[2], time.time() - t_ii))
        _log(fh)

    est_search = p_est.copy()
    est_ms = np.array([p_est[0], D - p_est[1], p_est[2]])
    truth = setup.occ_ll_true
    paper = np.array(PAPER_EST[scene])

    _log(fh, "最终（搜索坐标）    : [%.6f %.6f %.6f]" % tuple(est_search))
    _log(fh, "最终（论文口径 D-y）: [%.6f %.6f %.6f]" % tuple(est_ms))
    _log(fh, "真值（搜索坐标）    : [%.6f %.6f %.6f]" % tuple(truth))
    _log(fh, "论文的定位估计      : [%.6f %.6f %.6f]" % tuple(paper))
    _log(fh, "与真值的偏差 (m)    : [%+.6f %+.6f %+.6f]  |e|=%.6f"
         % (*tuple(est_search - truth), float(np.linalg.norm(est_search - truth))))
    _log(fh, "与论文估计的偏差 (m): [%+.6f %+.6f %+.6f]  |e|=%.6f"
         % (*tuple(est_search - paper), float(np.linalg.norm(est_search - paper))))

    return dict(variant=variant, scene=scene, n_iter=n_iter, rounds=rounds,
                est_search=est_search.tolist(), est_ms=est_ms.tolist(),
                truth=truth.tolist(), paper=paper.tolist(),
                err_truth=float(np.linalg.norm(est_search - truth)),
                err_paper=float(np.linalg.norm(est_search - paper))), E_r1


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default=DEFAULT_DATA)
    ap.add_argument("--scene", default="bu")
    ap.add_argument("--variants", default="faithful")
    ap.add_argument("--max-round", type=int, default=0,
                    help="0 = 跑满 table_S1 的 n_iter 轮；1 = 只跑全局轮")
    ap.add_argument("--log", default=os.path.join(RESULTS, "x4i_run.log"))
    ap.add_argument("--json", default=os.path.join(RESULTS, "x4i_result.json"))
    args = ap.parse_args()

    setup = FL.d11_loc_setup()
    mat = os.path.join(args.data_root, SCENES[args.scene]["mat"])
    if not os.path.exists(mat):
        raise SystemExit("找不到测量文件: %s" % mat)

    os.makedirs(os.path.dirname(args.log), exist_ok=True)
    fh = open(args.log, "a", encoding="utf-8", newline="\n")

    _log(fh, "")
    _log(fh, "=" * 68)
    _log(fh, "x4i  Table S1 复现   %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    _log(fh, "=" * 68)
    _log(fh, "测量文件 : %s" % mat)
    t0 = time.time()
    meas, gt = FL.load_measurement(mat, 4)
    _log(fh, "测量     : camera_capture %s  mean=%.4f  max=%.4f   (%.1f s)"
         % (meas.shape, meas.mean(), meas.max(), time.time() - t0))
    _log(fh, "隐藏场景真值 shape=%s 非零像素=%d" % (gt.shape, int(np.count_nonzero(gt))))

    out = []
    E_store = {}
    for v in [s.strip() for s in args.variants.split(",") if s.strip()]:
        if v not in VARIANTS:
            raise SystemExit("未知变体 %s（可选 %s）" % (v, sorted(VARIANTS)))
        res, E1 = run_variant(setup, meas, args.scene, v, args.max_round, fh)
        out.append(res)
        E_store[v] = E1
        _log(fh)

    with open(args.json, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    np.savez_compressed(os.path.join(RESULTS, "x4i_E_round1_%s.npz" % args.scene),
                        **E_store)

    _log(fh, "结果 JSON: %s" % args.json)
    _log(fh, "日志     : %s" % args.log)
    fh.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
