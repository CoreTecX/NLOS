# -*- coding: utf-8 -*-
"""X33 —— 展平序判定：把「算子母语列序」这件事用同一套驱动量出来。

问题
----
`model.psm` 的隐藏列序是列优先（F 序）：`forward_loc.make_plan` 里
`jj = (row0-1) + nrow*(col0-1)`。X20 在「vs 隐藏真值」与秩上限里用了 C 序
（`f2d.reshape(-1)`），秩 64 下给出 r=0.4335；X32 用 F 序给出 r=0.6750。
本脚本把两套口径并排重算，并补一个 X20/X32 都没给的对照：

  · 自洽口径（X20/X32 口径）：测量按该序从真值生成，解与该序的真值比。
  · 同一测量两种解释（本轮新增）：测量只按**算子母语 F 序**生成一次，解出 u 后
    分别用 F 序、C 序把 u 还原成图再与**同一张真值图**比。后者回答的是
    「同一个解、两种重排解释，哪种能还原图像」，与「测量怎么造」无关。

三者都走 X20 已冻结的定点核 / 双精度核 / 驱动（`build/scripts/x20_fixed_vs_double.cpp`，
只读复用），不改任何 HLS/RTL 源码。

用法（仓库根）：
    python build/scripts/x33_order_verdict.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import x20_hls_recon_image as X            # noqa: E402  指标 / 量化 / 驱动 / 出图口径
import x32_fixed_recon_paper_and_rank as X32  # noqa: E402  现场算子 / psm 读取 / 秩上限

WORK = os.path.join(REPO, "build", "x33_work")
RESULTS = os.path.join(REPO, "sim", "results")
GT_PATH = os.path.join(REPO, "sim", "vectors", "paper_gt.npy")
MAIN = ("main", {"n_iter": 1024, "lambda_q16": 66})


def log(msg: str) -> None:
    print(msg, flush=True)


def _flat(img, order):
    return np.ascontiguousarray(img.reshape(-1, order=order))


def interpretation(u, s_f, f2d, shape):
    """同一个解 u（场景单位）用 F / C 两种重排还原成图，与同一张真值图比。"""
    out = {}
    for tag, order in (("F", "F"), ("C", "C")):
        img = u.reshape(shape, order=order)
        m = X.metrics(u, _flat(f2d, order))
        m["iou"] = X.iou_binarized(img, f2d)
        out[tag] = m
    return out


def centroid_probe(setup_plan, ncol_guess):
    """对算子 A 的前若干列做「z 质心」探针，独立确认列序。

    把第 j 列还原成 (npx, npx) 墙图（行=z、列=x），量 z 向质心。若列序是 F 序
    （列号先走行=z），则列 0..nrow-1 的 z 质心应明显移动，第 nrow 列回到起点。
    """
    return None  # 由各档在 build 后用 A 直接做（见 run_case）


def run_case(tag, kind, cand, shape, U, S, V, smax, f2d, full_V=None,
             full_S=None):
    """两套自洽口径 + 「同一测量两种解释」。返回 (record, sols)。"""
    out = {}
    F_F, F_C = _flat(f2d, "F"), _flat(f2d, "C")
    driver = os.path.join(WORK, "x33_driver.exe")
    runsF, solsF, metaF, _ = X32.run_source(tag + "F", U, S, V, smax, F_F, shape,
                                            [MAIN], driver)
    runsC, solsC, metaC, _ = X32.run_source(tag + "C", U, S, V, smax, F_C, shape,
                                            [MAIN], driver)
    u = solsF["main"]["fixed"]
    interp = interpretation(u, metaF["s_f"], f2d, shape)
    out = dict(tag=tag, kind=kind, cand=cand,
               self_consistent=dict(
                   F=dict(fixed_vs_gt=runsF["main"]["fixed_vs_gt"],
                          fixed_vs_dblref=runsF["main"]["fixed_vs_dblref"],
                          overflow=runsF["main"]["overflow"]),
                   C=dict(fixed_vs_gt=runsC["main"]["fixed_vs_gt"],
                          fixed_vs_dblref=runsC["main"]["fixed_vs_dblref"],
                          overflow=runsC["main"]["overflow"])),
               same_measurement_interpretation=interp,
               run_meta=dict(s_d_F=metaF["s_d"], s_d_C=metaC["s_d"],
                             quant_rel_B=metaF["quant_rel_B"],
                             norm_B_kernel=metaF["norm_B_kernel"]))
    log("  [%s] 自洽 F 序 vs 真值 r=%.4f IoU=%.3f PSNR=%.2f | 自洽 C 序 r=%.4f IoU=%.3f"
        % (tag, runsF["main"]["fixed_vs_gt"]["pearson"],
           runsF["main"]["fixed_vs_gt"]["iou"],
           runsF["main"]["fixed_vs_gt"]["psnr_db"],
           runsC["main"]["fixed_vs_gt"]["pearson"],
           runsC["main"]["fixed_vs_gt"]["iou"]))
    log("  [%s] 同一测量两种解释   F 序 r=%.4f IoU=%.3f | C 序 r=%.4f IoU=%.3f"
        % (tag, interp["F"]["pearson"], interp["F"]["iou"],
           interp["C"]["pearson"], interp["C"]["iou"]))
    return out, {"F": solsF["main"], "C": solsC["main"]}


def column_order_probe(kind, cand, shape, nrow):
    """算子 A 的列序自证：量前 nrow+2 列的墙图 z 质心。"""
    from penumbrasight_model import forward_loc as FL
    from penumbrasight_model import build_model as BM
    if kind == "b":
        st = FL.b_loc_setup(); ds = 0
        spec, _ = FL.b_candidate_grid()
    else:
        st = FL.d11_loc_setup(); ds = 4
        spec = BM.PAPER_GRID
    grid = BM._candidate_grid(spec)
    _, _, _, ll = grid[cand]
    plan = FL.make_plan(st, 2, downsamp_factor=ds, mz_mirrored=False)
    A = FL.build_A_from_plan(plan, ll)
    npx = plan.npx
    ncol = A.shape[1] // nrow
    zc = []
    for j in range(nrow + 2):
        img = A[:, j].reshape(npx, npx)          # 行=z、列=x
        w = np.abs(img).sum()
        zc.append(float((np.arange(npx) * np.abs(img).sum(axis=1)).sum() / w)
                  if w > 0 else float("nan"))
    # 相邻列还原成场景图后的 L1 梯度（越平滑越接近真实排布）。
    # F 序：列号 jj = a + nrow*b（行=a 最快）-> 按 (ncol, nrow) 的 C 序 reshape。
    # C 序：列号 jj = a*ncol + b            -> 按 (nrow, ncol) 的 C 序 reshape。
    def smooth(order):
        S3 = (A.reshape(npx * npx, ncol, nrow).transpose(0, 2, 1) if order == "F"
              else A.reshape(npx * npx, nrow, ncol))
        g = 0.0
        for a in range(nrow):
            for b in range(ncol):
                v = S3[:, a, b]
                if b + 1 < ncol:
                    g += float(np.abs(v - S3[:, a, b + 1]).sum())
                if a + 1 < nrow:
                    g += float(np.abs(v - S3[:, a + 1, b]).sum())
        return g
    return dict(npx=npx, nrow=nrow, ncol=ncol,
                z_centroid_first_cols=zc,
                col_neighbour_L1=dict(F=smooth("F"), C=smooth("C")))


def main():
    t0 = time.time()
    os.makedirs(WORK, exist_ok=True)
    driver = X32.compile_driver(96, os.path.join(WORK, "x33_driver.exe"))
    # 让 x32 的 run_source 把 case/结果二进制写进本轮工作目录
    X32.WORK = WORK
    log("[0] 驱动已编译 %s" % os.path.relpath(driver, REPO))

    gt = np.load(GT_PATH)
    B_SHAPE, P_SHAPE = (18, 24), (29, 36)
    result = dict(
        note="F 序 = 算子/psm 母语列序（jj=(row0-1)+nrow*(col0-1)）；C 序 = X20 口径",
        main_cfg="lambda_q16=66 (lambda=1.007e-3), n_iter=1024",
        cases={})

    # ---------- B 档（候选 121，现场重算算子，秩 64）----------
    log("[1] B 档候选 121：现场重算秩 64 算子")
    Ub, Sb, Vb, smb, fmeta_b = X32.field_operator("b", X32.B_CAND, 64)
    Ubf, Sbf, Vbf, smbf, _ = X32.field_operator("b", X32.B_CAND, min(9120, 432))
    f2d_b = X.resize_avg(gt[:, :, 0].astype(float), B_SHAPE)
    f2d_b = f2d_b / f2d_b.max()
    rec_b, _ = run_case("vB", "b", X32.B_CAND, B_SHAPE, Ub, Sb, Vb, smb, f2d_b)
    ceil_b = X32.ceiling(Vbf, Sbf * smbf, [64, 96], {"F": _flat(f2d_b, "F"),
                                                      "C": _flat(f2d_b, "C")})
    log("  [B] 秩上限 K=64 corr_F=%.4f corr_C=%.4f | K=96 %.4f / %.4f"
        % (ceil_b[0]["corr_F"], ceil_b[0]["corr_C"],
           ceil_b[1]["corr_F"], ceil_b[1]["corr_C"]))
    rec_b["rank_ceiling"] = ceil_b
    rec_b["column_probe"] = column_order_probe("b", X32.B_CAND, B_SHAPE, B_SHAPE[0])
    result["cases"]["B"] = dict(meta=fmeta_b, **rec_b)

    # ---------- 论文档（候选 157，psm 算子，秩 64）----------
    log("[2] 论文档候选 157：model_paper.psm 算子")
    Up, Sp, Vp, smp, pmeta = X32.load_psm_operator(X32.P_PSM, X32.P_CAND)
    Upf, Spf, Vpf, smpf, _ = X32.field_operator("paper", X32.P_CAND,
                                                min(3906, 1044))
    f2d_p = gt[:, :, 0].astype(float)
    f2d_p = f2d_p / f2d_p.max()
    rec_p, _ = run_case("vP", "paper", X32.P_CAND, P_SHAPE, Up, Sp, Vp, smp, f2d_p)
    ceil_p = X32.ceiling(Vpf, Spf * smpf, [64, 96],
                         {"F": _flat(f2d_p, "F"), "C": _flat(f2d_p, "C")})
    log("  [论文档] 秩上限 K=64 corr_F=%.4f corr_C=%.4f | K=96 %.4f / %.4f"
        % (ceil_p[0]["corr_F"], ceil_p[0]["corr_C"],
           ceil_p[1]["corr_F"], ceil_p[1]["corr_C"]))
    rec_p["rank_ceiling"] = ceil_p
    rec_p["column_probe"] = column_order_probe("paper", X32.P_CAND, P_SHAPE,
                                               P_SHAPE[0])
    result["cases"]["paper"] = dict(meta=pmeta, s_max_field=smpf, **rec_p)

    result["elapsed_s"] = time.time() - t0
    jp = os.path.join(RESULTS, "x33_order_verdict.json")
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    log("已保存 %s（%.1f s）" % (os.path.relpath(jp, REPO), time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
