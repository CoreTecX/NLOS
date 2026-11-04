# -*- coding: utf-8 -*-
"""X32 —— 两项主机侧定点重建实验（复用 X20 已冻结的定点核 / 双精度核 / 驱动）。

实验 A：论文档（29x36，M=3906，N=1044，315 候选）的定点重建。
实验 B：B 档候选 121 的「秩 vs 重建质量」实测（秩 32/64/96 以及更高做对照）。

真值展平口径（关键，见报告「口径」一节）
----------------------------------------
X20 用 `f2d.reshape(-1)`（C 序）。但本仓其它定位实验（src/model/scripts/x4j/x4k/x4l、
forward_paper.py）一律用 `order="F"`，因为 model.psm 的隐藏列序是**列优先**
（forward_loc.make_plan 里 jj = (row0-1) + nrow*(col0-1)）。本脚本两种都算：
  · F 序 = 与算子/本仓定位链一致的原生口径（物理口径）
  · C 序 = X20 口径（复现 X20 的 vs 真值数字，便于逐位对照）
「定点 vs 双精度参考」一列与展平序无关（两条解用同一个 d）。

只写驱动/编排，不动任何 HLS/RTL 算法核源码。秩 >96 时靠 `-DPS_LR_MAX_RANK=<N>`
（以及双精度层的 `PS_LR_MAX_RANK_FP`）把核的编译期上限抬起来，**不改数值语义**。

用法（仓库根执行）：
    python build/scripts/x32_fixed_recon_paper_and_rank.py --stage paper
    python build/scripts/x32_fixed_recon_paper_and_rank.py --stage rank
    python build/scripts/x32_fixed_recon_paper_and_rank.py --stage both
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import x20_hls_recon_image as X  # noqa: E402  复用 X20 的指标/量化/驱动/出图口径

FL, BM, psm = X.FL, X.BM, X.psm

# 中间产物目录。刻意与 build/x32_work（另有并行任务在用）分开，避免互相清目录。
WORK = os.path.join(REPO, "build", "x32b_work")
RESULTS = os.path.join(REPO, "sim", "results")
B_PSM = os.path.join(REPO, "sim", "models", "model_b.psm")
P_PSM = os.path.join(REPO, "sim", "models", "model_paper.psm")
B_CAND = 121
P_CAND = 157
B_SHAPE = (18, 24)
P_SHAPE = (29, 36)


def log(msg: str) -> None:
    print(msg, flush=True)


# ------------------------------------------------------------------ 算子源

def field_operator(kind: str, cand: int, rank: int):
    """现场重算算子：A -> 竖直差分 -> Gram 截断 SVD（与 build_model 同一条数学）。"""
    if kind == "b":
        st = FL.b_loc_setup()
        spec, _ = FL.b_candidate_grid()
        ds = 0
    else:
        st = FL.d11_loc_setup()
        spec = BM.PAPER_GRID
        ds = 4
    grid = BM._candidate_grid(spec)
    _, _, _, ll = grid[cand]
    plan = FL.make_plan(st, 2, downsamp_factor=ds, mz_mirrored=False)
    A = FL.build_A_from_plan(plan, ll)
    npx = FL.num_pixels_for(st, ds)
    B = BM.apply_row_diff(A, npx, axis=0)
    U, S_norm, V, s_max = BM.truncated_svd_via_gram(B, rank)
    return U, S_norm, V, float(s_max), dict(kind=kind, cand=cand, req_rank=rank,
                                            xyz=[float(v) for v in ll],
                                            M=int(B.shape[0]), N=int(B.shape[1]),
                                            npx=int(npx))


def load_psm_operator(path: str, cand: int):
    """按 psm_format.h 的偏移直接读某候选，返回 (U(M,rank), S_norm, V(rank,N), s_max, meta)。"""
    with open(path, "rb") as f:
        hdr = psm.PsmHeader.unpack(f.read(psm.HEADER_BYTES))
        base = psm.HEADER_BYTES + cand * hdr.record_bytes
        f.seek(base)
        raw = f.read(hdr.record_bytes)
    rec = psm._unpack_record(hdr, raw)
    rec.validate(hdr)
    U = np.ascontiguousarray(rec.U.astype(np.float64).T * rec.u_scale)
    S_norm = rec.S.astype(np.float64) * rec.s_scale
    V = np.ascontiguousarray(rec.V.astype(np.float64) * rec.v_scale)
    g = rec.U.astype(np.float64) * rec.u_scale
    gram = g @ g.T
    meta = dict(path=os.path.relpath(path, REPO), cand=cand,
                M=int(hdr.meas_len), N=int(hdr.hidden_len),
                rank_max=int(hdr.rank_max), candidates=int(hdr.candidates),
                meas="%dx%d" % (hdr.meas_h, hdr.meas_w),
                hidden="%dx%d" % (hdr.hidden_h, hdr.hidden_w),
                geometry_hash=int(hdr.geometry_hash), crc32=int(hdr.payload_crc32),
                xyz_mm=[int(rec.x_mm), int(rec.wall_distance_mm), int(rec.z_mm)],
                u_exp=int(round(math.log2(rec.u_scale))),
                s_exp=int(round(math.log2(rec.s_scale))),
                v_exp=int(round(math.log2(rec.v_scale))),
                s_max=float(rec.s_max),
                nonzero_cols=int((np.abs(rec.U).sum(axis=0) != 0).sum()),
                gram_diag_dev=float(np.max(np.abs(np.diag(gram) - 1.0))),
                gram_off_dev=float(np.max(np.abs(gram - np.diag(np.diag(gram))))))
    return U, S_norm, V, float(rec.s_max), meta


def topk(U, S_norm, V, s_max, K):
    return (np.ascontiguousarray(U[:, :K]), np.ascontiguousarray(S_norm[:K]),
            np.ascontiguousarray(V[:K, :]), float(s_max))


# ------------------------------------------------------------------ 驱动

def compile_driver(max_rank: int, exe: str):
    src = os.path.join(HERE, "x20_fixed_vs_double.cpp")
    cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wno-unknown-pragmas",
           "-DPS_LR_MAX_RANK=%d" % max_rank,
           "-DPS_LR_MAX_RANK_FP=%d" % max_rank,
           "-Isrc/hardware", "-Isrc/software/include", "-Isrc/fpga/hls",
           "-Isrc/fpga/hls/pdhg_core", "-Isrc/fpga/hls/lr_operator",
           src,
           "src/fpga/hls/pdhg_core/pdhg_core_fixed.cpp",
           "src/fpga/hls/lr_operator/lr_operator_fixed.cpp",
           "src/fpga/hls/pdhg_core/pdhg_core.cpp",
           "src/fpga/hls/lr_operator/lr_operator.cpp",
           "-o", exe]
    r = subprocess.run(cmd, cwd=REPO, capture_output=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        log(r.stdout); log(r.stderr)
        raise SystemExit("编译失败")
    return exe


def run_source(tag, U, S_norm, V, s_max, F, shape, configs, driver):
    """与 X20 的 run_source 同构；F 既是造测量的真值向量，也是 vs 真值的比较对象。"""
    B_eff = (U * (S_norm * s_max)[None, :]) @ V
    d_unit = B_eff @ F
    s_f, s_d = X.choose_scales(F, d_unit, s_max)
    U_store, S_code, V_store, u_exp, v_exp, s_exp, S_scaled = X.quantize_factors(
        U, S_norm, V, s_max, s_f, s_d)
    D, d_code = X.quantize_measurement(d_unit, s_d)
    Ud_ref = np.ascontiguousarray(U.T)
    Sd_ref = S_norm * (s_max * s_f / s_d)
    Vd_ref = np.ascontiguousarray(V)

    Bq = (U_store.astype(float) * (2.0 ** u_exp)).T @ \
        ((S_code.astype(float) * (2.0 ** s_exp))[:, None] *
         (V_store.astype(float) * (2.0 ** v_exp)))
    Bref = (Ud_ref.T * Sd_ref[None, :]) @ Vd_ref
    quant_B = X.rel_l2(Bq, Bref)
    quant_d = X.rel_l2(d_code.astype(float) / 65536.0, D)
    norm_B = float(np.linalg.norm(Bq, 2))

    tau = math.sqrt(0.99) / math.sqrt(norm_B * norm_B + 8.0)
    tau_q16 = int(round(tau * 65536.0))
    base = dict(n_rows=shape[0], n_cols=shape[1], min_iters=8, residual_every=4,
                tv_mode=0, tau_q16=tau_q16, sigma_q16=tau_q16, theta_q16=65536,
                rho_stop_q8=0)
    sols, runs, stdout_main = {}, {}, ""
    for name, cfg in configs:
        c = dict(base); c.update(cfg)
        case = os.path.join(WORK, "case_%s_%s.bin" % (tag, name))
        res = os.path.join(WORK, "res_%s_%s.bin" % (tag, name))
        X.write_case(case, c, U_store, S_code, V_store, (u_exp, v_exp, s_exp),
                     d_code, Ud_ref, Sd_ref, Vd_ref, D)
        r = subprocess.run([driver, case, res], capture_output=True,
                           encoding="utf-8", errors="replace")
        if r.returncode != 0:
            log(r.stdout); log(r.stderr)
            raise SystemExit("驱动失败：%s/%s" % (tag, name))
        if name == "main":
            stdout_main = r.stdout
        R = X.read_result(res)
        f_fix = R["fixed"] * s_f
        f_ref = R["dbl_ref"] * s_f
        f_same = R["dbl_same"] * s_f
        gt = F * s_f
        sols[name] = dict(fixed=f_fix, dbl_ref=f_ref, dbl_same=f_same)
        runs[name] = dict(
            lambda_q16=c["lambda_q16"], lambda_=c["lambda_q16"] / 65536.0,
            n_iter=c["n_iter"], n_used=R["n_used"], overflow=R["overflow"],
            fixed_vs_dblref=X.metrics(f_fix, f_ref),
            fixed_vs_dblsame=X.metrics(f_fix, f_same),
            dblsame_vs_dblref=X.metrics(f_same, f_ref),
            fixed_vs_gt=dict(X.metrics(f_fix, gt), iou=X.iou_binarized(f_fix, gt)),
            dblref_vs_gt=dict(X.metrics(f_ref, gt), iou=X.iou_binarized(f_ref, gt)),
            u_inf=float(np.max(np.abs(f_fix))))
        rec = runs[name]
        log("      %-9s λ=%.2e N=%5d ovf=%d | fixed vs dblrelL2=%.3e r=%.7f PSNR=%.2f"
            " | fixed vs gt r=%.4f IoU=%.3f"
            % (name, rec["lambda_"], rec["n_iter"], rec["overflow"],
               rec["fixed_vs_dblref"]["rel_l2"], rec["fixed_vs_dblref"]["pearson"],
               rec["fixed_vs_dblref"]["psnr_db"],
               rec["fixed_vs_gt"]["pearson"], rec["fixed_vs_gt"]["iou"]))
    meta = dict(tag=tag, s_f=s_f, s_d=s_d, exps=dict(u=u_exp, v=v_exp, s=s_exp),
                S_code_max=int(S_code.max()), d_code_max=int(np.abs(d_code).max()),
                quant_rel_B=quant_B, quant_rel_d=quant_d,
                norm_B_kernel=norm_B, tau_q16=tau_q16, tau=tau, s_max=s_max)
    return runs, sols, meta, stdout_main


def diag_lines(stdout: str, keys=("[check]", "[A fixed]", "[C double-ref]")):
    return [ln for ln in stdout.splitlines() if any(k in ln for k in keys)]


# ------------------------------------------------------------------ 秩上限

def ceiling(V, S_abs, Ks, Fs):
    s2 = S_abs ** 2
    all_e = float(s2.sum())
    rows = []
    for K in Ks:
        Vk = V[:K, :]
        row = dict(K=int(K), s_min_in_rank=float(S_abs[K - 1]),
                   energy_frac=float(s2[:K].sum() / all_e))
        for tag, F in Fs.items():
            proj = Vk.T @ (Vk @ F)
            row["corr_" + tag] = X.pearson(proj, F)
            row["rel_l2_" + tag] = X.rel_l2(proj, F)
        rows.append(row)
    return rows


def crossing(Ks, corrs, target=0.9):
    for k, c in zip(Ks, corrs):
        if c >= target:
            return int(k)
    return None


# ------------------------------------------------------------------ 实验 A：论文档

def stage_paper(max_rank):
    t0 = time.time()
    log("=" * 88)
    log("实验 A：论文档（29x36, M=3906, N=1044）定点重建（候选 %d）" % P_CAND)
    log("=" * 88)
    driver = compile_driver(max_rank, os.path.join(WORK, "x32_driver.exe"))

    Up, Sp, Vp, smp, pmeta = load_psm_operator(P_PSM, P_CAND)
    log("[A1] psm 候选 %d：meas %s hidden %s rank_max=%d cands=%d geom=%08X crc=%08X"
        % (P_CAND, pmeta["meas"], pmeta["hidden"], pmeta["rank_max"],
           pmeta["candidates"], pmeta["geometry_hash"], pmeta["crc32"]))
    log("     非零列 %d/%d  Gram 对角偏差=%.4e 非对角=%.4e  s_max=%.6f  xyz_mm=%s"
        % (pmeta["nonzero_cols"], pmeta["M"], pmeta["gram_diag_dev"],
           pmeta["gram_off_dev"], pmeta["s_max"], pmeta["xyz_mm"]))

    Uf, Sf, Vf, smf, fmeta = field_operator("paper", P_CAND, 64)
    cmp = dict(
        u=X.metrics(Up, Uf),
        b=X.metrics((Up * (Sp * smp)[None, :]) @ Vp,
                    (Uf * (Sf * smf)[None, :]) @ Vf),
        s_max_field=smf, s_max_psm=smp, field_xyz=fmeta["xyz"])
    log("[A2] psm vs 现场重算：U relL2=%.4e r=%.10f max|Δ|=%.3e | B relL2=%.4e r=%.10f"
        % (cmp["u"]["rel_l2"], cmp["u"]["pearson"], cmp["u"]["max_abs"],
           cmp["b"]["rel_l2"], cmp["b"]["pearson"]))

    Ufr, Sfr, Vfr, smfr, _ = field_operator("paper", P_CAND, min(fmeta["M"], fmeta["N"]))
    log("[A3] 论文档全秩分解 U %s V %s s_max=%.4f（秩上限用）"
        % (Ufr.shape, Vfr.shape, smfr))

    gt = np.load(os.path.join(REPO, "sim", "vectors", "paper_gt.npy"))
    f2d = gt[:, :, 0].astype(float)
    f2d = f2d / f2d.max()
    F_F = np.ascontiguousarray(f2d.reshape(-1, order="F"))
    F_C = np.ascontiguousarray(f2d.reshape(-1, order="C"))
    log("[A4] 真值 paper_gt R (29x36)：max=%.3f 非零像素 %d/%d  ||F_F||=%.4f"
        % (f2d.max(), int((f2d > 0.2).sum()), f2d.size, float(np.linalg.norm(F_F))))

    Ks = [8, 16, 32, 48, 64, 96, 128, 192, 256, 384, 512, 768, 1044]
    ce = ceiling(Vfr, Sfr * smfr, Ks, {"F": F_F, "C": F_C})
    log("[A5] 论文档秩上限（行空间投影）：")
    for row in ce:
        log("     K=%4d s_min=%8.4f E=%.4f corr_F=%.4f corr_C=%.4f"
            % (row["K"], row["s_min_in_rank"], row["energy_frac"],
               row["corr_F"], row["corr_C"]))

    main_cfg = [("main", {"n_iter": 1024, "lambda_q16": 66})]
    scan_cfg = [("lam0", {"n_iter": 1024, "lambda_q16": 0}),
                ("nit16", {"n_iter": 16, "lambda_q16": 66}),
                ("nit256", {"n_iter": 256, "lambda_q16": 66}),
                ("nit4096", {"n_iter": 4096, "lambda_q16": 66})]
    for lam in (1, 3, 7, 33, 328, 655):
        scan_cfg.append(("lam%05d" % lam, {"n_iter": 1024, "lambda_q16": lam}))

    log("[A6] 定点重建（psm 算子，F 序母语口径）...")
    runsF, solsF, metaF, soutF = run_source("ppsmF", Up, Sp, Vp, smp, F_F, P_SHAPE,
                                            main_cfg + scan_cfg, driver)
    log("[A7] 定点重建（psm 算子，C 序 = X20 口径，只跑 main）...")
    runsC, solsC, metaC, soutC = run_source("ppsmC", Up, Sp, Vp, smp, F_C, P_SHAPE,
                                            main_cfg, driver)
    log("[A8] 现场重算算子交叉核对（F 序 main）...")
    runsFD, solsFD, metaFD, soutFD = run_source("pfldF", Uf, Sf, Vf, smf, F_F, P_SHAPE,
                                                main_cfg, driver)

    out = dict(
        meta=dict(cand=P_CAND, M=pmeta["M"], N=pmeta["N"], shape=list(P_SHAPE),
                  meas=pmeta["meas"], hidden=pmeta["hidden"],
                  psm=pmeta, s_max_field=smf,
                  geometry_hash=pmeta["geometry_hash"], crc32=pmeta["crc32"],
                  rank_ceiling=ce, note="论文档首次跑定点链路；真值 paper_gt R 通道"),
        psm_vs_field=cmp,
        order_note="F 序为算子/本仓定位链母语口径；C 序复现 X20 口径",
        runs_psm_F=dict(meta=metaF, runs=runsF),
        runs_psm_C_main=dict(meta=metaC, runs=runsC),
        runs_field_F_main=dict(meta=metaFD, runs=runsFD),
        driver_stdout=dict(F=diag_lines(soutF), C=diag_lines(soutC),
                           field=diag_lines(soutFD)),
        elapsed_s=time.time() - t0)
    _make_paper_figures(f2d, solsF, solsFD, pmeta, ce, Ks)
    jp = os.path.join(RESULTS, "x32_paper_fixed_recon.json")
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    log("      已保存 %s（%.1f s）" % (os.path.relpath(jp, REPO), time.time() - t0))
    return out


def _make_paper_figures(f2d, solsF, solsFD, pmeta, ce, Ks):
    font = X._setup_font()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def panel(ax, img, title):
        ax.imshow(img, cmap="gray", interpolation="nearest")
        ax.set_title(title, fontsize=8.5)
        ax.set_xticks([]); ax.set_yticks([])

    p = solsF["main"]
    fd = solsFD["main"]
    r_gt_fix = X.pearson(p["fixed"], f2d.reshape(-1, order="F"))
    r_gt_ref = X.pearson(p["dbl_ref"], f2d.reshape(-1, order="F"))
    fig, axes = plt.subplots(2, 3, figsize=(14.0, 8.4))
    # F 序：原生口径，显示也用 F 序
    panel(axes[0][0], f2d, "隐藏真值 paper_gt R（29x36）")
    panel(axes[0][1], p["dbl_ref"].reshape(P_SHAPE, order="F"),
          "论文档双精度参考（F 序）\nr_vs_gt=%.4f" % r_gt_ref)
    panel(axes[0][2], p["fixed"].reshape(P_SHAPE, order="F"),
          "论文档定点重建（psm，F 序）\nr_vs_gt=%.4f" % r_gt_fix)
    panel(axes[1][0], (p["fixed"] - p["dbl_ref"]).reshape(P_SHAPE, order="F"),
          "定点 − 双精度（本源）\nrelL2=%.2e"
          % X.rel_l2(p["fixed"], p["dbl_ref"]))
    # C 序：X20 口径，显示用 C 序
    panel(axes[1][1], p["fixed"].reshape(P_SHAPE, order="C"),
          "定点重建（psm，C 序=X20口径）\n显示按 C 序")
    panel(axes[1][2], fd["fixed"].reshape(P_SHAPE, order="F"),
          "现场重算算子·定点（F 序）\nvs psm定点 relL2=%.2e"
          % X.rel_l2(fd["fixed"], p["fixed"]))
    fig.suptitle("X32 实验A：论文档 29x36 定点重建（候选 %d，rank 64；中文字体 %s）"
                 % (P_CAND, font), fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    p1 = os.path.join(RESULTS, "x32_paper_recon.png")
    fig.savefig(p1, dpi=140); plt.close(fig)

    lams = [n for n in solsF if n.startswith("lam") or n in ("main", "nit16", "nit256", "nit4096")]
    fig2, ax2 = plt.subplots(1, 2, figsize=(11.5, 4.4))
    Kf = [r["K"] for r in ce]
    ax2[0].plot(Kf, [r["corr_F"] for r in ce], "o-", label="行空间投影上限（F 序）")
    ax2[0].plot(Kf, [r["corr_C"] for r in ce], "s--", label="行空间投影上限（C 序=X20）")
    ax2[0].set_xscale("log"); ax2[0].axhline(0.9, color="gray", ls=":")
    ax2[0].set_xlabel("秩 K"); ax2[0].set_ylabel("与真值相关系数上限")
    ax2[0].set_title("论文档秩上限"); ax2[0].legend(fontsize=8); ax2[0].grid(alpha=0.3)
    ax2[1].plot(Kf, [r["energy_frac"] for r in ce], "^-")
    ax2[1].set_xscale("log"); ax2[1].set_xlabel("秩 K"); ax2[1].set_ylabel("谱能量占比")
    ax2[1].set_title("论文档累积谱能量"); ax2[1].grid(alpha=0.3)
    fig2.tight_layout()
    p2 = os.path.join(RESULTS, "x32_paper_rank_ceiling.png")
    fig2.savefig(p2, dpi=140); plt.close(fig2)
    log("      已保存 %s / %s" % (os.path.relpath(p1, REPO), os.path.relpath(p2, REPO)))


# ------------------------------------------------------------------ 实验 B：秩

def stage_rank(max_rank):
    t0 = time.time()
    log("=" * 88)
    log("实验 B：B 档候选 %d 的「秩 vs 重建质量」实测" % B_CAND)
    log("=" * 88)
    driver = compile_driver(max_rank, os.path.join(WORK, "x32_driver.exe"))

    # 全秩分解一次，各 K 取前 K 个三元组（与 truncated_svd_via_gram(B,K) 同一组）
    Uf, Sf, Vf, smf, fmeta = field_operator("b", B_CAND, min(9120, 432))
    nfull = Vf.shape[0]
    log("[B1] B 档候选 %d 全秩分解 U %s V %s s_max=%.4f"
        % (B_CAND, Uf.shape, Vf.shape, smf))

    gt = np.load(os.path.join(REPO, "sim", "vectors", "paper_gt.npy"))
    f2d = X.resize_avg(gt[:, :, 0].astype(float), B_SHAPE)
    f2d = f2d / f2d.max()
    F_F = np.ascontiguousarray(f2d.reshape(-1, order="F"))
    F_C = np.ascontiguousarray(f2d.reshape(-1, order="C"))

    Ks_ce = [8, 16, 24, 32, 48, 64, 80, 96, 112, 128, 160, 192, 224, 256,
             320, 384, 432]
    ce = ceiling(Vf, Sf * smf, Ks_ce, {"F": F_F, "C": F_C})
    log("[B2] 秩上限（行空间投影）：")
    for row in ce:
        log("     K=%4d s_min=%8.4f E=%.4f corr_F=%.4f corr_C=%.4f"
            % (row["K"], row["s_min_in_rank"], row["energy_frac"],
               row["corr_F"], row["corr_C"]))
    kF = crossing(Ks_ce, [r["corr_F"] for r in ce], 0.9)
    kC = crossing(Ks_ce, [r["corr_C"] for r in ce], 0.9)
    log("     上限过 0.9 的最小 K：F 序=%s  C 序=%s" % (kF, kC))

    Ks = [32, 64, 96, 128, 192, 256, 384]
    main_cfg = [("main", {"n_iter": 1024, "lambda_q16": 66})]
    table = []
    sols_by_K = {}
    for K in Ks:
        Uk, Sk, Vk, smk = topk(Uf, Sf, Vf, smf, K)
        log("[B3] K=%d 定点重建 ..." % K)
        rF, sF, mF, oF = run_source("bK%03dF" % K, Uk, Sk, Vk, smk, F_F, B_SHAPE,
                                    main_cfg, driver)
        rC, sC, mC, oC = run_source("bK%03dC" % K, Uk, Sk, Vk, smk, F_C, B_SHAPE,
                                    main_cfg, driver)
        sols_by_K[K] = dict(F=sF["main"], C=sC["main"])
        row = dict(K=K, s_max=smk,
                   norm_B_kernel=mF["norm_B_kernel"], tau=mF["tau"],
                   quant_rel_B=mF["quant_rel_B"],
                   fixed_vs_dblref_F=rF["main"]["fixed_vs_dblref"],
                   fixed_vs_gt_F=rF["main"]["fixed_vs_gt"],
                   dblref_vs_gt_F=rF["main"]["dblref_vs_gt"],
                   fixed_vs_gt_C=rC["main"]["fixed_vs_gt"],
                   dblref_vs_gt_C=rC["main"]["dblref_vs_gt"],
                   overflow_F=rF["main"]["overflow"], overflow_C=rC["main"]["overflow"],
                   n_used_F=rF["main"]["n_used"])
        table.append(row)
        log("     K=%3d  fixed vs dbl relL2=%.3e r=%.7f | F序: r_gt=%.4f IoU=%.3f PSNR=%.2f"
            " | C序(X20): r_gt=%.4f IoU=%.3f"
            % (K, row["fixed_vs_dblref_F"]["rel_l2"], row["fixed_vs_dblref_F"]["pearson"],
               row["fixed_vs_gt_F"]["pearson"], row["fixed_vs_gt_F"]["iou"],
               row["fixed_vs_gt_F"]["psnr_db"],
               row["fixed_vs_gt_C"]["pearson"], row["fixed_vs_gt_C"]["iou"]))

    out = dict(
        meta=dict(cand=B_CAND, M=fmeta["M"], N=fmeta["N"], shape=list(B_SHAPE),
                  xyz=fmeta["xyz"], s_max=smf, full_rank=int(nfull),
                  max_rank_macro=max_rank,
                  crossing_K_F=kF, crossing_K_C=kC),
        rank_ceiling=ce, table=table,
        note="F 序=算子母语口径；C 序=X20 口径；定点核上限 PS_LR_MAX_RANK 默认 96",
        elapsed_s=time.time() - t0)
    _make_rank_figures(f2d, sols_by_K, ce, table)
    jp = os.path.join(RESULTS, "x32_rank_vs_quality.json")
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    log("      已保存 %s（%.1f s）" % (os.path.relpath(jp, REPO), time.time() - t0))
    return out


def _make_rank_figures(f2d, sols_by_K, ce, table):
    font = X._setup_font()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    Kf = [r["K"] for r in ce]
    fig, ax = plt.subplots(1, 2, figsize=(13.0, 4.6))
    ax[0].plot(Kf, [r["corr_F"] for r in ce], "o-", color="tab:blue",
               label="行空间投影上限（F 序）")
    ax[0].plot(Kf, [r["corr_C"] for r in ce], "s--", color="tab:orange",
               label="行空间投影上限（C 序=X20）")
    Kt = [r["K"] for r in table]
    ax[0].plot(Kt, [r["fixed_vs_gt_F"]["pearson"] for r in table], "D-", color="navy",
               label="定点重建 vs 真值（F 序）")
    ax[0].plot(Kt, [r["fixed_vs_gt_C"]["pearson"] for r in table], "v--", color="crimson",
               label="定点重建 vs 真值（C 序=X20）")
    ax[0].axhline(0.9, color="gray", ls=":")
    ax[0].set_xscale("log"); ax[0].set_xlabel("秩 K"); ax[0].set_ylabel("与真值相关系数")
    ax[0].set_title("B 档候选 %d：秩 vs 相关系数" % B_CAND)
    ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3, which="both")
    ax[1].plot(Kt, [r["fixed_vs_gt_F"]["psnr_db"] for r in table], "o-", color="navy",
               label="PSNR vs 真值（F 序）")
    ax[1].plot(Kt, [r["fixed_vs_gt_F"]["iou"] for r in table], "s-", color="darkgreen",
               label="IoU vs 真值（F 序）")
    ax[1].set_xscale("log"); ax[1].set_xlabel("秩 K"); ax[1].legend(fontsize=8)
    ax[1].set_title("B 档候选 %d：秩 vs PSNR / IoU" % B_CAND)
    ax[1].grid(alpha=0.3, which="both")
    fig.tight_layout()
    p1 = os.path.join(RESULTS, "x32_rank_vs_quality.png")
    fig.savefig(p1, dpi=140); plt.close(fig)

    show = [64, 96, 192, 384]
    fig2, axes = plt.subplots(1, len(show) + 1, figsize=(15.0, 3.6))
    axes[0].imshow(f2d, cmap="gray", interpolation="nearest")
    axes[0].set_title("真值（F 序显示）", fontsize=8.5)
    for i, K in enumerate(show):
        img = sols_by_K[K]["F"]["fixed"].reshape(B_SHAPE, order="F")
        axes[i + 1].imshow(img, cmap="gray", interpolation="nearest")
        r = X.pearson(sols_by_K[K]["F"]["fixed"], f2d.reshape(-1, order="F"))
        axes[i + 1].set_title("K=%d 定点(F序) r=%.3f" % (K, r), fontsize=8.5)
    for a in axes.ravel():
        a.set_xticks([]); a.set_yticks([])
    fig2.suptitle("X32 实验B：B 档候选 %d 不同秩下的定点重建（中文字体 %s）"
                  % (B_CAND, font), fontsize=10.5)
    fig2.tight_layout(rect=(0, 0, 1, 0.92))
    p2 = os.path.join(RESULTS, "x32_rank_recon_gallery.png")
    fig2.savefig(p2, dpi=140); plt.close(fig2)
    log("      已保存 %s / %s" % (os.path.relpath(p1, REPO), os.path.relpath(p2, REPO)))


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="both", choices=("paper", "rank", "both"))
    ap.add_argument("--max-rank", type=int, default=512,
                    help="编译期 PS_LR_MAX_RANK / PS_LR_MAX_RANK_FP")
    args = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)
    if args.stage in ("paper", "both"):
        stage_paper(args.max_rank)
    if args.stage in ("rank", "both"):
        stage_rank(args.max_rank)
    return 0


if __name__ == "__main__":
    sys.exit(main())
