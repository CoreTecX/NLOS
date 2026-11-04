# -*- coding: utf-8 -*-
"""X34 —— 把论文重建链的「双精度参考解」做成同源（M=15876 那条链）。

背景
----
论文重建链的算子来自 `fig4_column_c`（126x126 墙、M=15876、N=1044），与
本仓定位链的算子（`model_paper.psm`，63x63 墙、M=3906）**不同源**。所以
`sim/vectors/paper_recon_pdhg.npy` / `paper_recon_fista.npy` 这些参考解
无法与定位链的定点重建直接对照（X32 §六缺口 2）。

本脚本用**论文链自己的算子**跑一遍同一条定点链路：
  1) 按 `paper_recon_pdhg.py` 的写法重建 dA（15876x1044）与测量 dm；
  2) 截断 SVD 得 U/S/V（秩 K），喂给 build/scripts/x20_fixed_vs_double.cpp；
  3) 用**真实测量** paper_meas（不是自洽构造）当输入；
  4) 给出「定点 vs 双精度参考」的同源指标（相对 L2 / 相关系数 / PSNR），
     以及与论文链路参考解 paper_recon_{pdhg,fista}.npy 的对照。
只读复用 X20 的量化/驱动/指标；不改任何 HLS/RTL 源码。

用法（仓库根）：
    python build/scripts/x34_paper_chain_samesource.py --ranks 64,96
    python build/scripts/x34_paper_chain_samesource.py --ranks 1044 --iters 256
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

import x20_hls_recon_image as X  # noqa: E402

VEC = os.path.join(REPO, "sim", "vectors")
RESULTS = os.path.join(REPO, "sim", "results")
WORK = os.path.join(REPO, "build", "x34_work")

N = 126
ROWS, COLS = 29, 36
NH = ROWS * COLS
SHAPE = (ROWS, COLS)
CSCALE_R = 312.5           # fig4_column_c.m / paper_recon_pdhg.py 的 R 通道 cscale


def log(msg: str) -> None:
    print(msg, flush=True)


# ---------------------------------------------------------------- 论文链算子

def drows(M):
    o = np.empty_like(M)
    o[:-1, :] = M[:-1, :] - M[1:, :]
    o[-1, :] = M[-1, :]
    return o


def dvec(v):
    o = np.empty_like(v)
    o[:-1] = v[:-1] - v[1:]
    o[-1] = v[-1]
    return o


def rows_of(m2d, order):
    if order == "F":
        return np.ascontiguousarray(m2d.T).ravel(order="C")
    return np.ascontiguousarray(m2d).ravel(order="C")


def col_perm():
    return np.arange(NH).reshape(ROWS, COLS, order="F").ravel(order="C")


def build_paper_operator():
    """按 paper_recon_pdhg.py / x14_recon_corrected_a.py 的 build_dA 逐行重建。"""
    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    A3 = np.ascontiguousarray(A0.reshape(N, N, NH).transpose(1, 0, 2))
    del A0
    dAr = drows(A3.reshape(N * N, NH))
    del A3
    dA = np.ascontiguousarray(dAr[:, col_perm()])
    del dAr
    return dA


def svd_of(dA, K):
    """由 Gram 取前 K 个奇异三元组，返回 (U(M,K), S_norm(K), V(K,N), s_max)。"""
    G = dA.T @ dA
    G = 0.5 * (G + G.T)
    lam, Vc = np.linalg.eigh(G)              # 升序
    order = np.argsort(lam)[::-1]
    lam, Vc = np.maximum(lam[order], 0.0), Vc[:, order]
    lam = lam[:K]
    Vk = Vc[:, :K]
    S = np.sqrt(lam)
    s_max = float(S[0]) if S.size else 0.0
    U = (dA @ Vk) / np.maximum(S[None, :], 1e-300)
    U = U / np.maximum(np.linalg.norm(U, axis=0, keepdims=True), 1e-300)
    V = np.ascontiguousarray(Vk.T)
    return U, (S / s_max if s_max > 0 else S), V, s_max


# ---------------------------------------------------------------- 驱动

def compile_driver(max_rank, exe):
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
    r = subprocess.run(cmd, cwd=REPO, capture_output=True, encoding="utf-8",
                       errors="replace")
    if r.returncode != 0:
        log(r.stdout); log(r.stderr)
        raise SystemExit("编译失败")
    return exe


def choose_s_d(dm):
    """按 Q2.16 量程（|D| <= 1.5）选 2 的幂尺度（X.choose_scales 的推广）。"""
    p = int(math.ceil(math.log2(max(float(np.abs(dm).max()) / 1.5, 1e-30))))
    return float(2 ** max(0, p))


def run_one(U, S, V, s_max, dm, driver, iters, lam_q16, tag):
    s_f = 1.0
    s_d = choose_s_d(dm)
    U_store, S_code, V_store, u_exp, v_exp, s_exp, _ = X.quantize_factors(
        U, S, V, s_max, s_f, s_d)
    D, d_code = X.quantize_measurement(dm, s_d)
    Ud_ref = np.ascontiguousarray(U.T)
    Sd_ref = S * (s_max * s_f / s_d)
    Vd_ref = np.ascontiguousarray(V)
    norm_B = s_max * (s_f / s_d)
    tau = math.sqrt(0.99) / math.sqrt(norm_B * norm_B + 8.0)
    tau_q16 = int(round(tau * 65536.0))
    cfg = dict(n_rows=SHAPE[0], n_cols=SHAPE[1], n_iter=int(iters), min_iters=8,
               residual_every=4, tv_mode=0, tau_q16=tau_q16, sigma_q16=tau_q16,
               theta_q16=65536, lambda_q16=int(lam_q16), rho_stop_q8=0)
    case = os.path.join(WORK, "case_%s.bin" % tag)
    res = os.path.join(WORK, "res_%s.bin" % tag)
    X.write_case(case, cfg, U_store, S_code, V_store, (u_exp, v_exp, s_exp),
                 d_code, Ud_ref, Sd_ref, Vd_ref, D)
    r = subprocess.run([driver, case, res], capture_output=True, encoding="utf-8",
                       errors="replace")
    if r.returncode != 0:
        log(r.stdout); log(r.stderr)
        raise SystemExit("驱动失败 %s" % tag)
    R = X.read_result(res)
    out = dict(fixed=R["fixed"] * s_f, dbl_ref=R["dbl_ref"] * s_f,
               dbl_same=R["dbl_same"] * s_f, n_used=R["n_used"],
               overflow=R["overflow"], s_d=s_d, s_f=s_f, tau_q16=tau_q16,
               norm_B_kernel=norm_B, d_code_max=int(np.abs(d_code).max()),
               S_code_max=int(S_code.max()), n_iter=int(iters),
               lambda_q16=int(lam_q16))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ranks", default="64,96")
    ap.add_argument("--iters", type=int, default=1024)
    ap.add_argument("--lam-q16", default="66")
    ap.add_argument("--sweep-paper-lam", action="store_true",
                    help="另按论文 reg/cs 折算出同源 λ 再跑一档")
    ap.add_argument("--out", default="", help="结果 JSON 路径（默认 sim/results/...）")
    args = ap.parse_args()
    ranks = [int(x) for x in args.ranks.split(",") if x.strip()]
    lam_list = [int(x) for x in args.lam_q16.split(",") if x.strip()]
    os.makedirs(WORK, exist_ok=True)
    t0 = time.time()

    log("[1/4] 重建论文链算子 dA（126x126 墙、M=15876、N=1044）...")
    dA = build_paper_operator()
    log("      dA %s  ‖dA‖_F=%.4f  ‖dA‖_2(首奇异值估)=%.4f"
        % (dA.shape, float(np.linalg.norm(dA)), float(np.linalg.norm(dA[:, 0]))))

    meas = np.load(os.path.join(VEC, "paper_meas.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))
    dm = dvec(rows_of(meas[:, :, 0], "F"))
    ref_pdhg = np.load(os.path.join(VEC, "paper_recon_pdhg.npy"))[:, :, 0]
    ref_fista = np.load(os.path.join(VEC, "paper_recon_fista.npy"))
    log("      测量 dm 长度 %d  max|dm|=%.6g  ‖dm‖=%.6g" % (dm.size, np.abs(dm).max(),
                                                           float(np.linalg.norm(dm))))

    # 算子-参考解一致性自检：dA @ u_ref 与 dm 的残差（同源验证）
    log("[2/4] 同源自检：dA 作用在论文参考解上的残差（含尺度匹配）")
    selfchk = {}
    for nm, ref, cs in (("paper_recon_pdhg", ref_pdhg, CSCALE_R),
                        ("paper_recon_fista", ref_fista, 1.0)):
        u = (ref.astype(float).reshape(-1, order="C")) * cs
        y = dA @ u
        alpha = float(y @ dm) / float(y @ y) if float(y @ y) > 0 else float("nan")
        res0 = X.rel_l2(y, dm)
        resa = X.rel_l2(alpha * y, dm)
        selfchk[nm] = dict(raw_rel_l2=res0, scale_matched_rel_l2=resa,
                           scale_alpha=alpha,
                           pearson_to_gt=X.pearson(
                               u, gt[:, :, 0].astype(float).reshape(-1, order="C")))
        log("      %-18s 原始残差=%.4f  尺度匹配后残差=%.4f（α=%.3g）  r(u,gt)=%+.4f"
            % (nm, res0, resa, alpha, selfchk[nm]["pearson_to_gt"]))

    log("[3/4] 秩上限（把参考解投影到 dA 前 K 个右奇异向量的行空间）")
    G = dA.T @ dA
    G = 0.5 * (G + G.T)
    lam_all, Vc_all = np.linalg.eigh(G)
    order = np.argsort(lam_all)[::-1]
    lam_all = lam_all[order]
    Vc_all = Vc_all[:, order]
    ceil = []
    refs = dict(gt=gt[:, :, 0].astype(float).reshape(-1, order="C"),
                pdhg=ref_pdhg.astype(float).reshape(-1, order="C") * CSCALE_R,
                fista=ref_fista.astype(float).reshape(-1, order="C"))
    for K in [8, 16, 32, 64, 96, 128, 192, 256, 384, 512, 768, 1044]:
        Vk = Vc_all[:, :K]
        row = dict(K=K, energy_frac=float((lam_all[:K].sum()) / lam_all.sum()))
        for tag, x in refs.items():
            p = Vk @ (Vk.T @ x)
            row["corr_" + tag] = X.pearson(p, x)
        ceil.append(row)
    for row in ceil:
        log("      K=%4d E=%.4f corr_gt=%.4f corr_pdhg=%.4f corr_fista=%.4f"
            % (row["K"], row["energy_frac"], row["corr_gt"],
               row["corr_pdhg"], row["corr_fista"]))

    log("[4/4] 定点链路（同一论文链算子 + 真实测量）")
    max_rank = max(ranks)
    driver = compile_driver(max_rank, os.path.join(WORK, "x34_driver.exe"))
    table = []
    sols = {}
    for K in ranks:
        U, S, V, s_max = svd_of(dA, K)
        log("      K=%d  ‖B‖_2=s_max=%.4f  V %s" % (K, s_max, V.shape))
        for lam_q16 in lam_list:
            tag = "pK%04d_l%05d" % (K, lam_q16)
            o = run_one(U, S, V, s_max, dm, driver, args.iters, lam_q16, tag)
            u_fix = o["fixed"]
            u_ref = o["dbl_ref"]
            gtC = refs["gt"]
            rec = dict(K=K, lambda_q16=lam_q16, lambda_=lam_q16 / 65536.0,
                       n_iter=o["n_iter"], n_used=o["n_used"],
                       overflow=o["overflow"], s_d=o["s_d"], s_max=s_max,
                       fixed_vs_dblref=X.metrics(u_fix, u_ref),
                       fixed_vs_dblsame=X.metrics(u_fix, o["dbl_same"]),
                       dblref_vs_gt=dict(X.metrics(u_ref, gtC),
                                         iou=X.iou_binarized(u_ref, gtC)),
                       fixed_vs_gt=dict(X.metrics(u_fix, gtC),
                                        iou=X.iou_binarized(u_fix, gtC)),
                       dblref_vs_pdhg=dict(X.metrics(u_ref, refs["pdhg"]),
                                           pearson=X.pearson(u_ref, refs["pdhg"])),
                       fixed_vs_pdhg=dict(X.metrics(u_fix, refs["pdhg"]),
                                          pearson=X.pearson(u_fix, refs["pdhg"])))
            table.append(rec)
            sols[tag] = dict(fixed=u_fix, dbl_ref=u_ref)
            log("        K=%4d λ_q16=%5d N=%4d ovf=%d | 定点 vs 双精度源 relL2=%.4e r=%.10f PSNR=%.2f"
                " | 双精度源 vs 论文参考 r=%+.4f | vs 真值 r=%+.4f IoU=%.3f"
                % (K, lam_q16, o["n_iter"], o["overflow"],
                   rec["fixed_vs_dblref"]["rel_l2"],
                   rec["fixed_vs_dblref"]["pearson"],
                   rec["fixed_vs_dblref"]["psnr_db"],
                   X.pearson(u_ref, refs["pdhg"]),
                   rec["dblref_vs_gt"]["pearson"], rec["dblref_vs_gt"]["iou"]))
        del U, S, V

    out = dict(
        meta=dict(M=int(dA.shape[0]), N=int(dA.shape[1]), shape=list(SHAPE),
                  operator="fig4_column_c 重建链（126x126 墙）",
                  measurement="paper_meas R 通道，F 序展平 + 相邻行差分",
                  refs=["paper_recon_pdhg.npy", "paper_recon_fista.npy"],
                  ranks=ranks, iters=args.iters, max_rank=max_rank,
                  note="同源 = 论文链自己的 dA + 真实测量；定点 vs 双精度参考由同一驱动给出"),
        ceiling=ceil, self_check=selfchk, table=table,
        elapsed_s=time.time() - t0)
    jp = args.out or os.path.join(RESULTS, "x34_paper_chain_samesource.json")
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    log("已保存 %s（%.1f s）" % (os.path.relpath(jp, REPO), time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
