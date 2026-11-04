# -*- coding: utf-8 -*-
"""X20 —— HLS 定点核出图（本轮：走**真实 .psm 路径**）。

背景：上一轮 X20 因为 `sim/models/model_b.psm` 的 U 因子落盘少一次转置
（相对误差 100%），绕开了 psm，改用现场重算的秩 64 算子（见
sim/results/x20_hls_reconstruction_image.md）。缺陷已修（见
sim/results/x28b_psm_u_transpose_fix.md），本轮改回真实 psm 路径。

链条：X18（定点核）-> X19（资源/时序）-> X20（本文件，出图）。

本文件只写**驱动/编排**，不改任何 HLS 算法核源码（src/fpga/hls/**/*.cpp|hpp、
src/fpga/rtl/*.sv），也不改 build_model.py。它做五件事：
  1) 从 `sim/models/model_b.psm` 按 psm_format.h 的偏移直接读候选 121 的记录，
     还原 U/S/V（psm 路径）；同时从 build/x20_work 缓存取现场重算的秩 64 算子
     （现场路径），两者做 U 级与算子级（B = U diag(S) Vᵀ）定量对照。
  2) 对**两条路径各自**自洽地组装测量，按 Q1.15 + 2 的幂 / Q2.16 量化。
  3) 编译并调用 build/scripts/x20_fixed_vs_double.cpp，每个算子源三次运行：
         A 定点          （HLS 核，码字因子 + Q2.16 测量）
         B 双精度(同码字)
         C 双精度(参考)  （未量化因子 + 未量化测量）
  4) 汇总定量指标（含**跨路径**：psm 定点 vs 现场重算双精度参考）。
  5) 出对照图、写 sim/results/x20_hls_recon_image.json。

用法（在仓库根执行）：
    python build/scripts/x20_hls_recon_image.py
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir))
MODEL = os.path.join(REPO, "src", "model")
if MODEL not in sys.path:
    sys.path.insert(0, MODEL)

from penumbrasight_model import psm, quantize as Q, forward_loc as FL  # noqa: E402
from penumbrasight_model import build_model as BM                      # noqa: E402

WORK = os.path.join(REPO, "build", "x20_work")
RESULTS = os.path.join(REPO, "sim", "results")
PSM_PATH = os.path.join(REPO, "sim", "models", "model_b.psm")
MANIFEST_PATH = os.path.join(REPO, "sim", "models", "model_b.manifest.json")
FIELD_CACHE = os.path.join(WORK, "cand121_rank64.npz")
OLD_JSON = os.path.join(RESULTS, "x20_hls_recon_image.json")
CAND = 121                     # B 档候选网格 (4,4,1) = x 0 mm / y 570 mm / z 0 mm
SHAPE = (18, 24)               # (行 = z, 列 = x)
RANK = 64
PEAK = 2.0                     # 真值峰值（真值归一化到 [0,1]，peak = 2*max|gt|）


# ------------------------------------------------------------------ 小工具

def log(msg: str) -> None:
    print(msg, flush=True)


def rel_l2(a, b):
    a = np.asarray(a, float).ravel()
    b = np.asarray(b, float).ravel()
    den = float(b @ b)
    return math.sqrt(float(((a - b) ** 2).sum()) / den) if den > 0 else float("nan")


def pearson(a, b):
    a = np.asarray(a, float).ravel() - np.mean(a)
    b = np.asarray(b, float).ravel() - np.mean(b)
    d = np.linalg.norm(a) * np.linalg.norm(b)
    return float(a @ b / d) if d > 0 else float("nan")


def psnr(a, b, peak):
    mse = float(np.mean((np.asarray(a, float) - np.asarray(b, float)) ** 2))
    return float("inf") if mse <= 0 else 10.0 * math.log10(peak * peak / mse)


def metrics(a, b, peak=PEAK):
    a = np.asarray(a, float).ravel()
    b = np.asarray(b, float).ravel()
    return dict(rel_l2=rel_l2(a, b), pearson=pearson(a, b),
                psnr_db=psnr(a, b, peak), max_abs=float(np.max(np.abs(a - b))))


def iou_binarized(a, ref):
    a = np.asarray(a, float).ravel()
    r = np.asarray(ref, float).ravel()
    frac = float((r > 0).mean())
    if frac <= 0:
        return float("nan")
    thr = float(np.quantile(a, 1.0 - frac))
    A = a >= thr
    B = r > 0
    un = np.logical_or(A, B).sum()
    return float(np.logical_and(A, B).sum() / un) if un else 0.0


def resize_avg(a, out_shape):
    """块平均重采样（把 29x36 的论文真值降到 B 档 18x24）。"""
    oz, ox = out_shape
    iz, ix = a.shape
    zs = np.linspace(0, iz, oz + 1)
    xs = np.linspace(0, ix, ox + 1)
    out = np.zeros(out_shape, float)
    for i in range(oz):
        for j in range(ox):
            z0, z1 = int(round(zs[i])), int(round(zs[i + 1]))
            x0, x1 = int(round(xs[j])), int(round(xs[j + 1]))
            z1 = max(z1, z0 + 1)
            x1 = max(x1, x0 + 1)
            out[i, j] = a[z0:z1, x0:x1].mean()
    return out


# ------------------------------------------------------------------ 算子源

def build_field_operator():
    """现场路径：从 B 档几何重算候选 121 的差分算子并取秩 64 截断 SVD。

    U 形如 (M, rank)，V 形如 (rank, N)，与 build_model.truncated_svd_via_gram 一致。
    """
    if os.path.exists(FIELD_CACHE):
        z = np.load(FIELD_CACHE)
        log("[1a/6] 现场路径：复用缓存算子 %s" % os.path.relpath(FIELD_CACHE, REPO))
        return z["U"], z["S"], z["V"], float(z["s_max"])
    log("[1a/6] 现场路径：从 B 档几何重算候选 %d 的差分算子并取秩 %d 截断 SVD ..."
        % (CAND, RANK))
    st = FL.b_loc_setup()
    grid_spec, _ = FL.b_candidate_grid()
    grid = BM._candidate_grid(grid_spec)
    _, _, _, ll = grid[CAND]
    plan = FL.make_plan(st, 2, downsamp_factor=0, mz_mirrored=False)
    A = FL.build_A_from_plan(plan, ll)
    npx = int(round(math.sqrt(A.shape[0])))
    B = BM.apply_row_diff(A, npx, axis=0)
    U, S_norm, V, s_max = BM.truncated_svd_via_gram(B, RANK)
    log("      A %s -> B %s  ||B||_F=%.4f  s_max=%.4f"
        % (A.shape, B.shape, float(np.linalg.norm(B)), s_max))
    os.makedirs(WORK, exist_ok=True)
    np.savez(FIELD_CACHE, U=U, S=S_norm, V=V, s_max=s_max)
    return U, S_norm, V, float(s_max)


def load_psm_operator():
    """psm 路径：按 psm_format.h 的偏移直接读候选 121 的记录，还原 U/S/V。

    完全走 psm.py 的头部/记录解析（偏移是契约），不做任何现场重算。
    返回 (U(M,rank), S_norm(rank,), V(rank,N), meta)。
    """
    log("[1b/6] psm 路径：直接读 %s 的候选 %d ..." % (os.path.relpath(PSM_PATH, REPO), CAND))
    if not os.path.exists(PSM_PATH):
        raise SystemExit("找不到 %s" % PSM_PATH)
    with open(PSM_PATH, "rb") as f:
        hdr = psm.PsmHeader.unpack(f.read(psm.HEADER_BYTES))
        if not (0 <= CAND < hdr.candidates):
            raise SystemExit("候选下标越界")
        base = psm.HEADER_BYTES + CAND * hdr.record_bytes
        f.seek(base)
        raw = f.read(hdr.record_bytes)
        if len(raw) != hdr.record_bytes:
            raise SystemExit("候选记录读不满")
    rec = psm._unpack_record(hdr, raw)
    rec.validate(hdr)

    U = rec.U.astype(np.float64).T * rec.u_scale        # (rank,M)ᵀ -> (M,rank)
    S_norm = rec.S.astype(np.float64) * rec.s_scale     # 归一化奇异值（max=1）
    V = rec.V.astype(np.float64) * rec.v_scale          # (rank,N)

    # U 列正交性自检：这是「布局对 + 元素填满」的直接证据（也复刻 verify_psm_u_layout）。
    g = rec.U.astype(np.float64) * rec.u_scale          # (rank,M)
    gram = g @ g.T
    diag_dev = float(np.max(np.abs(np.diag(gram) - 1.0)))
    off_dev = float(np.max(np.abs(gram - np.diag(np.diag(gram)))))
    nonzero_cols = int((np.abs(rec.U).sum(axis=0) != 0).sum())
    log("      头部：meas %dx%d  hidden %dx%d  candidates=%d  rank_max=%d  "
        "geom=%08X  crc=%08X"
        % (hdr.meas_h, hdr.meas_w, hdr.hidden_h, hdr.hidden_w, hdr.candidates,
           hdr.rank_max, hdr.geometry_hash, hdr.payload_crc32))
    log("      候选 %d：xyz(mm)=(%d,%d,%d)  u_scale=2^%d  s_scale=2^%d  v_scale=2^%d"
        % (CAND, rec.x_mm, rec.wall_distance_mm, rec.z_mm,
           int(round(math.log2(rec.u_scale))), int(round(math.log2(rec.s_scale))),
           int(round(math.log2(rec.v_scale)))))
    log("      U 非零列 %d/%d  Gram 对角偏差=%.4e  非对角偏差=%.4e"
        % (nonzero_cols, rec.U.shape[1], diag_dev, off_dev))

    meta = dict(u_scale=rec.u_scale, s_scale=rec.s_scale, v_scale=rec.v_scale,
                u_exp=int(round(math.log2(rec.u_scale))),
                s_exp=int(round(math.log2(rec.s_scale))),
                v_exp=int(round(math.log2(rec.v_scale))),
                x_mm=rec.x_mm, wall_distance_mm=rec.wall_distance_mm, z_mm=rec.z_mm,
                geometry_hash=hdr.geometry_hash, payload_crc32=hdr.payload_crc32,
                candidates=hdr.candidates, rank_max=hdr.rank_max,
                nonzero_cols=nonzero_cols, gram_diag_dev=diag_dev, gram_off_dev=off_dev)
    return U, S_norm, V, meta


def load_psm_s_max():
    """psm 不存 s_max（只在 manifest），这是独立于本次修复的一个接口缺口。"""
    with open(MANIFEST_PATH, "r", encoding="utf-8") as f:
        man = json.load(f)
    m = man["candidates_meta"][CAND]
    return float(m["s_max"]), man


# ------------------------------------------------------------------ 对照（psm vs 现场）

def compare_operators(fld, ps):
    """U 级 + 算子级（B = U diag(S) Vᵀ）定量对照。口径同上一轮：相对 L2 的分母是
    右侧参考量（这里一律以**现场重算**为参考）、相关系数为去均值归一化内积、
    max|Δ| 为逐元素最大绝对差。"""
    Uf, Sf, Vf, smf = fld
    Up, Sp, Vp, smp = ps
    out = {}
    # U 级
    dot = (Up * Uf).sum(axis=0)
    out["u_sign_flips"] = int((dot < 0).sum())
    sgn = np.sign(dot)
    sgn[sgn == 0] = 1.0
    Up_a = Up * sgn[None, :]
    out["u_direct"] = metrics(Up, Uf)
    out["u_signaligned"] = metrics(Up_a, Uf)
    out["u_gram_psm"] = float(np.max(np.abs(Up.T @ Up - np.eye(Up.shape[1]))))
    out["u_gram_field"] = float(np.max(np.abs(Uf.T @ Uf - np.eye(Uf.shape[1]))))
    out["u_col_rel_l2"] = [float(np.linalg.norm(Up[:, k] - Uf[:, k]) /
                                 np.linalg.norm(Uf[:, k])) for k in range(Up.shape[1])]
    # S / V 级
    out["s"] = metrics(Sp, Sf)
    out["v"] = metrics(Vp, Vf)
    # 算子级
    Bp = (Up * (Sp * smp)[None, :]) @ Vp
    Bf = (Uf * (Sf * smf)[None, :]) @ Vf
    out["b"] = metrics(Bp, Bf)
    out["b_norm_psm"] = float(np.linalg.norm(Bp))
    out["b_norm_field"] = float(np.linalg.norm(Bf))
    out["b_specnorm_psm"] = float(np.linalg.norm(Bp, 2))
    out["b_specnorm_field"] = float(np.linalg.norm(Bf, 2))
    return out, Bp, Bf


# ------------------------------------------------------------------ 真值与测量

def build_target():
    gt = np.load(os.path.join(REPO, "sim", "vectors", "paper_gt.npy"))
    f2d = resize_avg(gt[:, :, 0].astype(float), SHAPE)
    f2d = f2d / f2d.max()                     # 归一化到 [0,1]
    F = f2d.reshape(-1)                       # 扁平索引 = z*W + x
    log("[2/6] 隐藏场景真值：max=%.4f  ||f||_2=%.4f  非零像素 %d/%d"
        % (f2d.max(), float(np.linalg.norm(F)), int((f2d > 0.2).sum()), F.size))
    return f2d, F


def measurement_for(U, S_norm, V, s_max, F):
    B_eff = (U * (S_norm * s_max)[None, :]) @ V
    d_unit = B_eff @ F
    return B_eff, d_unit


# ------------------------------------------------------------------ 选尺度 / 量化

def choose_scales(F, d_unit, s_max):
    s_f = 1.0
    cand = []
    for p in range(0, 13):
        s_d = float(2 ** p)
        Bk = s_max * (s_f / s_d)
        Dmax = float(np.abs(d_unit).max()) / s_d
        zmax = float(np.linalg.norm(F / s_f))
        ok = (Dmax <= 1.5) and (Bk * zmax <= 24.0)
        cand.append((s_d, Bk, Dmax, ok))
        if ok:
            break
    s_d = cand[-1][0]
    assert cand[-1][3], "找不到不溢出的 2 的幂尺度"
    return s_f, s_d


def quantize_factors(U, S_norm, V, s_max, s_f, s_d):
    c = s_f / s_d
    U_code, u_exp = _pow2_codes(U, 16)
    V_code, v_exp = _pow2_codes(V, 16)
    S_scaled = S_norm * (s_max * c)
    s_exp = int(math.ceil(math.log2(max(float(S_scaled.max()) / 65535.0, 1e-300))))
    S_code = np.clip(np.rint(S_scaled / (2.0 ** s_exp)), 0, 65535).astype(np.uint16)
    U_store = np.ascontiguousarray(U_code.T.astype(np.int16))     # (rank, M)
    V_store = np.ascontiguousarray(V_code.astype(np.int16))       # (rank, N)
    return U_store, S_code, V_store, u_exp, v_exp, s_exp, S_scaled


def _pow2_codes(x, bits):
    hi = (1 << (bits - 1)) - 1
    m = float(np.max(np.abs(x)))
    if m <= 0:
        scale = 1.0
    else:
        ideal = m / hi
        scale = float(math.ldexp(1.0, int(math.ceil(math.log2(ideal)))))
    codes = np.clip(np.rint(np.asarray(x, float) / scale), -hi, hi)
    return codes, int(round(math.log2(scale)))


def quantize_measurement(d_unit, s_d):
    D = d_unit / s_d
    d_code = np.clip(np.rint(D * 65536.0), -131071, 131071).astype(np.int32)
    return D, d_code


# ------------------------------------------------------------------ case / 驱动

def write_case(path, cfg, U_store, S_code, V_store, exps, d_code,
               Ud_ref, Sd_ref, Vd_ref, d_real):
    hdr = [U_store.shape[0], U_store.shape[1], V_store.shape[1],
           cfg["n_rows"], cfg["n_cols"], cfg["n_iter"], cfg["min_iters"],
           cfg["residual_every"], cfg["tv_mode"], cfg["tau_q16"], cfg["sigma_q16"],
           cfg["theta_q16"], cfg["lambda_q16"], cfg["rho_stop_q8"], 0]
    with open(path, "wb") as f:
        f.write(b"X20C")
        f.write(struct.pack("<%di" % len(hdr), *hdr))
        f.write(struct.pack("<3i", *exps))
        f.write(U_store.astype("<i2").tobytes(order="C"))
        f.write(S_code.astype("<u2").tobytes(order="C"))
        f.write(V_store.astype("<i2").tobytes(order="C"))
        f.write(d_code.astype("<i4").tobytes(order="C"))
        f.write(np.ascontiguousarray(Ud_ref, "<f8").tobytes(order="C"))
        f.write(np.ascontiguousarray(Sd_ref, "<f8").tobytes(order="C"))
        f.write(np.ascontiguousarray(Vd_ref, "<f8").tobytes(order="C"))
        f.write(np.ascontiguousarray(d_real, "<f8").tobytes(order="C"))


def read_result(path):
    with open(path, "rb") as f:
        magic = f.read(4)
        assert magic == b"X20R", magic
        hd = struct.unpack("<4i", f.read(16))
        N = hd[0]
        fx = np.frombuffer(f.read(8 * N), "<f8").copy()
        fb = np.frombuffer(f.read(8 * N), "<f8").copy()
        fc = np.frombuffer(f.read(8 * N), "<f8").copy()
    return {"n_used": hd[2], "overflow": hd[3], "fixed": fx, "dbl_same": fb,
            "dbl_ref": fc}


def compile_driver():
    exe = os.path.join(WORK, "x20_driver.exe")
    src = os.path.join(HERE, "x20_fixed_vs_double.cpp")
    cmd = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wno-unknown-pragmas",
           "-Isrc/hardware", "-Isrc/software/include", "-Isrc/fpga/hls",
           "-Isrc/fpga/hls/pdhg_core", "-Isrc/fpga/hls/lr_operator",
           src,
           "src/fpga/hls/pdhg_core/pdhg_core_fixed.cpp",
           "src/fpga/hls/lr_operator/lr_operator_fixed.cpp",
           "src/fpga/hls/pdhg_core/pdhg_core.cpp",
           "src/fpga/hls/lr_operator/lr_operator.cpp",
           "-o", exe]
    log("[4/6] 编译驱动 ...")
    r = subprocess.run(cmd, cwd=REPO, capture_output=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        log(r.stdout); log(r.stderr)
        raise SystemExit("编译失败")
    return exe


# ------------------------------------------------------------------ 跑一条算子源

def run_source(tag, U, S_norm, V, s_max, F, gt_flat, configs, driver, extra_meta=None):
    log("---- 算子源 [%s] ----" % tag)
    B_eff, d_unit = measurement_for(U, S_norm, V, s_max, F)
    s_f, s_d = choose_scales(F, d_unit, s_max)
    U_store, S_code, V_store, u_exp, v_exp, s_exp, S_scaled = quantize_factors(
        U, S_norm, V, s_max, s_f, s_d)
    D, d_code = quantize_measurement(d_unit, s_d)
    Ud_ref = np.ascontiguousarray(U.T)
    Sd_ref = S_norm * (s_max * s_f / s_d)
    Vd_ref = np.ascontiguousarray(V)
    d_real = D

    Bq = (U_store.astype(float) * (2.0 ** u_exp)).T @ \
         ((S_code.astype(float) * (2.0 ** s_exp))[:, None] * (V_store.astype(float) * (2.0 ** v_exp)))
    Bref = (Ud_ref.T * Sd_ref[None, :]) @ Vd_ref
    quant_B = rel_l2(Bq, Bref)
    quant_d = rel_l2(d_code.astype(float) / 65536.0, d_real)
    norm_B = float(np.linalg.norm(Bq, 2))
    log("      s_d=%.0f  u_exp=%d v_exp=%d s_exp=%d  S_code max=%d  d_code max=%d"
        % (s_d, u_exp, v_exp, s_exp, int(S_code.max()), int(np.abs(d_code).max())))
    log("      量化误差：B_kernel 相对 L2=%.4e  测量 相对 L2=%.4e  ||B_kernel||_2=%.6f"
        % (quant_B, quant_d, norm_B))

    tau = math.sqrt(0.99) / math.sqrt(norm_B * norm_B + 8.0)
    tau_q16 = int(round(tau * 65536.0))
    base = dict(n_rows=SHAPE[0], n_cols=SHAPE[1], min_iters=8, residual_every=4,
                tv_mode=0, tau_q16=tau_q16, sigma_q16=tau_q16, theta_q16=65536,
                rho_stop_q8=0)
    sols, runs = {}, {}
    for name, cfg in configs:
        c = dict(base); c.update(cfg)
        case = os.path.join(WORK, "case_%s_%s.bin" % (tag, name))
        res = os.path.join(WORK, "res_%s_%s.bin" % (tag, name))
        write_case(case, c, U_store, S_code, V_store, (u_exp, v_exp, s_exp), d_code,
                   Ud_ref, Sd_ref, Vd_ref, d_real)
        r = subprocess.run([driver, case, res], capture_output=True,
                           encoding="utf-8", errors="replace")
        if r.returncode != 0:
            log(r.stdout); log(r.stderr)
            raise SystemExit("驱动失败：%s/%s" % (tag, name))
        if name == "main":
            log("      --- 驱动 stdout（%s/main）---" % tag)
            for line in r.stdout.strip().splitlines():
                log("      " + line)
        R = read_result(res)
        f_fix = R["fixed"] * s_f
        f_ref = R["dbl_ref"] * s_f
        f_same = R["dbl_same"] * s_f
        gt = gt_flat * s_f
        sols[name] = dict(fixed=f_fix, dbl_ref=f_ref, dbl_same=f_same)
        runs[name] = dict(
            lambda_q16=c["lambda_q16"], lambda_=c["lambda_q16"] / 65536.0,
            n_iter=c["n_iter"], n_used=R["n_used"], overflow=R["overflow"],
            fixed_vs_dblref=metrics(f_fix, f_ref),
            fixed_vs_dblsame=metrics(f_fix, f_same),
            dblsame_vs_dblref=metrics(f_same, f_ref),
            fixed_vs_gt=dict(metrics(f_fix, gt), iou=iou_binarized(f_fix, gt)),
            dblref_vs_gt=dict(metrics(f_ref, gt), iou=iou_binarized(f_ref, gt)),
            u_inf=float(np.max(np.abs(f_fix))))
        rec = runs[name]
        log("      %-8s λ=%.2e N=%4d  定点vs双精度(本源) relL2=%.3e r=%.6f PSNR=%.2f | "
            "定点vs真值 r=%.4f"
            % (name, rec["lambda_"], rec["n_iter"], rec["fixed_vs_dblref"]["rel_l2"],
               rec["fixed_vs_dblref"]["pearson"], rec["fixed_vs_dblref"]["psnr_db"],
               rec["fixed_vs_gt"]["pearson"]))
    meta = dict(tag=tag, s_f=s_f, s_d=s_d, exps=dict(u=u_exp, v=v_exp, s=s_exp),
                S_code_max=int(S_code.max()), d_code_max=int(np.abs(d_code).max()),
                quant_rel_B=quant_B, quant_rel_d=quant_d,
                norm_B_kernel=norm_B, tau_q16=tau_q16, tau=tau,
                s_max=s_max, B_eff_norm=float(np.linalg.norm(B_eff)))
    if extra_meta:
        meta.update(extra_meta)
    return runs, sols, meta


# ------------------------------------------------------------------ 出图

def _setup_font():
    import warnings
    import matplotlib
    from matplotlib import font_manager
    # 题目要求显式指定 Noto Sans CJK SC / WenQuanYi Micro Hei；本机未装这两个时
    # 退到同族 Noto Sans SC（Noto 简体中文字形），再退 Windows 自带雅黑/黑体。
    prefer = ["Noto Sans CJK SC", "WenQuanYi Micro Hei", "Noto Sans SC",
              "Microsoft YaHei", "SimHei"]
    have = {f.name for f in font_manager.fontManager.ttflist}
    chosen = next((p for p in prefer if p in have), None)
    matplotlib.rcParams["font.sans-serif"] = prefer + ["DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    warnings.filterwarnings("ignore")
    return chosen if chosen else "（未找到任何 CJK 字体）"


def make_figures(sols, runs, f2d, cmp, Bp, Bf, psm_meta, fld_meta):
    font_used = _setup_font()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def panel(ax, img, title):
        ax.imshow(img, cmap="gray", interpolation="nearest")
        ax.set_title(title, fontsize=8.5)
        ax.set_xticks([]); ax.set_yticks([])

    # ---- 主对照图：psm 路径定点重建 vs 现场重算双精度参考 ----
    p_fix = sols["psm"]["main"]["fixed"]
    p_ref = sols["psm"]["main"]["dbl_ref"]
    f_ref = sols["fld"]["main"]["dbl_ref"]
    gt2 = f2d
    cross = metrics(p_fix, f_ref)
    internal = metrics(p_fix, p_ref)
    fig, axes = plt.subplots(2, 3, figsize=(14.0, 7.6))
    panel(axes[0][0], gt2, "隐藏场景真值（paper_gt R 重采样到 18x24）")
    panel(axes[0][1], f_ref.reshape(SHAPE),
          "现场重算算子 · 双精度参考\nr_vs_gt=%.4f" % runs["fld"]["main"]["dblref_vs_gt"]["pearson"])
    panel(axes[0][2], p_fix.reshape(SHAPE),
          "psm 算子 · 定点链路（λ=1.0e-3, N=1024）\nr_vs_gt=%.4f"
          % runs["psm"]["main"]["fixed_vs_gt"]["pearson"])
    panel(axes[1][0], p_ref.reshape(SHAPE),
          "psm 算子 · 双精度（自洽参考）\nrelL2_fixed=%.2e" % internal["rel_l2"])
    panel(axes[1][1], (p_fix - f_ref).reshape(SHAPE),
          "psm 定点 − 现场重算双精度参考\nrelL2=%.2e  r=%.6f"
          % (cross["rel_l2"], cross["pearson"]))
    panel(axes[1][2], (p_fix - p_ref).reshape(SHAPE),
          "psm 定点 − psm 双精度\nrelL2=%.2e" % internal["rel_l2"])
    fig.suptitle("X20（走真实 .psm 路径）HLS 定点核重建隐藏场景（B 档候选 %d，秩 %d；中文字体 %s）"
                 % (CAND, RANK, font_used), fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    png = os.path.join(RESULTS, "x20_psm_recon.png")
    fig.savefig(png, dpi=140)
    plt.close(fig)
    log("      已保存 %s" % os.path.relpath(png, REPO))

    # ---- 算子对照图：U 与 B 的 psm / 现场 / 差值 ----
    P = np.load(FIELD_CACHE)
    Uf = P["U"]
    # 重新从 psm 读出（避免把大数组在模块间传）
    Up, Sp, Vp, _m = load_psm_operator()

    def panel_wide(ax, img, title, cmap="gray", vlim=None):
        # U 是 (9120, 64)、B 是 (9120, 432)，是「极高极窄」的矩阵，必须放宽纵横比
        # 才能看到结构；差值图用对称限幅的双向色标，否则被大值域压成一片灰。
        if vlim is None:
            ax.imshow(img, cmap=cmap, interpolation="nearest", aspect="auto")
        else:
            ax.imshow(img, cmap=cmap, interpolation="nearest", aspect="auto",
                      vmin=-vlim, vmax=vlim)
        ax.set_title(title, fontsize=8.5)
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_ylabel("测量索引 m / 像素索引", fontsize=7)

    dU = Up - Uf
    dB = Bp - Bf
    fig2, ax2 = plt.subplots(2, 3, figsize=(14.0, 7.0))
    panel_wide(ax2[0][0], Up, "psm 的 U（%dx%d）" % Up.shape)
    panel_wide(ax2[0][1], Uf, "现场重算的 U（%dx%d）" % Uf.shape)
    panel_wide(ax2[0][2], dU, "ΔU = psm − 现场\nrelL2=%.3e  max|Δ|=%.2e"
               % (cmp["u_direct"]["rel_l2"], cmp["u_direct"]["max_abs"]),
               cmap="bwr", vlim=float(np.max(np.abs(dU))))
    panel_wide(ax2[1][0], Bp, "psm 的 B = U diag(S) V^T（%dx%d）" % Bp.shape)
    panel_wide(ax2[1][1], Bf, "现场重算的 B（%dx%d）" % Bf.shape)
    panel_wide(ax2[1][2], dB, "ΔB = psm − 现场\nrelL2=%.3e  r=%.8f"
               % (cmp["b"]["rel_l2"], cmp["b"]["pearson"]),
               cmap="bwr", vlim=float(np.max(np.abs(dB))))
    fig2.suptitle("从 psm 读出的算子 vs 现场重算（B 档候选 %d，秩 %d；中文字体 %s）"
                  % (CAND, RANK, font_used), fontsize=10.5)
    fig2.tight_layout(rect=(0, 0, 1, 0.94))
    png2 = os.path.join(RESULTS, "x20_psm_u_compare.png")
    fig2.savefig(png2, dpi=140)
    plt.close(fig2)
    log("      已保存 %s" % os.path.relpath(png2, REPO))

    # ---- λ 扫描图（psm 路径）----
    names = [n for n in runs["psm"] if n.startswith("sweep_") or n in ("main", "lam0")]
    lams = [runs["psm"][n]["lambda_"] for n in names]
    rgt = [runs["psm"][n]["fixed_vs_gt"]["pearson"] for n in names]
    rref = [runs["psm"][n]["fixed_vs_dblref"]["rel_l2"] for n in names]
    order = np.argsort(lams)
    lams = [lams[i] for i in order]; rgt = [rgt[i] for i in order]; rref = [rref[i] for i in order]
    fig3, ax3 = plt.subplots(1, 2, figsize=(11.0, 4.2))
    ax3[0].semilogx([x + 1e-7 for x in lams], rgt, "o-")
    ax3[0].set_xlabel("λ"); ax3[0].set_ylabel("与真值的相关系数 r")
    ax3[0].set_title("λ 扫描（psm 算子，N=1024，定点链路）"); ax3[0].grid(alpha=0.3, which="both")
    ax3[1].semilogx([x + 1e-7 for x in lams], rref, "s-", color="crimson")
    ax3[1].set_xlabel("λ"); ax3[1].set_ylabel("定点 vs 双精度（本源）相对 L2")
    ax3[1].set_title("λ 扫描：psm 路径定点与双精度的偏离"); ax3[1].grid(alpha=0.3, which="both")
    fig3.tight_layout()
    png3 = os.path.join(RESULTS, "x20_psm_lambda_and_error.png")
    fig3.savefig(png3, dpi=140)
    plt.close(fig3)
    log("      已保存 %s" % os.path.relpath(png3, REPO))
    return font_used, png, png2, png3


# ------------------------------------------------------------------ 主流程

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-build", action="store_true", help="（保留参数；本轮不再重算算子）")
    args = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)
    t0 = time.time()
    log("=" * 88)
    log("X20（走真实 .psm 路径）HLS 定点核出图")
    log("=" * 88)

    fld = build_field_operator()
    Uf, Sf, Vf, smf = fld
    Up, Sp, Vp, psm_meta = load_psm_operator()
    sm_psm, man = load_psm_s_max()
    log("      psm 的 s_max（来自 manifest，不在 psm 内）=%.6f" % sm_psm)
    psm = (Up, Sp, Vp, sm_psm)

    log("[3/6] U 级 / 算子级对照（参考 = 现场重算）...")
    cmp, Bp, Bf = compare_operators(fld, psm)
    log("      U  : relL2=%.4e  r=%.10f  max|Δ|=%.4e  符号翻转列=%d"
        % (cmp["u_direct"]["rel_l2"], cmp["u_direct"]["pearson"],
           cmp["u_direct"]["max_abs"], cmp["u_sign_flips"]))
    log("      S  : relL2=%.4e      V : relL2=%.4e" % (cmp["s"]["rel_l2"], cmp["v"]["rel_l2"]))
    log("      B  : relL2=%.4e  r=%.10f  max|Δ|=%.4e  ||B||_F=%.4f"
        % (cmp["b"]["rel_l2"], cmp["b"]["pearson"], cmp["b"]["max_abs"], cmp["b_norm_psm"]))
    log("      U 列 Gram 偏差：psm=%.4e  现场=%.4e"
        % (cmp["u_gram_psm"], cmp["u_gram_field"]))

    f2d, F = build_target()
    driver = compile_driver()

    psm_configs = [("main", {"n_iter": 1024, "lambda_q16": 66}),
                   ("lam0", {"n_iter": 1024, "lambda_q16": 0}),
                   ("nit16", {"n_iter": 16, "lambda_q16": 66}),
                   ("nit256", {"n_iter": 256, "lambda_q16": 66}),
                   ("nit4096", {"n_iter": 4096, "lambda_q16": 66})]
    for lam_q16 in (1, 3, 7, 33, 328, 655):
        psm_configs.append(("sweep_l%05d" % lam_q16, {"n_iter": 1024, "lambda_q16": lam_q16}))
    fld_configs = [("main", {"n_iter": 1024, "lambda_q16": 66}),
                   ("lam0", {"n_iter": 1024, "lambda_q16": 0}),
                   ("nit16", {"n_iter": 16, "lambda_q16": 66}),
                   ("nit4096", {"n_iter": 4096, "lambda_q16": 66})]

    log("[5/6] 跑定点/双精度（两条算子源）...")
    runs_psm, sols_psm, meta_psm = run_source(
        "psm", Up, Sp, Vp, sm_psm, F, f2d, psm_configs, driver, extra_meta=psm_meta)
    runs_fld, sols_fld, meta_fld = run_source(
        "fld", Uf, Sf, Vf, smf, F, f2d, fld_configs, driver)

    # 跨路径对照（都换算到场景单位）
    cross = {
        "psm_fixed_vs_field_dblref": metrics(sols_psm["main"]["fixed"], sols_fld["main"]["dbl_ref"]),
        "psm_fixed_vs_field_fixed": metrics(sols_psm["main"]["fixed"], sols_fld["main"]["fixed"]),
        "psm_dblref_vs_field_dblref": metrics(sols_psm["main"]["dbl_ref"], sols_fld["main"]["dbl_ref"]),
        "psm_fixed_vs_field_dblref_gt_iou": iou_binarized(sols_psm["main"]["fixed"], f2d),
    }
    log("[5b] 跨路径：psm 定点 vs 现场双精度参考 relL2=%.4e r=%.10f PSNR=%.2f dB"
        % (cross["psm_fixed_vs_field_dblref"]["rel_l2"],
           cross["psm_fixed_vs_field_dblref"]["pearson"],
           cross["psm_fixed_vs_field_dblref"]["psnr_db"]))

    log("[6/6] 出图 ...")
    font_used, png, png2, png3 = make_figures(
        {"psm": sols_psm, "fld": sols_fld},
        {"psm": runs_psm, "fld": runs_fld}, f2d, cmp, Bp, Bf, psm_meta, meta_fld)

    out = {
        "meta": dict(cand=CAND, rank=RANK, shape=list(SHAPE),
                     s_max_field=smf, s_max_psm=sm_psm,
                     psm_path=os.path.relpath(PSM_PATH, REPO),
                     manifest=os.path.relpath(MANIFEST_PATH, REPO),
                     psm_header=psm_meta.get("geometry_hash"),
                     font=font_used, elapsed_s=time.time() - t0,
                     note="本轮走真实 .psm 路径；s_max 需从 manifest 补回（psm 未存）"),
        "u_compare": cmp,
        "cross": cross,
        "sources": {"psm": dict(meta=meta_psm, runs=runs_psm),
                    "field": dict(meta=meta_fld, runs=runs_fld)},
    }
    # 沿用上一轮的秩上限分析（本轮未重算，保留既有结论）
    if os.path.exists(OLD_JSON):
        try:
            old = json.load(open(OLD_JSON, encoding="utf-8"))
            if "meta" in old and "rank_ceiling" in old["meta"]:
                out["meta"]["rank_ceiling"] = old["meta"]["rank_ceiling"]
                out["meta"]["rank_ceiling_note"] = "沿用上一轮（算子几何未变）"
        except Exception:
            pass
    out["figures"] = dict(recon=os.path.relpath(png, REPO),
                          u_compare=os.path.relpath(png2, REPO),
                          lambda_err=os.path.relpath(png3, REPO),
                          rank_ceiling=os.path.relpath(
                              os.path.join(RESULTS, "x20_rank_ceiling.png"), REPO))
    jp = os.path.join(RESULTS, "x20_hls_recon_image.json")
    with open(jp, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    log("      已保存 %s" % os.path.relpath(jp, REPO))
    log("总用时 %.1f s" % (time.time() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
