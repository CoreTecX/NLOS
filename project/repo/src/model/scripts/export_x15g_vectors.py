# -*- coding: utf-8 -*-
"""X15g 硬件向量导出：给 recon.hpp 的 M5 ADMM 内核做主机侧对照。

导出内容
--------
    x15g_ata_tri.bin   float64[n(n+1)/2]   A_dᵀA_d 的上三角（行优先，(i,j) i<=j）
    x15g_aty.bin       float64[n]          A_dᵀy
    x15g_ref_u.bin     float64[n]          Python ADMM 的参考解（定案参数）
    x15g_params.txt    文本参数与偏移约定

用哪份问题
----------
δ=0 的**自洽合成**问题（数据由 paper A 自己生成），理由：
  - 与 X15g 报告的 S2 第一行是同一个问题，参考解可交叉核对；
  - 真实数据那份的参考解受 A 的误差影响，不适合当数值对照基准。

定案参数（X15g 第五节）
----------------------
    λ   = REG_R / CSCALE_R = 160000
    ρ   = 0.1 · mean(diag(A_dᵀA_d))
    外层 = 400（固定）    内层 CG = 15（固定）

用法
----
    python src/model/scripts/export_x15g_vectors.py
"""

from __future__ import annotations

import argparse
import io
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..")))

# 复用 X15g 的算子与求解器，保证参考解与报告一致
sys.path.insert(0, HERE)
from x15g_model_error_tolerance import (  # noqa: E402
    CSCALE_R, REG_R, build_dA, admm_cg, make_problem, _grad, _div,
)

VEC = os.path.abspath(os.path.join(HERE, "..", "..", "..", "sim", "vectors"))
HW = os.path.abspath(os.path.join(HERE, "..", "..", "hardware", "hwdata"))


def tri_pack(ATA):
    """行优先上三角打包，与 recon.hpp 的 tri_offset() 同一约定。"""
    n = ATA.shape[0]
    iu = np.triu_indices(n)          # 行优先：(0,0),(0,1),...
    out = ATA[iu]
    assert out.size == n * (n + 1) // 2
    return np.ascontiguousarray(out, dtype=np.float64)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=HW)
    ap.add_argument("--n-outer", type=int, default=400)
    ap.add_argument("--n-cg", type=int, default=15)
    args = ap.parse_args()

    A0 = np.load(os.path.join(VEC, "paper_A_va1.npy"))
    gt = np.load(os.path.join(VEC, "paper_gt.npy"))

    dA = build_dA(A0)
    u_true = (CSCALE_R * gt[:, :, 0].astype(float)).reshape(-1)
    P = make_problem(dA, dA @ u_true)            # δ=0 自洽

    lam = P["lam_ref"]
    rho = 0.1 * P["rho_ref"]                     # 定案规则
    print(f"[info] n = {P['ATA'].shape[0]}   ‖dA‖² = {P['s2']:.6e}")
    print(f"[info] λ = {lam:.6g}   ρ = {rho:.6g}   "
          f"th = λ/ρ = {lam / rho:.6g}")
    print(f"[info] 外层 {args.n_outer} × 内层 CG {args.n_cg}")

    u_ref, _ = admm_cg(P, lam, rho, n_outer=args.n_outer, n_cg=args.n_cg)

    def sm_err(u, ref):
        a = float(u @ ref) / float(u @ u)
        return float(np.linalg.norm(a * u - ref) / np.linalg.norm(ref))

    print(f"[info] 参考解 sm_err(对真值) = {sm_err(u_ref, u_true):.4e}")

    os.makedirs(args.out_dir, exist_ok=True)

    def w(name, arr):
        p = os.path.join(args.out_dir, name)
        np.ascontiguousarray(arr).tofile(p)
        print("  -> %-20s %-9s %s" % (name, str(arr.dtype), list(arr.shape)))

    tri = tri_pack(P["ATA"])
    w("x15g_ata_tri.bin", tri)
    w("x15g_aty.bin", np.ascontiguousarray(P["ATb"], dtype=np.float64))
    w("x15g_ref_u.bin", np.ascontiguousarray(u_ref, dtype=np.float64))
    w("x15g_ref_utrue.bin", np.ascontiguousarray(u_true, dtype=np.float64))

    n = P["ATA"].shape[0]
    lines = [
        "# X15g 向量参数清单（由 export_x15g_vectors.py 生成）",
        "# 问题：δ=0 自洽合成（数据由 paper A 自己生成）",
        "n_scene           = %d" % n,
        "n_rows            = 29",
        "n_cols            = 36",
        "tri_len           = %d" % (n * (n + 1) // 2),
        "norm_sq_dA        = %.17g" % P["s2"],
        "mean_diag_ATA     = %.17g" % P["rho_ref"],
        "lam               = %.17g" % lam,
        "rho               = %.17g" % rho,
        "n_outer           = %d" % args.n_outer,
        "n_cg              = %d" % args.n_cg,
        "u_true_inf        = %.17g" % float(np.max(np.abs(u_true))),
        "ref_u_inf         = %.17g" % float(np.max(np.abs(u_ref))),
        "",
        "# 偏移约定",
        "x15g_ata_tri.bin  : float64, (i,j) i<=j 的偏移 = i*n - i*(i-1)/2 + (j-i)",
        "                    即行优先上三角；i>j 时与 (j,i) 同格",
        "x15g_aty.bin      : float64, 索引 = i",
        "x15g_ref_u.bin    : float64, 索引 = i",
    ]
    with io.open(os.path.join(args.out_dir, "x15g_params.txt"), "w",
                 encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("  -> x15g_params.txt")
    print(f"[done] 输出目录 {args.out_dir}")


if __name__ == "__main__":
    main()
