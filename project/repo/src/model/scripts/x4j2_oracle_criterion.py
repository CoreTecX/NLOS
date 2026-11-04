# -*- coding: utf-8 -*-
"""x4j2 —— oracle 判据：**图像里到底有没有定位信息？**

x4j 里 6 个判据全部失败（最好的是论文原判据 5.6 cm）。但那 6 个都要在场景未知的
条件下工作。这一步把场景真值（测量文件自带 ground_truth）**固定住**，只扫遮挡物位置：

    argmax_p  corr( A(p) @ vec(x_gt) , y )

- 若真值位置最优 → **信息在图像里**，失败只是估计器问题（可以用强场景先验的联合估计救）
- 若真值位置不最优 → **光凭这一张墙图在本文模型下无法定位**，选项 A 直接判死

这是能区分「信息不在」与「估计器不行」的最直接实验，而且很便宜。
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

REPO = os.path.abspath(os.path.join(_MODEL_DIR, os.pardir, os.pardir))
RESULTS = os.path.join(REPO, "sim", "results")
DEFAULT_MAT = (r"D:\学习\26秋嵌入式\论文数据集\Ordinary-Camera-master"
               r"\Data\TestPosD11\image_test_bur20.mat")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mat", default=DEFAULT_MAT)
    ap.add_argument("--nx", type=int, default=11)
    ap.add_argument("--ny", type=int, default=9)
    ap.add_argument("--nz", type=int, default=7)
    ap.add_argument("--xlo", type=float, default=0.34)
    ap.add_argument("--xhi", type=float, default=0.60)
    ap.add_argument("--ylo", type=float, default=0.42)
    ap.add_argument("--yhi", type=float, default=0.70)
    ap.add_argument("--zlo", type=float, default=0.12)
    ap.add_argument("--zhi", type=float, default=0.28)
    ap.add_argument("--downsample", type=int, default=4)
    ap.add_argument("--out", default=os.path.join(RESULTS, "x4j2_oracle.json"))
    args = ap.parse_args()

    st = FL.d11_loc_setup()
    truth = np.asarray(st.occ_ll_true, dtype=float)
    meas, gt = FL.load_measurement(args.mat, args.downsample)
    ds = args.downsample

    gx = np.linspace(args.xlo, args.xhi, args.nx)
    gy = np.linspace(args.ylo, args.yhi, args.ny)
    gz = np.linspace(args.zlo, args.zhi, args.nz)
    shp = (args.nx, args.ny, args.nz)

    print("=" * 74)
    print("x4j2 oracle 判据：固定场景真值，只扫遮挡物位置")
    print("=" * 74)
    print("真值 [%.4f %.4f %.4f]   网格 %d×%d×%d = %d"
          % (truth[0], truth[1], truth[2], args.nx, args.ny, args.nz, args.nx * args.ny * args.nz))
    print("网格步长 x %.4f  y %.4f  z %.4f"
          % (gx[1] - gx[0], gy[1] - gy[0], gz[1] - gz[0]))
    bp = np.array([gx[np.argmin(abs(gx - truth[0]))], gy[np.argmin(abs(gy - truth[1]))],
                   gz[np.argmin(abs(gz - truth[2]))]])
    print("网格上离真值最近的点 %.4f %.4f %.4f，距离 %.6f m"
          % (bp[0], bp[1], bp[2], np.linalg.norm(bp - truth)))

    # oracle 场景：优先用 ground_truth；g/b 通道为空则置零
    xgt = [np.asarray(gt[:, :, c], dtype=float).reshape(-1, order="F") for c in range(3)]
    y = [meas[:, :, c].reshape(-1, order="F") for c in range(3)]
    print("oracle 场景非零块数 r/g/b = %s（与 A 的 %d 列对应）"
          % ([int(np.count_nonzero(v)) for v in xgt], 29 * 36))

    plan = FL.make_plan(st, 2, downsamp_factor=ds)
    corr = np.zeros(shp)
    relres = np.zeros(shp)

    t0 = time.time()
    n = 0
    for ix in range(args.nx):
        for iy in range(args.ny):
            for iz in range(args.nz):
                A = FL.build_A_from_plan(plan, [gx[ix], gy[iy], gz[iz]])
                yp = A @ xgt[0]
                yn = y[0]
                if np.linalg.norm(yp) > 0:
                    corr[ix, iy, iz] = float(np.corrcoef(yp, yn)[0, 1])
                    al = float(yp @ yn / (yp @ yp))
                    relres[ix, iy, iz] = float(np.linalg.norm(yn - al * yp) / np.linalg.norm(yn))
                else:
                    corr[ix, iy, iz] = -1.0
                    relres[ix, iy, iz] = 1.0
                n += 1
        print("  x=%d/%d  %.1f s" % (ix + 1, args.nx, time.time() - t0))

    print("\n完成，%d 个候选，用时 %.1f s" % (n, time.time() - t0))

    def _best(surf, maximize, name):
        i, j, k = np.unravel_index(int(np.argmax(surf) if maximize else np.argmin(surf)),
                                   surf.shape)
        p = np.array([gx[i], gy[j], gz[k]])
        d = float(np.linalg.norm(p - truth))
        # 真值最近网格点的取值，以及全网格的分布
        ti = (int(np.argmin(abs(gx - truth[0]))), int(np.argmin(abs(gy - truth[1]))),
              int(np.argmin(abs(gz - truth[2]))))
        print("%-22s argmax=%.4f %.4f %.4f  距真值 %.6f m   (%s=%.6f)"
              % (name, p[0], p[1], p[2], d, "真值网格点", surf[ti]))
        print("    全网格 %s: min=%.6f max=%.6f mean=%.6f"
              % (name, surf.min(), surf.max(), surf.mean()))
        return p, d, float(surf[ti])

    print()
    print("=" * 74)
    p1, d1, tv1 = _best(corr, True, "corr(A@x_gt, y)")
    print()
    p2, d2, tv2 = _best(relres, False, "相对残差（越小越好）")
    print()
    print("=" * 74)
    print("判定")
    print("=" * 74)
    for nm, d, tv, best in (("相关系数", d1, tv1, corr.max()),
                            ("相对残差", d2, tv2, relres.min())):
        ok = d <= 0.03
        print("%s: 最优候选到真值 %.6f m；真值网格点取值 %.6f vs 全网格最优 %.6f"
              % (nm, d, tv, best))
        print("      -> 真值位置%s最优  =>  %s"
              % ("是" if (nm == "相关系数" and abs(tv - best) < 1e-9) or
                 (nm == "相对残差" and abs(tv - best) < 1e-9) else "不是",
                 "信息在图像里（问题只是估计器）" if ok else "光凭这张墙图定位不了（选项 A 判死）"))

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(dict(truth=truth.tolist(), grid=dict(gx=gx.tolist(), gy=gy.tolist(),
                                                       gz=gz.tolist()),
                       corr_best=p1.tolist(), corr_dist=d1,
                       relres_best=p2.tolist(), relres_dist=d2), f,
                  ensure_ascii=False, indent=2)
    np.savez_compressed(args.out.replace(".json", "_surf.npz"), corr=corr, relres=relres)
    print("\n结果: %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
