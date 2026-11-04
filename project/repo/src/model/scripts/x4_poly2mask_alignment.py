# -*- coding: utf-8 -*-
"""X4 收尾 —— 遮挡栅格化与 MATLAB ``poly2mask`` 对齐的对照脚本。

两组证据
--------
1. **小算例**（``--small``）：8×8 网格上的矩形 / 三角形 / 整数边界矩形，逐像素列出
   「MATLAB `poly2mask` 语义」与「X4 之前的旧口径」的差异，用来把**约定**（评估网格 +
   边界包含规则）摊开看，而不是调参凑误差。
2. **大算例**（``--big``）：对 MATLAB 导出的 `simA`（15876×1044，float64）整矩阵比对，
   报相对 Frobenius 误差 vs X4 判据 1e-6。同时给出旧口径的残留误差作对照。

用法::

    python src/model/scripts/x4_poly2mask_alignment.py --small
    python src/model/scripts/x4_poly2mask_alignment.py --big \
        --data-root "<...>/Ordinary-Camera-master/Data" \
        --raw-dir   "<...>/Ordinary-Camera-master/ref_out"

``--big`` 在缺参照数据时打印 SKIP 并返回 0，不让 `make test` 变红。
本脚本**只读**，不往 sim/results 写任何东西（报告由人手写）。
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(_HERE, ".."))
if _MODEL_DIR not in sys.path:
    sys.path.insert(0, _MODEL_DIR)

from penumbrasight_model import forward as F                    # noqa: E402
from penumbrasight_model import forward_paper as FP             # noqa: E402
from penumbrasight_model import poly2mask as P2M                # noqa: E402

CRITERION = 1e-6


# ------------------------------------------------------------------ 工具


def _grid(mask: np.ndarray) -> list:
    return ["".join("1" if v else "." for v in row) for row in mask]


def _print_mask(p, title, mask):
    p("  %s（'1' = 像素中心在多边形内）" % title)
    p("       列 " + "".join("%2d" % c for c in range(1, mask.shape[1] + 1)))
    for r in range(mask.shape[0]):
        p("     行%2d %s" % (r + 1, " ".join("1" if v else "." for v in mask[r])))


def _print_cells(p, title, mask):
    cells = ["(%d,%d)" % (r + 1, c + 1)
             for r in range(mask.shape[0]) for c in range(mask.shape[1]) if mask[r, c]]
    p("  %s：%d 个 -> %s" % (title, len(cells), " ".join(cells) if cells else "（空）"))
    return cells


def legacy_sample_grid_mask(vertices, m, n, closed_rect=None):
    """X4 之前的旧口径：在**墙采样点网格**上判定。

    墙采样点射到像素坐标里是 ``s_j = (j-1)*n/(n-1)``（j = 1..n，FOV 下角 → 0、
    上角 → n）。矩形用旧代码原样的**闭区间** ``>= / <=``；一般多边形用 even-odd
    （把多边形按 ``f^{-1}(q) = q*(n-1)/n`` 变到整数网格上等价实现）。
    """
    if closed_rect is not None:
        x0, x1, y0, y1 = closed_rect
        s = (np.arange(1, n + 1) - 1.0) * n / (n - 1.0)
        in_x = (s >= x0) & (s <= x1)
        in_y = (s >= y0) & (s <= y1)
        return in_y[:, None] & in_x[None, :]
    v = np.asarray(vertices, dtype=np.float64) * (n - 1.0) / n
    return P2M.region_mask(v, m, n)


def matlab_mask(vertices, m, n):
    """MATLAB `poly2mask` 语义：整数像素中心 + even-odd（已由 simA 逐值验证）。"""
    return P2M.region_mask(vertices, m, n)


def small_cases(p):
    n = 8
    p("=" * 78)
    p("[1/3] 小算例：8×8 网格上的逐像素对照")
    p("=" * 78)
    p("像素坐标约定：x = 列、y = 行，像素中心在整数 1..8，多边形坐标定义在 [0, n] 上")
    p("（论文 simulate_occluder 正是这个口径：FOV 下角 -> 0、上角 -> n）。")
    p("墙采样点射到同一坐标里是 s_j = (j-1)*n/(n-1)，与整数网格最多差 1 个像素。")

    # ---------------- Case A：矩形（分数边界） ----------------
    p("")
    p("-" * 78)
    p("算例 A：矩形，x∈[2.3, 6.7], y∈[1.2, 5.1]（与像素中心/采样点都错开）")
    p("-" * 78)
    rect_v = np.array([[2.3, 1.2], [6.7, 1.2], [6.7, 5.1], [2.3, 5.1]])
    a_m = matlab_mask(rect_v, n, n)
    a_l = legacy_sample_grid_mask(None, n, n, closed_rect=(2.3, 6.7, 1.2, 5.1))
    _print_mask(p, "MATLAB poly2mask 口径（整数像素中心）", a_m)
    _print_mask(p, "旧口径（墙采样点网格 + 闭区间）", a_l)
    _print_mask(p, "差异 XOR", a_m != a_l)
    only_m = (a_m & ~a_l)
    only_l = (a_l & ~a_m)
    _print_cells(p, "只在 MATLAB 口径里被判为「被挡」", only_m)
    _print_cells(p, "只在旧口径里被判为「被挡」", only_l)
    p("  → 形状一模一样（同一个矩形），差异 %d 像素 = 新口径多判 %d + 旧口径多判 %d。"
      % (int((a_m != a_l).sum()), int(only_m.sum()), int(only_l.sum())))
    p("     机制：旧口径把**墙采样点**代进判据，而墙采样点射到这套像素坐标里是")
    p("     s_j = (j-1)*n/(n-1) < j（n=8 时 0, 1.143, ..., 8：两端对齐、中间偏小），")
    p("     要盖住同一块物理区域就得用**更大的索引** ⇒ 旧口径判出的索引区间两端各")
    p("     +0 或 +1（本例少了第 3 列与第 2 行）。这正是 4.57e-2 的来源。")

    # ---------------- Case B：三角形 ----------------
    p("")
    p("-" * 78)
    p("算例 B：三角形 (1.4,1.6) (7.6,2.9) (3.2,7.4)（非轴对齐，看顶点/斜边规则）")
    p("-" * 78)
    tri_v = np.array([[1.4, 1.6], [7.6, 2.9], [3.2, 7.4]])
    b_m = matlab_mask(tri_v, n, n)
    b_l = legacy_sample_grid_mask(tri_v, n, n)
    _print_mask(p, "MATLAB poly2mask 口径（整数像素中心）", b_m)
    _print_mask(p, "旧口径（墙采样点网格）", b_l)
    _print_mask(p, "差异 XOR", b_m != b_l)
    _print_cells(p, "只在 MATLAB 口径里被判为「被挡」", b_m & ~b_l)
    _print_cells(p, "只在旧口径里被判为「被挡」", b_l & ~b_m)
    p("  → 差异 %d 像素，全部落在斜边的那一像素带上；两口径面积 %d vs %d（差 %.1f%%）。"
      % (int((b_m != b_l).sum()), int(b_m.sum()), int(b_l.sum()),
         100.0 * (b_m.sum() - b_l.sum()) / max(b_m.sum(), 1)))

    # ---------------- 一般多边形：与独立实现（matplotlib Path）对数 ----------------
    p("")
    p("-" * 78)
    p("独立佐证：凸多边形 vs matplotlib.path.Path（另一套独立 point-in-polygon）")
    p("-" * 78)
    try:
        from matplotlib.path import Path
        from scipy.spatial import ConvexHull, QhullError
        rng = np.random.default_rng(20260926)
        nn = 16
        rr, cc = np.mgrid[1:nn + 1, 1:nn + 1]      # 行优先展平，与掩码的 C 序一致
        pts = np.column_stack([cc.ravel().astype(float),
                               rr.ravel().astype(float)])
        bad = 0
        tot = 0
        tried = 0
        for _ in range(200):
            k = int(rng.integers(3, 9))
            v = rng.uniform(0.5, nn + 0.5, size=(k, 2))
            try:
                hull = ConvexHull(v)          # 凸包 ⇒ 简单多边形，两套规则必一致
            except QhullError:
                continue
            v = v[hull.vertices]
            tried += 1
            m1 = matlab_mask(v, nn, nn).reshape(-1)
            # 注意：Path(v, closed=True) 会把最后一个顶点当成 CLOSEPOLY 丢掉，
            # 必须显式首尾闭合、不传 codes。
            m2 = Path(np.vstack([v, v[:1]])).contains_points(pts)
            bad += int((m1 != m2).sum())
            tot += m1.size
        p("  %d 个随机凸多边形 × 16×16 像素：与 Path.contains_points 不一致 %d / %d"
          % (tried, bad, tot))
        p("  → 本模块的扫描线（半开 y 区间 + 向左严格跨过计数）与独立实现逐像素一致；")
        p("     非轴对齐情形的实现正确性由此确认，轴对齐矩形再另由 simA 整矩阵钉死。")
    except Exception as exc:            # pragma: no cover
        p("  跳过（%s）" % exc)

    # ---------------- Case C：整数边界 ----------------
    p("")
    p("-" * 78)
    p("算例 C：矩形，x∈[2, 7], y∈[2, 6]（边界正好压在整数像素中心 → 平局）")
    p("-" * 78)
    c_v = np.array([[2.0, 2.0], [7.0, 2.0], [7.0, 6.0], [2.0, 6.0]])
    c_m = matlab_mask(c_v, n, n)
    c_l = legacy_sample_grid_mask(None, n, n, closed_rect=(2.0, 7.0, 2.0, 6.0))
    _print_mask(p, "MATLAB poly2mask 口径", c_m)
    _print_mask(p, "旧口径（闭区间，含边界）", c_l)
    p("  → 结论：本口径下 even-odd 的「向左射线严格跨过」使**左/上边界像素被排除、"
      "右/下边界像素被包含**（平局只在坐标恰好为整数时出现）。真实数据里遮挡物投影"
      "边界是连续实数，平局概率为零，所以本项只作口径说明，不影响大算例。")


# ------------------------------------------------------------------ 参照

def _load_raw(raw_dir, tag, name):
    with open(os.path.join(raw_dir, "raw_%s__%s.shape" % (tag, name))) as fh:
        shape = tuple(int(s) for s in fh.read().split())
    a = np.fromfile(os.path.join(raw_dir, "raw_%s__%s.f64" % (tag, name)), dtype="<f8")
    return a.reshape(shape, order="F")


def _one_column(setup, corner_polys, mc, mr, mode, snap=False, ndiscr=6):
    """按 build_paper_A 的算术复现**单列**，遮栅格化口径由 (mode, snap) 指定。

    与 ``FP.build_paper_A`` 的对应口径逐位一致（下面有断言自检）。
    """
    n = setup.wall.n
    wall_x = setup.wall.x_min + np.arange(n) * (setup.wall.x_max - setup.wall.x_min) / (n - 1)
    wall_z = setup.wall.z_min + np.arange(n) * (setup.wall.z_max - setup.wall.z_min) / (n - 1)
    yw2 = float(setup.wall.y ** 2)
    t = setup.occluder.y / setup.wall.y

    lx0 = float(setup.monitor.x_centers[mc])
    lz0 = float(setup.monitor.z_centers[mr])
    sx = lx0 - (np.arange(ndiscr) / ndiscr) * setup.monitor.block_w
    sz = lz0 - (np.arange(ndiscr) / ndiscr) * setup.monitor.block_h
    SX, SZ = np.meshgrid(sx, sz)
    lx_all, lz_all = SX.reshape(-1), SZ.reshape(-1)

    rects = []
    for c in (FP.default_polygons(setup) if corner_polys is None else corner_polys):
        a = np.asarray(c, dtype=np.float64)
        rects.append(FP._rect_from_corners(a) if a.shape == (4, 3)
                     else tuple(float(v) for v in a.ravel()[:4]))

    img = np.zeros((n, n), dtype=np.float64)
    for i in range(lx_all.size):
        lx, lz = float(lx_all[i]), float(lz_all[i])
        dx2 = (wall_x - lx) ** 2
        dz2 = (wall_z - lz) ** 2
        tmp = dz2[:, None] + yw2 + dx2[None, :]
        inten = yw2 / (tmp * tmp)
        if mode == "wall_samples":
            qx = (1.0 - t) * lx + t * wall_x
            qz = (1.0 - t) * lz + t * wall_z
            vis = np.ones((n, n), dtype=bool)
            for (x0, x1, z0, z1) in rects:
                vis &= ~(((qz >= z0) & (qz <= z1))[:, None] &
                         ((qx >= x0) & (qx <= x1))[None, :])
        else:
            vis = np.ones((n, n), dtype=bool)
            for corners in FP.corner_polygons(setup, corner_polys):
                pix = P2M.project_occluder_corners(
                    corners, [lx, 0.0, lz], wall_y=setup.wall.y,
                    fov_x0=setup.wall.x_min, fov_w=setup.wall.x_max - setup.wall.x_min,
                    fov_z0=setup.wall.z_min, fov_h=setup.wall.z_max - setup.wall.z_min,
                    n=n)
                vis &= ~P2M.poly2mask(pix[:, 0], pix[:, 1], n, n, snap=snap)
        img += inten * vis
    return np.flipud(img).reshape(-1, order="F")


def convention_probe(p, setup, polys2, ref_va0):
    """在**真实几何**上用一列把候选约定摊开：评估网格 / 边界规则 / 子像素吸附。"""
    nrow, ncol = setup.monitor.shape
    ref = ref_va0[:, 0]
    p("")
    p("=" * 78)
    p("[2/3] 约定判别：真实几何下的**单列**（simA_va0 第 0 列，mc=36, mr=1）")
    p("=" * 78)
    p("  候选约定只在两点上不同：(a) 判定的评估网格；(b) 顶点是否吸附到 5×5 子像素网格。")
    p("")
    p("  %-34s %14s %14s %10s" % ("候选约定", "max|dA|", "相对误差", "零元一致"))
    cases = [("① 旧口径：墙采样点 + 闭区间", "wall_samples", False),
             ("② 整数像素中心 + 不吸附", "poly2mask", False),
             ("③ 整数像素中心 + 吸附(round)", "poly2mask", "round"),
             ("④ 整数像素中心 + 吸附(ceil)", "poly2mask", "ceil"),
             ("⑤ 整数像素中心 + 吸附(floor)", "poly2mask", "floor")]
    for label, mode, snap in cases:
        c = _one_column(setup, polys2, ncol - 1, 0, mode, snap)
        d = c - ref
        same_zero = bool(np.array_equal(c == 0, ref == 0))
        p("  %-34s %14.4e %14.3e %10s"
          % (label, np.abs(d).max(), np.linalg.norm(d) / np.linalg.norm(ref),
             "是" if same_zero else "否"))
    p("")
    p("  → 只有「整数像素中心」这一族能把误差降到 1e-14（浮点量级）；")
    p("     吸附用 round 与不吸附等价、ceil/floor 立刻破功（1e-2 量级）。")

    # 自检：单列复现器必须与 build_paper_A 的对应口径逐位一致
    for mode in ("wall_samples", "poly2mask"):
        A, _ = FP.build_paper_A(setup, view_angle=False, mz_reversed=True,
                                polygons=polys2, occluder_mask=mode)
        c = _one_column(setup, polys2, ncol - 1, 0, mode, False)
        ok = bool(np.array_equal(c, A[:, 0]))
        p("  自检：单列复现器 == build_paper_A[:,0]（%s）：%s" % (mode, ok))
        if not ok:
            raise AssertionError("单列复现器与 build_paper_A 不一致（%s）" % mode)


def big_cases(p, data_root, raw_dir):
    p("")
    p("=" * 78)
    p("[3/3] 大算例：整矩阵比对 MATLAB simA（判据 相对 Frobenius 误差 ≤ 1e-6）")
    p("=" * 78)

    cfg = os.path.join(data_root, "TestPosChair", "chair_test.mat")
    needed = [os.path.join(raw_dir, "raw_chair_va1__simA.f64"),
              os.path.join(raw_dir, "raw_chair_va0__simA.f64"),
              os.path.join(raw_dir, "raw_chair_va1__occ_corner.f64"),
              cfg]
    missing = [x for x in needed if not os.path.exists(x)]
    if missing:
        p("  SKIP：缺参照数据：")
        for m in missing:
            p("    " + m)
        return 0

    setup = F.load_setup(cfg)
    occ_corner = _load_raw(raw_dir, "chair_va1", "occ_corner")
    polys2 = FP.default_polygons(setup, occ_corner)
    n = setup.wall.n

    p("  walln = %d   墙点 %d   场景块 %d" % (n, n * n, setup.monitor.n_pixels))
    p("  多边形 %d 个：" % len(polys2))
    for i, q in enumerate(polys2):
        p("    #%d  x[%.6f, %.6f]  z[%.6f, %.6f]" % (i + 1, q[0], q[1], q[2], q[3]))

    convention_probe(p, setup, polys2, _load_raw(raw_dir, "chair_va0", "simA"))

    rows = []
    for tag, va in (("chair_va0", False), ("chair_va1", True)):
        ref = _load_raw(raw_dir, tag, "simA")
        p("")
        p("  ---- %s（%s）  |simA|_F = %.9e  max|simA| = %.9e ----"
          % (tag, "含视角因子 18/1" if va else "viewAngleCorrection=0",
             np.linalg.norm(ref), np.abs(ref).max()))
        for label, kw in (("poly2mask（新，默认）", dict(occluder_mask="poly2mask")),
                          ("wall_samples（旧口径）", dict(occluder_mask="wall_samples"))):
            t0 = time.time()
            A, meta = FP.build_paper_A(setup, view_angle=va, mz_reversed=True,
                                       polygons=polys2, **kw)
            el = time.time() - t0
            err = float(np.linalg.norm(A - ref) / np.linalg.norm(ref))
            dmax = float(np.abs(A - ref).max())
            exact = int((A == ref).sum())
            nz_a, nz_r = (A != 0), (ref != 0)
            ca, cr = np.linalg.norm(A, axis=0), np.linalg.norm(ref, axis=0)
            ok = cr > 0
            colr = np.linalg.norm(A[:, ok] - ref[:, ok], axis=0) / cr[ok]
            rows.append((tag, label, err, dmax, el))
            p("    %-22s 相对误差 %.6e  max|dA| %.4e  耗时 %5.1fs  %s"
              % (label, err, dmax, el, "PASS" if err <= CRITERION else "FAIL vs 1e-6"))
            p("      逐元素完全相同 %d / %d（%.4f%%，其余为浮点求和次序差）"
              % (exact, A.size, 100.0 * exact / A.size))
            p("      非零元：新 %d / 参照 %d ；逐列相对误差 中位 %.3e 90分位 %.3e max %.3e"
              % (int(nz_a.sum()), int(nz_r.sum()),
                 float(np.median(colr)), float(np.percentile(colr, 90)), float(colr.max())))
            if label.startswith("poly2mask"):
                last = (A, ref)

    # 吸附变体：掩码级的差异统计（直接比掩码，不经 A 的浮点求和）
    p("")
    p("  ---- 旧口径残差的结构：是边界散点噪声，还是系统性偏移？----")
    # 两个口径下「矩形掩码」都等价于「x 索引区间 × z 索引区间」，
    # 于是可以只比区间端点，把差异性质定量说清。
    s_grid = (np.arange(1, n + 1) - 1.0) * n / (n - 1.0)


    def _idx_range(vals, lo, hi, closed):
        if closed:
            sel = (vals >= lo) & (vals <= hi)
        else:
            sel = (vals > lo) & (vals < hi)
        w = np.where(sel)[0]
        return (int(w.min()) + 1, int(w.max()) + 1) if w.size else None


    dlo = {}
    dhi = {}
    n_mask_pairs = 0
    max_dlo = max_dhi = 0
    for k in (0, 1, 2, 300, 700, 1043):
        mc = setup.monitor.shape[1] - 1 - (k // setup.monitor.shape[0])
        mr = k % setup.monitor.shape[0]
        lx0 = float(np.asarray(setup.monitor.x_centers)[mc])
        lz0 = float(np.asarray(setup.monitor.z_centers)[mr])
        for lx, lz in ((lx0, lz0),
                       (lx0 - setup.monitor.block_w / 6, lz0 - setup.monitor.block_h / 6),
                       (lx0 - setup.monitor.block_w / 2, lz0 - setup.monitor.block_h / 2)):
            for corners in FP.corner_polygons(setup, polys2):
                pix = P2M.project_occluder_corners(
                    corners, [lx, 0.0, lz], wall_y=setup.wall.y,
                    fov_x0=setup.wall.x_min, fov_w=setup.wall.x_max - setup.wall.x_min,
                    fov_z0=setup.wall.z_min, fov_h=setup.wall.z_max - setup.wall.z_min,
                    n=n)
                for axis in (0, 1):
                    a, b = float(pix[:, axis].min()), float(pix[:, axis].max())
                    r_new = _idx_range(np.arange(1, n + 1), a, b, False)
                    r_old = _idx_range(s_grid, a, b, True)
                    if r_new is None or r_old is None:
                        continue                      # 一方为空（区间落在 FOV 外）
                    n_mask_pairs += 1
                    e_lo = r_old[0] - r_new[0]
                    e_hi = r_old[1] - r_new[1]
                    dlo[e_lo] = dlo.get(e_lo, 0) + 1
                    dhi[e_hi] = dhi.get(e_hi, 0) + 1
                    max_dlo = max(max_dlo, abs(e_lo))
                    max_dhi = max(max_dhi, abs(e_hi))
    p("    抽样 %d 条边（列 × 子采样 × 多边形 × 轴），比较两种口径判出的**索引区间端点**："
      % n_mask_pairs)
    p("      下端点差分布：%s（max|Δ| = %d）"
      % (", ".join("%+d -> %d" % (k, v) for k, v in sorted(dlo.items())), max_dlo))
    p("      上端点差分布：%s（max|Δ| = %d）"
      % (", ".join("%+d -> %d" % (k, v) for k, v in sorted(dhi.items())), max_dhi))
    p("    → 端点差只有 0 和 +1、且方向一致 ⇒ 旧残差是**整条边（乃至整个阴影）被平移")
    p("      一个索引**的相干偏移（系统性），不是边界散点噪声：所以虽然只有 0.156% 的")
    p("      元素不同，Frobenius 相对误差仍达 4.6e-2（整元素 0 <-> 满值地翻）。")

    p("")
    p("  ---- 5×5 子像素吸附（老版 MATLAB 文档口径）的掩码级影响 ----")
    nMasks = 0
    d_round = d_ceil = d_floor = 0
    for k in (0, 1, 2, 300, 700, 1043):
        mc = setup.monitor.shape[1] - 1 - (k // setup.monitor.shape[0])
        mr = k % setup.monitor.shape[0]
        lx0 = float(np.asarray(setup.monitor.x_centers)[mc])
        lz0 = float(np.asarray(setup.monitor.z_centers)[mr])
        for lx, lz in ((lx0, lz0),
                       (lx0 - setup.monitor.block_w / 6, lz0 - setup.monitor.block_h / 6)):
            for corners in FP.corner_polygons(setup, polys2):
                pix = P2M.project_occluder_corners(
                    corners, [lx, 0.0, lz], wall_y=setup.wall.y,
                    fov_x0=setup.wall.x_min, fov_w=setup.wall.x_max - setup.wall.x_min,
                    fov_z0=setup.wall.z_min, fov_h=setup.wall.z_max - setup.wall.z_min,
                    n=n)
                base = P2M.poly2mask(pix[:, 0], pix[:, 1], n, n, snap=False)
                d_round += int((P2M.poly2mask(pix[:, 0], pix[:, 1], n, n,
                                              snap="round") != base).sum())
                d_ceil += int((P2M.poly2mask(pix[:, 0], pix[:, 1], n, n,
                                             snap="ceil") != base).sum())
                d_floor += int((P2M.poly2mask(pix[:, 0], pix[:, 1], n, n,
                                              snap="floor") != base).sum())
                nMasks += 1
    p("    抽样 %d 张掩码（每张 %d 像素）：round 差异 %d、ceil 差异 %d、floor 差异 %d"
      % (nMasks, n * n, d_round, d_ceil, d_floor))
    p("    → 0.2 步长吸附的位移 ≤0.1 像素，跨不过任何整数像素中心，故 round 型吸附")
    p("      在本数据上与不吸附**完全等价**（这解释了为什么只用 simA 无法区分两者）；")
    p("      默认 snap=False，并保留 ceil/floor 以标注该自由度。")

    p("")
    p("=" * 78)
    p("汇总（判据 1e-6）")
    p("=" * 78)
    p("  %-12s %-22s %14s %14s %8s" % ("场景", "口径", "相对误差", "max|dA|", "耗时s"))
    worst = 0.0
    for tag, label, err, dmax, el in rows:
        p("  %-12s %-22s %14.6e %14.4e %8.1f" % (tag, label, err, dmax, el))
        if label.startswith("poly2mask"):
            worst = max(worst, err)
    p("")
    p("  新口径最坏相对误差 = %.6e   →  %s（判据 %.0e）"
      % (worst, "PASS" if worst <= CRITERION else "FAIL", CRITERION))
    p("  旧口径相对误差量级 = 4~5e-2（与 X4 之前的 4.57e-2 一致），"
      "全部来自遮挡边界上 ~1 像素宽的翻转载带。")
    return 0 if worst <= CRITERION else 1


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--small", action="store_true", help="只跑 8x8 小算例")
    ap.add_argument("--big", action="store_true", help="只跑 simA 整矩阵比对")
    ap.add_argument("--data-root", default=None)
    ap.add_argument("--raw-dir", default=None)
    ap.add_argument("--report", default=None, help="另存一份文本（默认只打印）")
    args = ap.parse_args()

    out = open(args.report, "w", encoding="utf-8") if args.report else sys.stdout

    def p(*a):
        print(*a, file=out)

    do_small = args.small or not args.big
    do_big = args.big or not args.small

    rc = 0
    if do_small:
        small_cases(p)
    if do_big:
        data_root = args.data_root or os.environ.get("PS_DATA_ROOT")
        raw_dir = args.raw_dir or os.environ.get("PS_REF_DIR")
        if not data_root or not raw_dir:
            p("")
            p("  SKIP：未给 --data-root / --raw-dir（也没有 PS_DATA_ROOT / PS_REF_DIR），"
              "跳过 simA 整矩阵比对。")
            p("        参照量由 MATLAB 的 export_ref_for_python.m 生成到 ref_out/。")
        else:
            rc = big_cases(p, data_root, raw_dir)

    if out is not sys.stdout:
        out.close()
        print("已写出 %s" % args.report)
    return rc


if __name__ == "__main__":
    sys.exit(main())
