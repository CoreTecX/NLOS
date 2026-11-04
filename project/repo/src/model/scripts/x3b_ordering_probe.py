"""X3b —— 排列约定探测：A 的行（墙点）与列（隐藏像素）的排列顺序约定

上一轮 X3 的结果是「几何全对，但 A 逐元素相关系数只有 0.10」。
这种形态的典型原因是**排列不同**而非数值不同 —— 即同一个物理量，
我们按 (z 行, x 列) 展平，MATLAB 可能按 (x, z)、或反序展平。
如果只是排列差，那么「A 对某一轴求和」得到的投影图应当是同一张图
（至多相差转置/翻转），这就把 15876×1044 的比对降成 126×126 和 29×36 的图比对。

做法：对两个轴各求投影图，然后穷举二面体群 8 种变换，看哪一种能把
MATLAB 的图对齐到我们的图，并报告相关系数。
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.abspath(os.path.join(HERE, ".."))
if _MODEL_DIR not in sys.path:
    sys.path.insert(0, _MODEL_DIR)
try:
    from penumbrasight_model import forward as F
except Exception:  # pragma: no cover
    import forward as F  # type: ignore

from x3_compare_reference import load_raw  # 复用读取函数


def dihedral(M: np.ndarray):
    """二面体群 8 个元素（含转置路径），返回 {名字: 变换后的矩阵}。"""
    out = {}
    for tname, T in (("", M), ("T", M.T)):
        out["原样" + tname] = T
        out["左右翻" + tname] = np.fliplr(T)
        out["上下翻" + tname] = np.flipud(T)
        out["左右+上下翻" + tname] = np.flipud(np.fliplr(T))
    return out


def best_align(img_a: np.ndarray, img_b: np.ndarray, label: str, p):
    """把 img_a 用 8 种变换对齐到 img_b，报告最好的一种。"""
    p("\n[%s]  形状 A=%s  B=%s" % (label, img_a.shape, img_b.shape))
    if img_a.shape != img_b.shape:
        p("  形状不同，无法直接对齐（转置后可能相同）")
    a = np.asarray(img_a, dtype=np.float64).reshape(-1)
    rows = []
    for name, T in dihedral(np.asarray(img_a, dtype=np.float64)).items():
        if T.shape != img_b.shape:
            continue
        b = np.asarray(img_b, dtype=np.float64).reshape(-1)
        ta = T.reshape(-1)
        if ta.std() == 0 or b.std() == 0:
            continue
        c = float(np.corrcoef(ta, b)[0, 1])
        # 归一化后的相对误差（各自先减均值除以标准差，去掉尺度差异）
        r = float(np.linalg.norm(ta - b) / np.linalg.norm(b)) if np.linalg.norm(b) else float("nan")
        rows.append((c, r, name, ta))
    rows.sort(key=lambda t: -t[0])
    for c, r, name, ta in rows:
        p("  %-14s 相关系数 %.9f   原尺度相对误差 %.6e" % (name, c, r))
    if rows:
        c, r, name, ta = rows[0]
        p("  → 最佳： %s   相关系数 %.9f" % (name, c))
        # 最优单一尺度下的相对误差（真正衡量「形状是否一致」）
        num = float(np.sum(ta * np.asarray(img_b, dtype=np.float64).reshape(-1)))
        den = float(np.sum(ta * ta))
        alpha = num / den if den else float("nan")
        bb = np.asarray(img_b, dtype=np.float64).reshape(-1)
        p("    最优尺度 alpha = %.9e ；除以该尺度后相对误差 = %.6e"
          % (alpha, float(np.linalg.norm(alpha * ta - bb) / np.linalg.norm(bb))))
    return rows[0][2] if rows else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--raw-dir", required=True)
    ap.add_argument("--tag", default="chair_va1")
    ap.add_argument("--report", default=None)
    args = ap.parse_args()

    out = open(args.report, "w", encoding="utf-8") if args.report else sys.stdout
    p = lambda *a: print(*a, file=out)

    A_ml = load_raw(args.raw_dir, args.tag, "simA")
    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup = F.load_setup(cfg)
    A_us = F.build_transport_matrix(setup)

    p("参照 A： %s" % (A_ml.shape,))
    p("我们 A： %s" % (A_us.shape,))

    n = setup.wall.n                      # 126
    nr, nc = setup.monitor.shape          # (29, 36)

    # --- 场景轴投影：对墙点求和 → (N_scene,)
    p("\n" + "=" * 78)
    p("场景轴（列）投影： sum over wall points")
    p("=" * 78)
    s_ml = A_ml.sum(axis=0)
    s_us = A_us.sum(axis=0)
    p("  取 (29,36) 时 MATLAB 需 order='C' 还是 'F'：")
    for order in ("C", "F"):
        img = s_ml.reshape(nr, nc, order=order)
        p("    order=%s 首行 %s" % (order, np.array2string(img[0, :4], precision=6)))
    p("  我们的（order='C'，行=z、列=x）首行 %s"
      % np.array2string(s_us.reshape(nr, nc, order="C")[0, :4], precision=6))
    for order in ("C", "F"):
        best_align(s_ml.reshape(nr, nc, order=order),
                   s_us.reshape(nr, nc, order="C"),
                   "场景轴投影 order=%s" % order, p)

    # --- 墙轴投影：对隐藏像素求和 → (126*126,)
    p("\n" + "=" * 78)
    p("墙轴（行）投影： sum over scene pixels")
    p("=" * 78)
    w_ml = A_ml.sum(axis=1)
    w_us = A_us.sum(axis=1)
    for order in ("C", "F"):
        best_align(w_ml.reshape(n, n, order=order),
                   w_us.reshape(n, n, order="C"),
                   "墙轴投影 order=%s" % order, p)

    # --- 逐块定位：取几个隐藏像素，看它对应的行投影，反推列序
    p("\n" + "=" * 78)
    p("列序定位：挑 MATLAB 第 0 列，把它对齐到我们每一列，看最像哪一列")
    p("=" * 78)
    col = A_ml[:, 0]
    cn = col / (np.linalg.norm(col) or 1.0)
    Un = A_us / np.maximum(np.linalg.norm(A_us, axis=0, keepdims=True), 1e-300)
    c = Un.T @ cn
    j = int(np.argmax(c))
    p("  MATLAB 第 0 列 最像我们的第 %d 列（相关系数 %.9f）" % (j, c[j]))
    p("  按「行优先 z×x」约定，第 0 列对应 (z 行 0, x 列 0)，序号应为 0")
    p("  按「列优先 x×z」约定，第 0 列对应序号 0，也对")
    p("  我们的第 %d 列按行优先=(行 %d, 列 %d)，按列优先=(行 %d, 列 %d)"
      % (j, j // nc, j % nc, j % nr, j // nr))
    top = np.argsort(-c)[:8]
    p("  前 8 个最像的列： %s" % np.array2string(top))
    p("  对应相关系数：    %s" % np.array2string(c[top], precision=6))

    if out is not sys.stdout:
        out.close()
        print("报告已写入 %s" % args.report)


if __name__ == "__main__":
    main()
