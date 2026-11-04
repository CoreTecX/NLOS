# -*- coding: utf-8 -*-
"""X9 测试向量导出：给 M3 Gram 累加核的 C++ 主机测试用。

导出内容（追加到 x8 的向量旁边）
--------------------------------
    x9_yd.bin            float32[(n-1)*n]  差分测量（G 通道），供 A_d^T y
    x9_cand_ll.bin       float32[3*K]      候选位置（与 x8 同序：真值 / y 上界 / y 下界）
    x9_ref_aty.bin       float32[K*N]      参考 A_d^T y
    x9_ref_gram.bin      float32[K*T]      参考 A_d^T A_d 的**上三角**（行优先）
    x9_gram_meta.bin     int32[4]          N、上三角元素数 T、K、保留位

上三角的排布顺序与 C++ 累加器一致：np.triu_indices(N)，即
    (0,0),(0,1),...,(0,N-1),(1,1),(1,2),...
块内偏移 = Σ_{j=lo}^{i-1}(N-j) + (i'-i)（见 gram.hpp 的 block_base）。

用法
----
    python src/hardware/scripts/export_x9_vectors.py \
        --data-root "D:\\学习\\26秋嵌入式\\论文数据集\\Ordinary-Camera-master\\Data"
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_HW = os.path.dirname(_HERE)
_SRC = os.path.dirname(_HW)
_SRC_MODEL = os.path.join(_SRC, "model")
if _SRC_MODEL not in sys.path:
    sys.path.insert(0, _SRC_MODEL)

from penumbrasight_model import forward as F  # noqa: E402
from penumbrasight_model import io_mat  # noqa: E402
from penumbrasight_model import localization as L  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", required=True)
    ap.add_argument("--out-dir", default=os.path.join(_HW, "hwdata"))
    args = ap.parse_args()

    cfg = os.path.join(args.data_root, "TestPosChair", "chair_test.mat")
    setup = F.load_setup(cfg)
    n = setup.wall.n
    N = setup.monitor.n_pixels
    d = io_mat.load_mat(cfg)

    yd = L.apply_row_difference_img(np.asarray(d["meas_g"], dtype=np.float64))
    truth = np.asarray(setup.occluder.ll, dtype=np.float64)[:3]
    y_vals = np.asarray(io_mat.load_mat(cfg)["yhatvals"],
                        dtype=np.float64).reshape(-1)

    cands = []
    for y in (float(truth[1]), float(y_vals.max()), float(y_vals.min())):
        ll = truth.copy()
        ll[1] = y
        cands.append(ll)
    cands = np.asarray(cands, dtype=np.float64)
    K = cands.shape[0]

    iu = np.triu_indices(N)
    T = iu[0].size
    ref_aty = np.zeros((K, N), dtype=np.float32)
    ref_gram = np.zeros((K, T), dtype=np.float32)

    for k in range(K):
        s = setup.with_occluder_ll(cands[k])
        Ad = L.apply_row_difference_A(F.build_transport_matrix(s), n)
        ref_aty[k] = (Ad.T @ yd).astype(np.float32)
        G = Ad.T @ Ad
        ref_gram[k] = G[iu].astype(np.float32)
        print("[info] 候选 %d/%d  ll=%s  |A_d|_F=%.6e  max|G_tri|=%.6e"
              % (k + 1, K, np.round(cands[k], 5).tolist(),
                 float(np.linalg.norm(Ad)), float(np.abs(G[iu]).max())))
        del Ad, G
        sys.stdout.flush()

    os.makedirs(args.out_dir, exist_ok=True)

    def w(name, arr):
        arr.tofile(os.path.join(args.out_dir, name))
        print("  -> %-22s %-10s %s" % (name, str(arr.dtype), list(arr.shape)))

    w("x9_yd.bin", np.ascontiguousarray(yd, dtype=np.float32))
    w("x9_cand_ll.bin", np.ascontiguousarray(cands, dtype=np.float32))
    w("x9_ref_aty.bin", ref_aty.reshape(-1))
    w("x9_ref_gram.bin", ref_gram.reshape(-1))
    w("x9_gram_meta.bin", np.array([N, T, K, 0], dtype=np.int32))
    print("[done] 输出目录 %s" % args.out_dir)


if __name__ == "__main__":
    main()
