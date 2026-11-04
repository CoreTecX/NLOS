# -*- coding: utf-8 -*-
"""X5 遗留 R4 / R6 判定：官方能量面的「跨通道一致性」与「平坦度」随阈值档的退化。

背景
----
`sim/results/x5_recon_verification.md` §7.3 把两条留成「待查」：

  R6  第 3 轮信道会翻转符号（红 −0.5771 / 蓝 +0.4811），与「官方 R/G/B 高互相关」
      的常识相悖，未查因。
  R4  「K≡1 档下 r 无效」需要一个前置判据（建议先报首奇异值方差占比 / K 分布）。

本脚本只读官方 `chair_test.mat`，把这两条一起判定：
  1) 逐轮、逐阈值档 t，算官方**自己**三个通道面 r/g/b 的两两 Pearson；
  2) 逐档算三个通道面的**相对离散度** cv = std / |mean|（面「平不平」的可观测代理）；
  3) 逐轮找出「跨通道 r 开始塌陷」的档，看它是否与「cv 掉到 1e-3 以下」同步。

结论见 `sim/results/x5_r4_r6_surface_degeneracy.md`，日志
`sim/results/x5_channel_degeneracy_run.log`。

用法（仓库根目录）：
  python src/model/scripts/x5_channel_degeneracy_probe.py --mat "<chair_test.mat>"
"""
import argparse

import numpy as np
import scipy.io as sio

_ROUNDS = ((1, ""), (2, "_2"), (3, "_3"), (4, "_4"))
_THRESH = {1: 1, 2: 4, 3: 6, 4: 1}     # thresh{k}（1 基）→ 工作档 t = thresh{k} - 1


def pear(a, b):
    x = np.asarray(a, dtype=np.float64).reshape(-1)
    y = np.asarray(b, dtype=np.float64).reshape(-1)
    x = x - x.mean()
    y = y - y.mean()
    den = np.sqrt(float(x @ x) * float(y @ y))
    return float(x @ y / den) if den > 0 else float("nan")


def cv(a):
    s = np.asarray(a, dtype=np.float64).reshape(-1)
    return float(s.std() / (abs(s.mean()) + 1e-30))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mat", required=True, help="chair_test.mat 的完整路径")
    args = ap.parse_args()

    d = sio.loadmat(args.mat)

    print("=" * 78)
    print("X5 R4/R6 判定：官方能量面的跨通道一致性与平坦度 vs 阈值档")
    print("=" * 78)

    th = np.asarray(d["threshold"]).ravel()
    print("官方 threshold 档列表（σ）:", np.array2string(th, precision=4))
    print("各轮 thresh_k（1 基）:", {k: int(np.asarray(d["thresh" + ("_" + str(k) if k > 1 else "")]).ravel()[0])
                                    for k, _ in _ROUNDS})

    tbl = {}
    for k, suf in _ROUNDS:
        ar = np.asarray(d["proj_yOnRangeA" + suf], dtype=np.float64)
        ag = np.asarray(d["proj_yOnRangeA_g" + suf], dtype=np.float64)
        ab = np.asarray(d["proj_yOnRangeA_b" + suf], dtype=np.float64)
        nt = ar.shape[3] if ar.ndim == 4 else 1
        t_work = _THRESH[k] - 1
        rows = []
        print(f"\n--- 轮 {k}：档数 = {nt}，工作档 t = {t_work}（σ = {th[t_work]:g}）---")
        print("   t    σ           r(r,g)    r(r,b)    r(g,b)    cv_r      cv_g      cv_b   ")
        for t in range(nt):
            a = ar[:, :, :, t] if ar.ndim == 4 else ar
            b = ag[:, :, :, t] if ag.ndim == 4 else ag
            c = ab[:, :, :, t] if ab.ndim == 4 else ab
            r_rg, r_rb, r_gb = pear(a, b), pear(a, c), pear(b, c)
            c_r, c_g, c_b = cv(a), cv(b), cv(c)
            mark = "  <== 工作档" if t == t_work else ""
            print(f"  {t:2d}  {th[t]:<9.4g} {r_rg:+.4f}   {r_rb:+.4f}   {r_gb:+.4f}   "
                  f"{c_r:.5f}   {c_g:.5f}   {c_b:.5f}{mark}")
            rows.append(dict(t=t, sigma=float(th[t]), r_rg=r_rg, r_rb=r_rb, r_gb=r_gb,
                             cv=[c_r, c_g, c_b]))
        tbl[k] = rows

        # 判定：跨通道 r 塌陷的档 vs cv 跌破 1e-3 的档
        collapse = [r["t"] for r in rows if min(r["r_rb"], r["r_gb"]) < 0.5]
        flat = [r["t"] for r in rows if max(r["cv"]) < 1e-3]
        print(f"  → 跨通道 r(r,b) 或 r(g,b) < 0.5 的档：{collapse or '无'}")
        print(f"  → 三通道 cv 全 < 1e-3（面接近常数）的档：{flat or '无'}")
        print(f"  → 工作档 t={t_work}：cv = "
              f"{[round(x, 5) for x in rows[t_work]['cv']]}，"
              f"min(r_rb, r_gb) = {min(rows[t_work]['r_rb'], rows[t_work]['r_gb']):+.4f}")

    # 交叉表：把两件事并排
    print("\n" + "=" * 78)
    print("交叉表：工作档的「面平坦度」与「跨通道一致性」")
    print("=" * 78)
    print("  轮  工作档t  σ        max cv      min r(跨通道)   判定")
    for k, _ in _ROUNDS:
        rows = tbl[k]
        t = _THRESH[k] - 1
        r = rows[t]
        mx = max(r["cv"])
        mn = min(r["r_rb"], r["r_gb"])
        verdict = "面有信息可用" if (mx > 1e-3 and mn > 0.5) else "面已退化，r 不可引"
        print(f"  {k}   {t:5d}   {r['sigma']:<8.4g} {mx:.5f}     {mn:+.4f}         {verdict}")


if __name__ == "__main__":
    main()
