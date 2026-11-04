# -*- coding: utf-8 -*-
"""论文差分重建求解器的 Python 逐行复现（fig4_column_c 口径）。

对应 MATLAB 源文件
------------------
    Functions/reconstruct_tv_it_cbg.m   （主流程 + co_prox_tv + fista_backtrack）

关键点
------
1. 差分矩阵 Diff 作用于 A 的行（15876 个墙点）。MATLAB 的 A(:) 是列优先展平，
   因此 y[i] - y[i+1] 是「同一列的相邻行」——即垂直差分。
2. 走正规方程路径：先算 dA = Diff*A、ATA = dA'*dA（1044x1044），
   迭代中只需 ATA*x，单次仅 1044^2 次乘加。
3. 初始估计为最小二乘解（tik_reg = [0,0,0] 时退化为 (cscale^2*ATA)\(cscale*AT*m)）。
4. TV 邻近算子用 Beck & Teboulle (2009) 的算法，内部自带迭代。
5. FISTA 带回溯线搜索，步长初始 1e-5、每轮 ×5、回溯时 ×0.3，最多 1100 轮。
"""
import os
import time

import numpy as np

# ---------------------------------------------------------------- 参数
NUMBLOCKS = (29, 36)
N_DISCR = 6
SUB_BLOCKS = np.array([1, 1])

# fig4_column_c.m 的 bu 分支
TV_REG_PARAM = 1e07 * np.array([5.0, 25.0, 25.0])
TIK_REG = np.array([0.0, 0.0, 0.0])          # 调用时传的是 [0,0,0]


def cscale_bu(n_discr=N_DISCR):
    """fig4_column_c.m 的 bu 分支里 sr/sg/sb 的算法。"""
    p = float(np.prod(SUB_BLOCKS))
    sr = 12500 / (n_discr ** 2) * p * 0.9
    sg = 15625 / (n_discr ** 2) * p * 0.98
    sb = 1.7188e+04 / (n_discr ** 2) * p * 1.04
    return np.array([sr, sg, sb])


# ---------------------------------------------------------------- 差分算子
def diff_matrix_apply(v, n):
    """等价于 Diff @ v，其中 Diff = diag(ones(n,1),0) - diag(ones(n-1,1),1)。"""
    out = np.empty_like(v)
    out[:-1] = v[:-1] - v[1:]
    out[-1] = v[-1]
    return out


def rs(x, shape=NUMBLOCKS):
    """复现 MATLAB 的 rs()：reshape 为 (NumBlocks(1), NumBlocks(2))，列优先。"""
    return np.asarray(x).reshape(shape, order="F")


# ---------------------------------------------------------------- TV 邻近算子
def co_prox_tv(b, lam, tol=1e-6, max_iter=20000, verbose=False):
    """复现 co_prox_tv.m：各向同性 TV 的邻近算子（Beck & Teboulle 2009）。

    求解 argmin_X 0.5||X - b||^2 + lam*TV(X)，约束为 none。
    """
    def Lt(x):
        p = np.zeros_like(x)
        q = np.zeros_like(x)
        p[:-1, :] = -(x[1:, :] - x[:-1, :])
        q[:, :-1] = -(x[:, 1:] - x[:, :-1])
        return p, q

    def Lop(dx, dy):
        ix = np.zeros_like(dx)
        iy = np.zeros_like(dy)
        ix[1:, :] = dx[1:, :] - dx[:-1, :]
        iy[:, 1:] = dy[:, 1:] - dy[:, :-1]
        ix[0, :] = dx[0, :]
        iy[:, 0] = dy[:, 0]
        return ix + iy

    b = np.asarray(b, dtype=np.float64)
    r = np.zeros_like(b)
    s = np.zeros_like(b)
    p_old = r.copy()
    q_old = s.copy()
    t_old = 1.0
    old_val = 0.0

    for it in range(max_iter):
        sol = b - lam * Lop(r, s)
        tmpr, tmps = Lt(sol)

        r = r + (1.0 / (8.0 * lam)) * tmpr
        s = s + (1.0 / (8.0 * lam)) * tmps

        weights = np.maximum(1.0, np.sqrt(r ** 2 + s ** 2))
        p = r / weights
        q = s / weights

        t = (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2)) / 2.0
        t_old = t

        r = p + ((t_old - 1.0) / t) * (p - p_old)
        s = q + ((t_old - 1.0) / t) * (q - q_old)
        p_old = p
        q_old = q

        cur_val = 0.5 * np.sum((b - sol) ** 2) + lam * np.sum(np.sqrt(tmpr ** 2 + tmps ** 2))
        if cur_val <= 0:
            break
        rel_obj = abs(cur_val - old_val) / cur_val
        if rel_obj < tol:
            break
        old_val = cur_val

    if verbose:
        print(f"      co_prox_tv 内层迭代 {it + 1} 次")
    return sol


# ---------------------------------------------------------------- FISTA
def fista_backtrack(ata, at_m, meas, xinit, lam, max_iter=1100, tol=2.4e-6,
                    step0=1e-5, alpha=1.0, verbose=True):
    """复现 fista_backtrack.m。

    这里把 grad 内联为 -ATm + ATA*x，避免反复 reshape。
    """
    x_old = xinit.copy()
    y_old = xinit.copy()
    t_old = 1.0
    step = step0
    n_el = x_old.size

    for it in range(max_iter):
        yoldgrad = -at_m + ata @ y_old.ravel(order="F")
        yoldgrad = yoldgrad.reshape(NUMBLOCKS, order="F")

        x_new = co_prox_tv(y_old - step * yoldgrad, lam * step)
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2))
        y_new = x_new + alpha * (t_old - 1.0) / t_new * (x_new - x_old)

        c = 0.5 * np.linalg.norm(meas - (ata @ x_new.ravel(order="F"))) ** 2 if False else \
            0.5 * np.sum((meas - reshape_to_full(ata, x_new)) ** 2)
        oldc = 0.5 * np.sum((meas - reshape_to_full(ata, y_old)) ** 2)

        n_line = 0
        while c > oldc + np.sum(yoldgrad * (x_new - y_old)) + \
                (1.0 / (2.0 * step)) * np.linalg.norm(x_new - y_old) ** 2:
            step = step * 0.3
            x_new = co_prox_tv(y_old - step * yoldgrad, lam * step)
            y_new = x_new + alpha * (t_old - 1.0) / t_new * (x_new - x_old)
            c = 0.5 * np.sum((meas - reshape_to_full(ata, x_new)) ** 2)
            n_line += 1
            if n_line > 60:
                break

        e = np.linalg.norm(x_new - x_old) / n_el
        if verbose and (it % 50 == 0 or it < 5):
            print(f"    iter {it + 1:4d}  step {step:.3e}  数据项 {c:.6e}  e {e:.3e}",
                  flush=True)
        if it > 10 and e < tol:
            if verbose:
                print(f"    在第 {it + 1} 轮收敛（e={e:.3e}）")
            break

        x_old = x_new
        t_old = t_new
        y_old = y_new
        step = step * 5.0

    return x_new


def reshape_to_full(ata, x):
    """占位函数：FISTA 的 calc_G 里 constants.A 是 cscale*dA（15876x1044），
    这里无法直接由 ata 得到，故在调用处用闭包替换。见 solve_channel。"""
    raise RuntimeError("应由闭包替换")


# ---------------------------------------------------------------- 单通道求解
def solve_channel(dA, mr, cs, reg, tik_reg, max_iter=1100, tol=2.4e-6, verbose=True):
    """复现 reconstruct_tv_it_cbg.m 里单通道的处理。"""
    AT = dA.T                                        # 1044 x 15876
    ATA = AT @ dA                                    # 1044 x 1044
    ATm = cs * (AT @ mr)                             # 1044

    # 初始估计：Tikhonov（tik_reg=0 时退化为最小二乘）
    if tik_reg > 0:
        raise NotImplementedError("tik_reg>0 的 DCT 先验未实现（本次试验用 [0,0,0]）")
    test = np.linalg.solve(cs ** 2 * ATA, ATm)
    test = rs(test)

    # FISTA：grad 用 ATA 表示，calc_G 需要完整的 A*x，这里用 dA 直接算
    A_full = cs * dA

    def calc_G(x):
        return 0.5 * np.sum((mr - A_full @ x.ravel(order="F")) ** 2)

    x_old = test.copy()
    y_old = test.copy()
    t_old = 1.0
    step = 1e-5
    alpha = 1.0
    n_el = x_old.size

    for it in range(max_iter):
        yoldgrad = (-ATm + (cs ** 2) * ATA @ y_old.ravel(order="F")).reshape(NUMBLOCKS, order="F")
        x_new = co_prox_tv(y_old - step * yoldgrad, reg * step)
        t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t_old ** 2))
        y_new = x_new + alpha * (t_old - 1.0) / t_new * (x_new - x_old)

        c = calc_G(x_new)
        oldc = calc_G(y_old)
        n_line = 0
        while c > oldc + np.sum(yoldgrad * (x_new - y_old)) + \
                (1.0 / (2.0 * step)) * np.linalg.norm(x_new - y_old) ** 2:
            step = step * 0.3
            x_new = co_prox_tv(y_old - step * yoldgrad, reg * step)
            y_new = x_new + alpha * (t_old - 1.0) / t_new * (x_new - x_old)
            c = calc_G(x_new)
            n_line += 1
            if n_line > 60:
                break

        e = np.linalg.norm(x_new - x_old) / n_el
        if verbose and (it % 25 == 0 or it < 3):
            print(f"    iter {it + 1:4d}  step {step:.2e}  数据项 {c:.4e}  e {e:.2e}"
                  f"  (回溯 {n_line})", flush=True)
        if it > 10 and e < tol:
            if verbose:
                print(f"    收敛于第 {it + 1} 轮（e={e:.3e}）")
            break

        x_old = x_new
        t_old = t_new
        y_old = y_new
        step = step * 5.0

    return x_new


# ---------------------------------------------------------------- 主流程
def reconstruct(A, measurement, cscale=None, reg=None, max_iter=1100,
                save_prefix=None, verbose=True):
    """复现 reconstruct_tv_it_cbg.m（tik_reg = [0,0,0]）。"""
    n = A.shape[0]
    cscale = cscale_bu() if cscale is None else np.asarray(cscale, float)
    reg = TV_REG_PARAM if reg is None else np.asarray(reg, float)

    print(f"  cscale = {cscale}")
    print(f"  reg    = {reg}")

    # 差分
    dA = np.empty_like(A)
    dA[:-1, :] = A[:-1, :] - A[1:, :]
    dA[-1, :] = A[-1, :]

    out = np.zeros(NUMBLOCKS + (3,))
    for ci, name in enumerate(("红", "绿", "蓝")):
        m = measurement[:, :, ci]
        mr = np.empty(n)
        mr[:-1] = m.ravel(order="F")[:-1] - m.ravel(order="F")[1:]
        mr[-1] = m.ravel(order="F")[-1]

        print(f"\n  通道 {name}：开始重建")
        t0 = time.time()
        x = solve_channel(dA, mr, cscale[ci], reg[ci], TIK_REG[ci],
                          max_iter=max_iter, verbose=verbose)
        print(f"  通道 {name} 完成，{time.time() - t0:.1f}s  "
              f"范围 [{x.min():.3e}, {x.max():.3e}]")
        out[:, :, ci] = x

    if save_prefix:
        np.save(save_prefix + "_recon.npy", out)
        print(f"\n已保存 {save_prefix}_recon.npy")
    return out


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--iter", type=int, default=1100)
    ap.add_argument("--quick", action="store_true", help="只跑 40 轮，用于验证管线")
    args = ap.parse_args()
    if args.quick:
        args.iter = 40

    base = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        "..", "..", "..", "sim", "vectors"))
    A = np.load(os.path.join(base, "paper_A_va1.npy"))
    meas = np.load(os.path.join(base, "paper_meas.npy"))
    gt = np.load(os.path.join(base, "paper_gt.npy"))

    print("=" * 70)
    print(f"论文差分重建复现 · A{A.shape}  测量{meas.shape}  真值{gt.shape}")
    print(f"最大迭代 {args.iter}")
    print("=" * 70)

    t0 = time.time()
    recon = reconstruct(A, meas, max_iter=args.iter,
                        save_prefix=os.path.join(base, "paper_recon"))
    print(f"\n总用时 {time.time() - t0:.1f}s")

    # 与真值做目视对照用的归一化
    for ci, name in enumerate(("R", "G", "B")):
        x = recon[:, :, ci]
        g = gt[:, :, ci].astype(float)
        xs = x - x.mean()
        gs = g - g.mean()
        r = float(np.sum(xs * gs) / (np.linalg.norm(xs) * np.linalg.norm(gs) + 1e-30))
        print(f"  通道 {name}: 相关系数 {r:+.4f}   重建范围 [{x.min():.2e}, {x.max():.2e}]")
