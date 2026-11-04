"""Wall difference operators and nonnegative TV solver; no targets or GT access."""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(key, '1')
import time
import numpy as np
from scipy.linalg import cho_factor, cho_solve
from reconstruction_cpu import gradient, adjoint


def difference_operator(values, wall_shape, mode):
    """Values are rows in MATLAB column-major wall order, trailing axes arbitrary."""
    h, w = wall_shape
    if values.shape[0] != h*w:
        raise ValueError('Wall shape and measurement rows disagree')
    if mode == 'author':
        result = values.copy()
        result[:-1] = values[:-1] - values[1:]
        return result
    if mode == 'no_anchor':
        return values[:-1] - values[1:]
    vertical = np.arange(h*w)[np.arange(h*w) % h != h-1]
    dv = values[vertical] - values[vertical+1]
    if mode == 'vertical':
        return dv
    if mode == 'spatial2d':
        dh = values[:-h] - values[h:]
        return np.concatenate((dv, dh), axis=0) / np.sqrt(2)
    raise ValueError(f'Unknown wall difference mode: {mode}')


class NonnegativeTV:
    def __init__(self, gram, channel_scale, scene_shape, rho=.01):
        self.gram = np.asarray(gram)
        self.cc = np.asarray(channel_scale)
        self.shape = tuple(scene_shape)
        self.rho = rho
        n = int(np.prod(self.shape))
        if self.gram.shape != (n, n) or np.any(self.cc <= 0):
            raise ValueError('Invalid Gram matrix or channel scales')
        dmat = gradient(np.eye(n).reshape(*self.shape, n)).reshape(2*n, n)
        regularizer = dmat.T @ dmat + np.eye(n)
        self.regularizer = regularizer
        self._factorize(rho)

    def _factorize(self,rho):
        self.rho=rho
        self.factors = [cho_factor(self.gram*c*c+rho*self.regularizer,
                                  check_finite=False) for c in self.cc]

    def objective(self, f, h, lam):
        flat = f.reshape(-1, len(self.cc))
        return float(.5*np.sum(flat*(self.gram@flat*self.cc**2))
                     - np.sum(h*flat)
                     + lam*np.sqrt(np.sum(gradient(f)**2, axis=0)).sum())

    def solve(self, h, lam=.001, initial=None, limit=6000, tol=1e-5,
              progress=None, adaptive_penalty=False):
        if limit < 1 or tol <= 0 or lam <= 0:
            raise ValueError('Invalid solver configuration')
        shape = self.shape + (len(self.cc),)
        a = np.zeros(shape) if initial is None else np.array(initial, copy=True)
        if a.shape != shape or not np.isfinite(a).all():
            raise ValueError('Invalid initial reconstruction')
        w = np.maximum(a, 0)
        z = gradient(a); u = np.zeros_like(z); v = np.zeros_like(a)
        trace = []; tic = time.perf_counter();initial_rho=self.rho;penalty_updates=[]
        for it in range(1, limit+1):
            rhs = h + self.rho*(adjoint(z-u)+w-v).reshape(h.shape)
            a = np.column_stack([cho_solve(factor, rhs[:,c], check_finite=False)
                                 for c, factor in enumerate(self.factors)]).reshape(shape)
            g = gradient(a); t = g+u
            magnitude = np.sqrt(np.sum(t*t, axis=0))
            oldz = z; oldw = w
            z = t*np.maximum(0, 1-lam/self.rho/np.maximum(magnitude, 1e-30))[None]
            w = np.maximum(0, a+v); u += g-z; v += a-w
            if it % 100 == 0 or it == limit:
                primal = float(np.sqrt(np.sum((g-z)**2)+np.sum((a-w)**2)))
                dual = float(self.rho*np.linalg.norm(adjoint(z-oldz)+w-oldw))
                primal_limit = tol*max(1, float(np.linalg.norm(w)))
                dual_limit = tol*max(1, float(np.linalg.norm(h)))
                passed = primal < primal_limit and dual < dual_limit
                trace.append(dict(iteration=it, primal=primal, dual=dual,
                                  rho=self.rho,
                                  primal_limit=primal_limit, dual_limit=dual_limit,
                                  objective_without_constant=self.objective(w,h,lam)))
                if progress is not None:
                    progress(it, passed)
                if passed:
                    break
                if adaptive_penalty and it<limit:
                    primal_ratio=primal/primal_limit;dual_ratio=dual/dual_limit
                    new_rho=self.rho
                    if primal_ratio>10*dual_ratio:
                        new_rho=min(.1,self.rho*2)
                    elif dual_ratio>10*primal_ratio:
                        new_rho=max(1e-5,self.rho/2)
                    if new_rho!=self.rho:
                        old_rho=self.rho
                        u*=old_rho/new_rho;v*=old_rho/new_rho
                        self._factorize(new_rho)
                        penalty_updates.append(dict(iteration=it,old=old_rho,new=new_rho))
        assert np.isfinite(w).all() and w.min() >= 0
        tv_dual = self.rho*u; cone_dual = self.rho*v
        stationarity = (self.gram@w.reshape(h.shape)*self.cc**2-h).reshape(shape) + adjoint(tv_dual) + cone_dual
        dual_norm = np.sqrt(np.sum(tv_dual*tv_dual,axis=0))
        fenchel_mismatch = lam*np.sqrt(np.sum(gradient(w)**2,axis=0)).sum()-np.sum(tv_dual*gradient(w))
        return w, dict(iterations=it, split_tolerance_passed=bool(passed),
                       tolerance=tol, rho=self.rho, initial_rho=initial_rho,
                       adaptive_penalty=adaptive_penalty,penalty_updates=penalty_updates,
                       seconds=time.perf_counter()-tic,
                       kkt_diagnostics=dict(relative_stationarity=float(np.linalg.norm(stationarity)/max(1,np.linalg.norm(h))),
                                            tv_dual_ball_excess=float(max(0,dual_norm.max()-lam)),
                                            cone_dual_positive_excess=float(max(0,cone_dual.max())),
                                            tv_fenchel_mismatch=float(fenchel_mismatch)),
                       convergence_note='Split residual diagnostic, not an optimality or image-confidence certificate',
                       trace=trace)
