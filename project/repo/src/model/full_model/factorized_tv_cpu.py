"""Low-rank CPU nonnegative TV; float64 arithmetic, no ground-truth access."""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(key, '1')
import time
import numpy as np
from scipy.fft import dctn, idctn
from scipy.linalg import eigh
from reconstruction_cpu import gradient, adjoint


class FactorizedTV:
    """Solves the same ADMM subproblem as NonnegativeTV for Gram = F.T @ F."""
    def __init__(self, factor, channel_scale, scene_shape, rho=.01):
        self.F = np.asarray(factor, dtype=float)
        self.cc = np.asarray(channel_scale, dtype=float)
        self.shape = tuple(scene_shape)
        self.rho = rho
        n = int(np.prod(self.shape))
        if self.F.ndim != 2 or self.F.shape[1] != n or self.cc.shape != (3,):
            raise ValueError('Invalid factor or channel dimensions')
        if not np.isfinite(self.F).all() or not np.isfinite(self.cc).all() or np.any(self.cc <= 0):
            raise ValueError('Factors must be finite and channel scales positive')
        hh, ww = self.shape
        self.rspec = (1 + 2-2*np.cos(np.pi*np.arange(hh)/hh)[:,None]
                      + 2-2*np.cos(np.pi*np.arange(ww)/ww)[None,:])
        rinv_ft = self.regularizer_inverse(self.F.T)
        ev, q = eigh(self.F @ rinv_ft, check_finite=False)
        if ev.min() < -1e-10*max(1, ev.max()):
            raise ValueError('Factor Gram spectrum is not nonnegative')
        self.ev = np.maximum(ev, 0)
        self.k = rinv_ft @ q

    def regularizer_inverse(self, rhs):
        a = rhs.reshape(*self.shape, -1)
        return idctn(dctn(a, type=2, norm='ortho', axes=(0,1)) /
                     self.rspec[:,:,None], type=2, norm='ortho', axes=(0,1)).reshape(rhs.shape)

    def linear_solve(self, rhs):
        coeff = self.cc[None,:]**2 / (self.rho+self.ev[:,None]*self.cc[None,:]**2)
        return (self.regularizer_inverse(rhs)-self.k@(coeff*(self.k.T@rhs)))/self.rho

    def gram_product(self, flat):
        return self.F.T @ (self.F @ flat)

    def objective(self, f, h, lam):
        flat = f.reshape(-1,3)
        return float(.5*np.sum((self.F@flat*self.cc)**2)-np.sum(h*flat)
                     +lam*np.sqrt(np.sum(gradient(f)**2,axis=0)).sum())

    def solve(self, h, lam=.001, initial=None, limit=20000, tol=1e-7,
              progress=None, adaptive_penalty=True):
        h = np.asarray(h, dtype=float)
        shape = self.shape+(3,)
        if h.shape != (int(np.prod(self.shape)),3) or not np.isfinite(h).all():
            raise ValueError('Invalid measurement right-hand side')
        if limit < 1 or tol <= 0 or lam <= 0:
            raise ValueError('Invalid solver configuration')
        a = np.zeros(shape) if initial is None else np.array(initial, dtype=float, copy=True)
        if a.shape != shape or not np.isfinite(a).all():
            raise ValueError('Invalid initial reconstruction')
        w = np.maximum(a,0); z=gradient(a); u=np.zeros_like(z); v=np.zeros_like(a)
        trace=[]; updates=[]; tic=time.perf_counter(); initial_rho=self.rho
        for it in range(1,limit+1):
            rhs=h+self.rho*(adjoint(z-u)+w-v).reshape(h.shape)
            a=self.linear_solve(rhs).reshape(shape)
            g=gradient(a); t=g+u; magnitude=np.sqrt(np.sum(t*t,axis=0))
            oldz=z; oldw=w
            z=t*np.maximum(0,1-lam/self.rho/np.maximum(magnitude,1e-30))[None]
            w=np.maximum(0,a+v); u+=g-z; v+=a-w
            if it%100==0 or it==limit:
                primal=float(np.sqrt(np.sum((g-z)**2)+np.sum((a-w)**2)))
                dual=float(self.rho*np.linalg.norm(adjoint(z-oldz)+w-oldw))
                plim=tol*max(1,float(np.linalg.norm(w))); dlim=tol*max(1,float(np.linalg.norm(h)))
                passed=primal<plim and dual<dlim
                trace.append(dict(iteration=it,primal=primal,dual=dual,rho=self.rho,
                                  primal_limit=plim,dual_limit=dlim,
                                  objective_without_constant=self.objective(w,h,lam)))
                if progress is not None: progress(it,passed)
                if passed: break
                if adaptive_penalty and it<limit:
                    new_rho=self.rho
                    if primal/plim>10*dual/dlim:new_rho=min(.1,self.rho*2)
                    elif dual/dlim>10*primal/plim:new_rho=max(1e-5,self.rho/2)
                    if new_rho!=self.rho:
                        old_rho=self.rho;u*=old_rho/new_rho;v*=old_rho/new_rho;self.rho=new_rho
                        updates.append(dict(iteration=it,old=old_rho,new=new_rho))
        if not np.isfinite(w).all(): raise ArithmeticError('Non-finite reconstruction')
        td=self.rho*u; cd=self.rho*v
        stationarity=(self.gram_product(w.reshape(h.shape))*self.cc**2-h).reshape(shape)+adjoint(td)+cd
        return w,dict(iterations=it,split_tolerance_passed=bool(passed),tolerance=tol,
                      rho=self.rho,initial_rho=initial_rho,adaptive_penalty=adaptive_penalty,
                      penalty_updates=updates,seconds=time.perf_counter()-tic,
                      kkt_diagnostics=dict(relative_stationarity=float(np.linalg.norm(stationarity)/max(1,np.linalg.norm(h))),
                        tv_dual_ball_excess=float(max(0,np.sqrt(np.sum(td*td,axis=0)).max()-lam)),
                        cone_dual_positive_excess=float(max(0,cd.max())),
                        tv_fenchel_mismatch=float(lam*np.sqrt(np.sum(gradient(w)**2,axis=0)).sum()-np.sum(td*gradient(w)))),
                      convergence_note='Split residuals are not calibrated image confidence',trace=trace)
