"""CPU quality reference with explicit geometry, scale and convergent split residuals."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
os.environ.setdefault('MKL_NUM_THREADS','1')
import numpy as np
from scipy.linalg import cho_factor, cho_solve, eigh

def gradient(a):
    g=np.zeros((2,)+a.shape)
    g[0,:-1]=np.diff(a,axis=0);g[1,:,:-1]=np.diff(a,axis=1)
    return g

def adjoint(g):
    a=np.zeros(g.shape[1:]);a[:-1]-=g[0,:-1];a[1:]+=g[0,:-1]
    a[:,:-1]-=g[1,:,:-1];a[:,1:]+=g[1,:,:-1]
    return a

class TVReference:
    def __init__(self,B,shape,rank=None,rho=.05):
        self.shape=shape;self.n=int(np.prod(shape));self.rho=rho
        gram=B.T@B
        ev,V=eigh(gram,check_finite=False);order=np.argsort(ev)[::-1]
        ev=np.maximum(ev[order],0);V=V[:,order]
        self.singular=np.sqrt(ev);self.norm=max(self.singular[0],1e-30)
        k=self.n if rank is None else min(rank,self.n)
        self.V=V[:,:k];self.B=B@self.V/self.norm
        self.G=self.V@np.diag(ev[:k]/self.norm**2)@self.V.T
        D=gradient(np.eye(self.n).reshape(*shape,self.n)).reshape(2*self.n,self.n)
        self.factor=cho_factor(self.G+rho*(D.T@D+np.eye(self.n)),check_finite=False)
    def solve(self,d,lam=.001,iterations=1500,tol=1e-5):
        h=self.V@(self.B.T@d)
        a=np.zeros(self.shape+(d.shape[1],));w=a.copy();z=gradient(a)
        u=np.zeros_like(z);v=np.zeros_like(a)
        for it in range(1,iterations+1):
            rhs=h+self.rho*(adjoint(z-u)+w-v).reshape(self.n,-1)
            a=cho_solve(self.factor,rhs,check_finite=False).reshape(a.shape)
            g=gradient(a);t=g+u;n=np.sqrt(np.sum(t*t,axis=0))
            oldz=z;oldw=w
            z=t*np.maximum(0,1-lam/self.rho/np.maximum(n,1e-30))[None]
            w=np.maximum(0,a+v);u+=g-z;v+=a-w
            if it%20==0 or it==iterations:
                pr=float(np.sqrt(np.sum((g-z)**2)+np.sum((a-w)**2)))
                du=float(self.rho*np.linalg.norm(adjoint(z-oldz)+w-oldw))
                if pr<tol*max(1,np.linalg.norm(w)) and du<tol*max(1,np.linalg.norm(h)):break
        r=self.B@(self.V.T@w.reshape(self.n,-1))-d
        tv=float(np.sqrt(np.sum(gradient(w)**2,axis=0)).sum())
        return w,dict(iterations=it,converged=bool(pr<tol*max(1,np.linalg.norm(w)) and du<tol*max(1,np.linalg.norm(h))),
                      split_primal=pr,split_dual=du,relative_residual=float(np.linalg.norm(r)/max(np.linalg.norm(d),1e-30)),
                      objective=float(.5*np.sum(r*r)+lam*tv),lambda_normalized=lam,
                      model_top_singular=self.norm,rank=self.V.shape[1])

def rgb_metrics(a,gt):
    gt=np.asarray(gt,dtype=float)
    gain=max(0.,float(np.sum(a*gt)/max(np.sum(a*a),1e-30)))
    e=a*gain-gt
    return dict(relative_l2=float(np.linalg.norm(e)/max(np.linalg.norm(gt),1e-30)),
                correlation=float(np.corrcoef(a.ravel(),gt.ravel())[0,1]) if np.std(a)>0 else 0.,
                psnr_db=float(10*np.log10(max(gt.max()**2,1e-30)/max(np.mean(e*e),1e-30))),shared_rgb_gain=gain)
