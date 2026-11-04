"""Compare finite NumPy arrays without using the candidate solver."""
import argparse,numpy as np,json,sys
p=argparse.ArgumentParser(description=__doc__);p.add_argument('reference');p.add_argument('candidate');p.add_argument('--rtol',type=float,default=1e-7);p.add_argument('--atol',type=float,default=1e-12);a=p.parse_args()
x=np.load(a.reference,allow_pickle=False);y=np.load(a.candidate,allow_pickle=False)
ok=x.shape==y.shape and np.isfinite(x).all() and np.isfinite(y).all()
if ok:
 d=np.linalg.norm((x-y).ravel());n=np.linalg.norm(x.ravel());ok=bool(np.allclose(x,y,rtol=a.rtol,atol=a.atol));print(json.dumps({'passed':ok,'relative_l2':float(d/max(n,1e-300)),'max_abs':float(np.max(np.abs(x-y)))}))
else:print('FAIL: shape or finite-value contract')
sys.exit(0 if ok else 1)
