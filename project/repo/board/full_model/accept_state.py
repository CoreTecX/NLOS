"""Offline independent FP64 raw-state audit. Requires NumPy/SciPy on audit host.

No candidate gradient, adjoint, residual, DCT or objective code is imported.
PFS is an opt-in diagnostic; it does not replace the ordinary output contract.
"""
from pathlib import Path
import argparse,hashlib,json,struct,zlib,os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ.setdefault(key,'1')
import numpy as np
from scipy.fft import dctn,idctn
H,W,C=29,36,3
N=H*W;S=N*C;M=126*126

def read_blob(path,kind,shape,count=None):
    raw=Path(path).read_bytes()
    if len(raw)<64 or raw[:8]!=b'PSFULL1\0' or struct.unpack_from('<IIII',raw,8)!=(1,kind,*shape):raise ValueError('Invalid blob header')
    if struct.unpack_from('<Q',raw,24)[0]!=len(raw)-64 or raw[36:40]!=bytes(4) or raw[56:64]!=bytes(8):raise ValueError('Invalid blob length/reserved')
    if zlib.crc32(raw[64:])!=struct.unpack_from('<I',raw,32)[0]:raise ValueError('CRC mismatch')
    if count is not None and len(raw)!=64+count*8:raise ValueError('Wrong payload length')
    return raw[40:56],raw[64:]

def model(path):
    identity,payload=read_blob(path,1,(H,W));offset=8+N*8+N*M*2
    expected=8+N*8+N*M*2+3*N*N*8+2*N*8
    if len(payload)!=expected:raise ValueError('Invalid model size')
    def take(n):
        nonlocal offset
        out=np.frombuffer(payload,dtype='<f8',count=n,offset=offset);offset+=8*n;return out
    k=take(N*N).reshape(N,N);kt=take(N*N).reshape(N,N);ev=take(N);spec=take(N).reshape(H,W);factor=take(N*N).reshape(N,N)
    if not all(np.isfinite(a).all() for a in (k,kt,ev,spec,factor)) or not np.array_equal(k.T,kt):raise ValueError('Invalid model arrays')
    return dict(identity=identity,k=k,kt=kt,ev=ev,spec=spec,factor=factor)

def gradient(a):
    result=np.zeros((2,H,W,C));result[0,:-1]=np.diff(a,axis=0);result[1,:,:-1]=np.diff(a,axis=1);return result

def adjoint(a):
    result=np.zeros((H,W,C));result[:-1]-=a[0,:-1];result[1:]+=a[0,:-1]
    result[:,:-1]-=a[1,:,:-1];result[:,1:]+=a[1,:,:-1];return result

def decode_state(path):
    identity,payload=read_blob(path,5,(H,W),7+15*S);raw=np.frombuffer(payload,dtype='<f8')
    if not np.isfinite(raw).all():raise ValueError('Nonfinite state')
    rho,it,pr,du,pl,dl,alpha=raw[:7];offset=7;arrays={}
    for name,sz in [('a',S),('w',S),('z',2*S),('u',2*S),('v',S),('old_a',S),('old_w',S),('old_z',2*S),('old_u',2*S),('old_v',S),('h',S)]:
        arrays[name]=raw[offset:offset+sz].reshape((2,H,W,C) if sz==2*S else (H,W,C));offset+=sz
    if not(1e-5<=rho<=.1 and it==int(it) and 0<it<=20000 and 1<=alpha<2):raise ValueError('Invalid state scalars')
    return dict(identity=identity,rho=rho,iteration=int(it),logged_primal=pr,logged_dual=du,logged_plim=pl,logged_dlim=dl,alpha=alpha,**arrays)

def evaluate(model_path,measurement_path,prefix,reference_prefix=None):
    m=model(model_path);identity,payload=read_blob(measurement_path,2,(126,126),3+3*M)
    meas=np.frombuffer(payload,dtype='<f8');sc=meas[:3]
    if not np.isfinite(meas).all() or np.any(sc<=0) or np.any(meas[3:]<0):raise ValueError('Invalid measurement')
    cc=sc/sc.max();s=decode_state(Path(prefix).with_suffix('.pfs'))
    outid,outpayload=read_blob(Path(prefix).with_suffix('.pfo'),4,(H,W),3*S);out=np.frombuffer(outpayload,dtype='<f8').reshape(3,H,W,C)
    if not(identity==m['identity']==s['identity']==outid):raise ValueError('State/model identity mismatch')
    if not np.array_equal(out[0],s['h']) or not np.array_equal(out[1],s['w']):raise ValueError('State/output mismatch')
    g=gradient(s['a']);rho=s['rho'];w=s['w'];h=s['h']
    primal=float(np.sqrt(np.sum((g-s['z'])**2)+np.sum((s['a']-w)**2)))
    dual=float(rho*np.linalg.norm(adjoint(s['z']-s['old_z'])+w-s['old_w']))
    plim=1e-7*max(1.,float(np.linalg.norm(w)));dlim=1e-7*max(1.,float(np.linalg.norm(h)))
    # Audit the final update too; corrupted dual states cannot hide behind logs.
    alpha=s['alpha'];relaxed_g=alpha*g+(1-alpha)*s['old_z'];relaxed_a=alpha*s['a']+(1-alpha)*s['old_w']
    t=relaxed_g+s['old_u'];magnitude=np.sqrt(np.sum(t*t,axis=0))
    expected_z=t*np.maximum(0,1-.001/rho/np.maximum(magnitude,1e-30))[None]
    expected_w=np.maximum(0,relaxed_a+s['old_v'])
    checks=[np.allclose(s['z'],expected_z,rtol=1e-12,atol=2e-13),np.allclose(w,expected_w,rtol=1e-12,atol=2e-13),
            np.allclose(s['u'],s['old_u']+relaxed_g-s['z'],rtol=1e-12,atol=2e-13),np.allclose(s['v'],s['old_v']+relaxed_a-w,rtol=1e-12,atol=2e-13)]
    rhs=h+rho*(adjoint(s['old_z']-s['old_u'])+s['old_w']-s['old_v'])
    ri=idctn(dctn(rhs,type=2,norm='ortho',axes=(0,1))/m['spec'][:,:,None],type=2,norm='ortho',axes=(0,1))
    coeff=cc**2/(rho+m['ev'][:,None]*cc**2)
    expected_a=(ri.reshape(N,C)-m['k']@(coeff*(m['kt']@rhs.reshape(N,C))))/rho
    linear_error=float(np.linalg.norm(expected_a-s['a'].reshape(N,C))/max(1.,np.linalg.norm(expected_a)))
    flat=w.reshape(N,C);fw=m['factor']@flat
    objective=float(.5*np.sum((fw*cc)**2)-np.sum(h*w)+.001*np.sqrt(np.sum(gradient(w)**2,axis=0)).sum())
    stationarity=(m['factor'].T@fw*cc**2-h.reshape(N,C)).reshape(H,W,C)+rho*(adjoint(s['u'])+s['v'])
    original_residual_pass=bool(primal<plim and dual<dlim)
    log_match=all(np.isclose(a,b,rtol=1e-8,atol=1e-13) for a,b in [(primal,s['logged_primal']),(dual,s['logged_dual']),(plim,s['logged_plim']),(dlim,s['logged_dlim'])])
    report=dict(passed=bool(original_residual_pass and log_match and all(checks) and linear_error<1e-9 and np.isfinite(objective) and np.min(w)>=0),
        original_residual_pass=original_residual_pass,primal=primal,dual=dual,primal_limit=plim,dual_limit=dlim,objective_without_constant=objective,
        update_checks=[bool(v) for v in checks],linear_subproblem_relative_error=linear_error,logged_residuals_match=bool(log_match),iterations=s['iteration'],
        relative_stationarity=float(np.linalg.norm(stationarity)/max(1.,np.linalg.norm(h))),state_sha256=hashlib.sha256(Path(prefix).with_suffix('.pfs').read_bytes()).hexdigest(),independent='NumPy/SciPy FP64; original state residuals, direct frozen-F objective; no candidate helper imports')
    if reference_prefix:
        ref=evaluate(model_path,measurement_path,reference_prefix)
        error=abs(objective-ref['objective_without_constant'])/max(1.,abs(ref['objective_without_constant']))
        report['objective_relative_difference']=error;report['passed']=bool(report['passed'] and ref['passed'] and error<1e-6)
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('model',type=Path);p.add_argument('measurement',type=Path);p.add_argument('prefix',type=Path);p.add_argument('--reference-prefix',type=Path);p.add_argument('--report',type=Path);a=p.parse_args()
    result=evaluate(a.model,a.measurement,a.prefix,a.reference_prefix);text=json.dumps(result,indent=2,allow_nan=False)
    if a.report:a.report.write_text(text+'\n','utf-8')
    print(text);return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
