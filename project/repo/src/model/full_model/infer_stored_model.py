"""Full-rank spatial2d inference with reduced model storage and float64 arithmetic."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
from pathlib import Path
import argparse,json
import numpy as np
from scipy.linalg import eigh
from scipy.io import loadmat
from factorized_tv_cpu import FactorizedTV
from spatial_difference_cpu import difference_operator


def encode_model(model,kind):
    a=np.asarray(model,dtype=float)
    if not np.isfinite(a).all():raise ValueError('Nonfinite model')
    if kind=='float32':return dict(model=a.astype(np.float32))
    if kind!='int16':raise ValueError('Unsupported model storage')
    scales=np.maximum(np.max(np.abs(a),axis=0),1e-30)/32767
    return dict(model=np.clip(np.rint(a/scales),-32767,32767).astype(np.int16),column_scales=scales)


def decode_model(package):
    model=package['model'].astype(float)
    if 'column_scales' in package:model*=package['column_scales']
    return model


def prepare_operator(package):
    a=decode_model(package)
    wall=tuple(int(v) for v in package['wall_shape']);scene=tuple(int(v) for v in package['scene_shape'])
    n=int(np.prod(scene));norm=float(package['model_norm'])
    if a.shape!=(int(np.prod(wall)),n) or norm<=0 or not np.isfinite(norm) or not np.isfinite(a).all():
        raise ValueError('Invalid model dimensions or normalization')
    canonical=np.arange(n).reshape(*scene,order='F').ravel()
    b=difference_operator(a[:,canonical],wall,'spatial2d')/norm
    gram=b.T@b;ev,v=eigh(gram,check_finite=False)
    if ev.min() < -1e-10*max(1,ev.max()):raise ArithmeticError('Invalid Gram spectrum')
    factor=np.sqrt(np.maximum(ev,0))[:,None]*v.T
    return b,factor


def infer(package,measurement,scales,tol=1e-7,limit=20000):
    wall=tuple(int(v) for v in package['wall_shape']);scene=tuple(int(v) for v in package['scene_shape'])
    norm=float(package['model_norm']);sc=np.asarray(scales,dtype=float);y=np.asarray(measurement,dtype=float)
    if y.shape!=wall+(3,) or sc.shape!=(3,) or np.any(sc<=0) or not np.isfinite(y).all() or not np.isfinite(sc).all():
        raise ValueError('Invalid wall measurement or RGB scales')
    b,factor=prepare_operator(package)
    raw=y.reshape(-1,3,order='F');ds=float(np.max(np.abs(difference_operator(raw,wall,'author'))))
    if ds<=0:raise ValueError('Cannot normalize zero measurement')
    d=difference_operator(raw,wall,'spatial2d')/ds;cc=sc/sc.max()
    h=b.T@d*cc;solver=FactorizedTV(factor,cc,scene);f,diag=solver.solve(h,tol=tol,limit=limit)
    physical=f*ds/(norm*sc.max())
    return physical,f,dict(diagnostics=diag,storage=str(package['storage']),rank=int(np.prod(scene)),
        arithmetic='float64; stored model dequantized before differences and all operations',high_confidence=False,
        storage_is_not_peak_runtime_memory=True,
        normalization=dict(model=norm,measurement=ds,channel=cc.tolist(),physical_multiplier=ds/(norm*sc.max())))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model-package',type=Path,required=True);p.add_argument('--measurement-mat',type=Path,required=True)
    p.add_argument('--measurement-field',default='test_image1');p.add_argument('--measurement-index',type=int)
    p.add_argument('--scales',type=float,nargs=3);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();data=loadmat(str(a.measurement_mat),variable_names=[a.measurement_field,'sr','sg','sb','scales'])
    y=data[a.measurement_field]
    if a.measurement_index is not None:y=y.ravel()[a.measurement_index]
    sc=a.scales if a.scales else (data['scales'].ravel() if 'scales' in data else [data[k].item() for k in ('sr','sg','sb')])
    with np.load(a.model_package,allow_pickle=False) as package:physical,f,meta=infer(package,y,sc)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(a.output,reconstruction=physical,reconstruction_normalized=f)
    a.output.with_suffix('.json').write_text(json.dumps(meta,indent=2),encoding='utf8')
    print(json.dumps(meta,indent=2))
