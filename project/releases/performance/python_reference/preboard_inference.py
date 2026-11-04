"""Guarded, streamed CPU reconstruction. Diagnostics are NOT confidence probabilities."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
from pathlib import Path
import argparse,hashlib,json,time
import numpy as np
from scipy.io import loadmat
from scipy.ndimage import median_filter
from infer_stored_model import prepare_operator
from factorized_tv_cpu import FactorizedTV
from spatial_difference_cpu import difference_operator

CACHE_VERSION=1


def json_write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf8')


def input_quality(measurement,wall_shape=(126,126),full_scale=None,allow_signed=False):
    y=np.asarray(measurement,dtype=float)
    if y.shape!=tuple(wall_shape)+(3,):raise ValueError(f'测量必须为 {wall_shape[0]}×{wall_shape[1]}×3；不能输入对照图或直接缩放其他网格')
    if not np.isfinite(y).all():raise ValueError('测量含 NaN 或无穷大')
    if not allow_signed and np.any(y<0):raise ValueError('原始测量含负值；仅显式背景扣除数据允许负值')
    raw=y.reshape(-1,3,order='F');d=difference_operator(raw,wall_shape,'spatial2d')
    ynorm=float(np.linalg.norm(y));dnorm=float(np.linalg.norm(d))
    if not np.isfinite([ynorm,dnorm]).all():raise ValueError('输入数值范围过大，空间差分或范数溢出')
    if ynorm==0 or dnorm<=1e-12*max(1,ynorm):raise ValueError('测量没有可用的空间变化，拒绝输出重建')
    cross=y[1:,1:]-y[1:,:-1]-y[:-1,1:]+y[:-1,:-1]
    center=np.median(cross,axis=(0,1));noise=np.median(np.abs(cross-center),axis=(0,1))/(2*.67448975)
    rms=float(np.sqrt(np.mean(d*d)));noise_rms=float(np.sqrt(np.mean(noise*noise)))
    noise_proxy=rms/max(noise_rms,1e-30)
    local=y-median_filter(y,size=(3,3,1),mode='nearest')
    normal=float(np.percentile(np.abs(local),99));outlier_ratio=float(np.max(np.abs(local))/max(normal,1e-30))
    flags=[]
    if noise_proxy<3:flags.append('low_gradient_to_noise_proxy')
    if outlier_ratio>30:flags.append('isolated_pixel_outlier')
    channel_norm=np.linalg.norm(raw,axis=0)
    if np.any(channel_norm<1e-10*max(float(channel_norm.max()),1e-30)):flags.append('empty_rgb_channel')
    saturation=None
    if full_scale is not None:
        if not np.isfinite(full_scale) or full_scale<=0:raise ValueError('满量程必须为有限正值，且与输入累计/平均口径一致')
        saturation=float(np.mean(y>=full_scale))
        if saturation>.001:flags.append('possible_saturation')
    return dict(flags=flags,gradient_relative_norm=dnorm/ynorm,gradient_rms=rms,
        spatial_noise_proxy_per_channel=noise.tolist(),gradient_to_noise_proxy=min(noise_proxy,1e30),
        isolated_outlier_ratio=min(outlier_ratio,1e30),saturation_fraction=saturation,
        note='Spatial noise proxy may include scene edges; thresholds are engineering diagnostics, not calibrated sensor noise or confidence')


def build_cache(model_path,cache_path):
    tic=time.perf_counter()
    with np.load(model_path,allow_pickle=False) as package:
        b,f=prepare_operator(package)
        scene=tuple(int(x) for x in package['scene_shape']);solver=FactorizedTV(f,np.ones(3),scene)
        gram_error=float(np.linalg.norm(f.T@f-b.T@b)/np.linalg.norm(b.T@b))
        assert gram_error<1e-10
    cache_path.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(cache_path,version=CACHE_VERSION,model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
        factor=solver.F,k=solver.k,ev=solver.ev,rspec=solver.rspec,scene_shape=scene)
    return dict(seconds=time.perf_counter()-tic,gram_factor_relative_error=gram_error,cache_sha256=hashlib.sha256(cache_path.read_bytes()).hexdigest())


class PreparedReconstructor:
    def __init__(self,model_path,cache_path,block_columns=32):
        self.model_path=Path(model_path);self.cache_path=Path(cache_path)
        if not isinstance(block_columns,int) or block_columns<1:raise ValueError('block_columns must be positive')
        self.block_columns=block_columns
        with np.load(self.model_path,allow_pickle=False) as p:self.package={k:p[k] for k in p.files}
        with np.load(self.cache_path,allow_pickle=False) as p:self.cache={k:p[k] for k in p.files}
        if int(self.cache['version'])!=CACHE_VERSION:raise ValueError('Unsupported prepared cache version')
        if str(self.cache['model_sha256'])!=hashlib.sha256(self.model_path.read_bytes()).hexdigest():raise ValueError('缓存与模型不匹配，必须重新准备缓存')
        self.wall=tuple(int(x) for x in self.package['wall_shape']);self.scene=tuple(int(x) for x in self.package['scene_shape'])
        self.norm=float(self.package['model_norm']);self.n=int(np.prod(self.scene))
        if not np.isfinite(self.norm) or self.norm<=0:raise ValueError('Invalid model norm')
        self.canonical=np.arange(self.n).reshape(*self.scene,order='F').ravel()
        if self.package['model'].shape!=(int(np.prod(self.wall)),self.n):raise ValueError('Model shape mismatch')
        for k,shape in [('factor',(self.n,self.n)),('k',(self.n,self.n)),('ev',(self.n,)),('rspec',self.scene)]:
            if self.cache[k].shape!=shape or not np.isfinite(self.cache[k]).all():raise ValueError(f'Invalid cache {k}')

    def solver(self,cc):
        s=FactorizedTV.__new__(FactorizedTV)
        s.F=self.cache['factor'];s.k=self.cache['k'];s.ev=self.cache['ev'];s.rspec=self.cache['rspec']
        s.shape=self.scene;s.cc=np.asarray(cc,dtype=float);s.rho=.01
        return s

    def h_from_difference(self,d,cc):
        h=np.empty((self.n,3));coeff=self.package['model']
        for start in range(0,self.n,self.block_columns):
            end=min(start+self.block_columns,self.n);indices=self.canonical[start:end]
            a=coeff[:,indices].astype(float)
            if 'column_scales' in self.package:a*=self.package['column_scales'][indices]
            b=difference_operator(a,self.wall,'spatial2d')/self.norm
            h[start:end]=b.T@d*cc
        return h

    def reconstruct(self,measurement,scales,lam=.001,stability=False,full_scale=None,allow_signed=False):
        tic=time.perf_counter();y=np.asarray(measurement,dtype=float);sc=np.asarray(scales,dtype=float)
        if sc.shape!=(3,) or not np.isfinite(sc).all() or np.any(sc<=0):raise ValueError('必须提供三个有效的正RGB标定系数')
        if not np.isfinite(lam) or lam<=0:raise ValueError('Invalid regularization')
        quality=input_quality(y,self.wall,full_scale,allow_signed)
        raw=y.reshape(-1,3,order='F');ds=float(np.max(np.abs(difference_operator(raw,self.wall,'author'))))
        d=difference_operator(raw,self.wall,'spatial2d')/ds;cc=sc/sc.max()
        if np.any(cc<=0):raise ValueError('RGB标定动态范围超出float64可表示范围')
        h=self.h_from_difference(d,cc)
        s=self.solver(cc);f,diag=s.solve(h,lam=lam)
        multiplier=ds/(self.norm*sc.max());physical=f*multiplier
        if not np.isfinite(multiplier) or multiplier<=0 or not np.isfinite(physical).all():raise ArithmeticError('物理重建超出float64可表示范围')
        metadata=dict(input_quality=quality,diagnostics=diag,lambda_value=lam,
            normalization=dict(model=self.norm,measurement=ds,channel=cc.tolist(),physical_multiplier=multiplier),
            high_confidence=False,confidence_probability=None,status='uncalibrated',
            uncertainty_note='No independent accuracy calibration; solver residuals and stability are separate diagnostics',
            arithmetic='float64, complete INT16-stored model',stability=None)
        maps={}
        if not diag['split_tolerance_passed']:metadata['status']='numerical_failure'
        if stability and diag['split_tolerance_passed']:
            rng=np.random.default_rng(20261014)
            noise=rng.normal(size=y.shape)
            dr=difference_operator(noise.reshape(-1,3,order='F'),self.wall,'spatial2d')
            dr*=.01*np.linalg.norm(d)/max(np.linalg.norm(dr),1e-30)
            hn=self.h_from_difference(dr,cc)
            arr=[];results=[]
            tests=[('noise_plus',h+hn,lam),('noise_minus',h-hn,lam),('lambda_half',h,lam/2),('lambda_double',h,lam*2)]
            for name,hp,lp in tests:
                fp,dd=self.solver(cc).solve(hp,lam=lp)
                candidate=fp*multiplier;gain=max(0,float(np.sum(candidate*physical)/max(np.sum(candidate*candidate),1e-30)))
                delta=float(np.linalg.norm(candidate*gain-physical)/max(np.linalg.norm(physical),1e-30))
                arr.append(candidate);results.append(dict(test=name,shape_difference=delta,diagnostics=dd))
            sensitivity=np.max(np.linalg.norm(np.stack(arr)-physical,axis=-1),axis=0)/max(float(np.max(np.linalg.norm(physical,axis=-1))),1e-30)
            maps=dict(sensitivity_map=sensitivity,unstable_mask=(sensitivity>.05))
            passed=all(r['diagnostics']['split_tolerance_passed'] for r in results)
            sensitive=any(r['shape_difference']>.05 for r in results)
            metadata['stability']=dict(tests=results,engineering_shape_threshold=.05,all_solvers_passed=passed,
                sensitive=sensitive,unstable_pixel_fraction=float(np.mean(sensitivity>.05)),
                note='Prescribed synthetic sensitivity, not a probability or noise confidence interval')
            if not passed:metadata['status']='stability_numerical_failure'
            elif sensitive:metadata['status']='sensitive'
        metadata['seconds_total']=time.perf_counter()-tic
        return dict(reconstruction=physical,reconstruction_normalized=f,**maps),metadata


def read_measurement(path,field,index,scales):
    x=loadmat(str(path),variable_names=[field,'sr','sg','sb','scales'])
    if field not in x:raise ValueError(f'MAT中缺少测量字段 {field}')
    y=x[field]
    if index is not None:y=y.ravel()[index]
    if scales is None:
        if 'scales' in x:scales=x['scales'].ravel()
        elif all(k in x for k in ['sr','sg','sb']):scales=[x[k].item() for k in ['sr','sg','sb']]
        else:raise ValueError('缺少RGB标定系数；请用 --scales 提供')
    return y,scales


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True);p.add_argument('--cache',type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true');p.add_argument('--measurement-mat',type=Path)
    p.add_argument('--field',default='test_image1');p.add_argument('--index',type=int);p.add_argument('--scales',type=float,nargs=3)
    p.add_argument('--lambda-value',type=float,default=.001);p.add_argument('--stability',action='store_true')
    p.add_argument('--full-scale',type=float);p.add_argument('--allow-signed',action='store_true');p.add_argument('--output',type=Path)
    a=p.parse_args()
    if a.prepare_only:
        print(json.dumps(build_cache(a.model,a.cache),indent=2));raise SystemExit(0)
    if a.measurement_mat is None or a.output is None:p.error('--measurement-mat and --output are required for inference')
    if a.output.suffix.lower()!='.npz':p.error('输出文件扩展名必须为 .npz')
    if a.output.exists() or a.output.with_suffix('.json').exists():p.error('输出已存在，请使用新的输出文件名以保留原结果')
    try:
        y,sc=read_measurement(a.measurement_mat,a.field,a.index,a.scales)
        engine=PreparedReconstructor(a.model,a.cache)
        arrays,meta=engine.reconstruct(y,sc,a.lambda_value,a.stability,a.full_scale,a.allow_signed)
        a.output.parent.mkdir(parents=True,exist_ok=True);np.savez_compressed(a.output,**arrays)
        json_write(a.output.with_suffix('.json'),meta)
        print(json.dumps({k:meta[k] for k in ['status','high_confidence','seconds_total']},ensure_ascii=False))
        if 'failure' in meta['status']:raise SystemExit(2)
    except (ValueError,KeyError,IndexError,ArithmeticError) as exc:
        json_write(a.output.with_suffix('.json'),dict(status='rejected',reason=str(exc),high_confidence=False))
        print(str(exc));raise SystemExit(2)
