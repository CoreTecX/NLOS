from pathlib import Path
import json,hashlib,argparse
p=argparse.ArgumentParser(description='Prepare deterministic P0 synthetic inputs and independent tighter FP64 references')
p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();BASE=a.base.resolve();OUT=a.output.resolve()
if OUT==BASE or BASE in OUT.parents:raise ValueError('Cannot write into frozen reference package')
OUT.mkdir(parents=True,exist_ok=False)
CAND=Path(__file__).resolve().parents[3]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

import os,sys,struct,zlib
for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS']:os.environ[k]='1'
sys.path.insert(0,str(CAND/'src/model/full_model'))
import numpy as np
from scipy.io import loadmat
from preboard_inference import PreparedReconstructor,input_quality
from spatial_difference_cpu import difference_operator
base=BASE
rows=json.loads((base/'input_manifest.json').read_text('utf-8'));row=next(r for r in rows if r['key']=='D11_rgb')
engine=PreparedReconstructor(base/row['model'],base/row['cache'])
original=loadmat(str(base/row['measurement']));y=original['test_image1'];sc=original['scales'].ravel()
rr,cc=np.meshgrid(np.arange(126),np.arange(126),indexing='ij')
variants=[('illumination',y*(1+.02*np.sin(2*np.pi*rr/126))[:,:,None]),
          ('spatial_channels',y*(1+.015*np.cos(2*np.pi*(rr+2*cc)/126))[:,:,None]*np.array([.98,1.01,1.02]))]
out=OUT/'new_inputs';out.mkdir(exist_ok=True);records=[]
mid=bytes.fromhex(row['model_sha256'])[:16]
for name,measurement in variants:
    if (out/(name+'.json')).exists():
        diag=json.loads((out/(name+'.json')).read_text('utf-8'))['diagnostics']
        records.append(dict(name=name,model=row['model'],cache=row['cache'],source_measurement=row['measurement'],source_sha256=sha(base/row['measurement']),measurement_sha256=sha(out/(name+'.pfy')),reference_sha256=sha(out/(name+'.npz')),iterations=diag['iterations']))
        continue
    quality=input_quality(measurement)
    raw=measurement.reshape(-1,3,order='F');ds=np.max(np.abs(difference_operator(raw,engine.wall,'author')))
    h=engine.h_from_difference(difference_operator(raw,engine.wall,'spatial2d')/ds,sc/sc.max())
    # Reference convergence may use a larger offline budget. Runtime acceptance remains <=20000 at1e-7.
    reference_tol=1e-8 if name=='spatial_channels' else 1e-9
    solver=engine.solver(sc/sc.max());f,diag=solver.solve(h,tol=reference_tol,limit=60000)
    write(out/(name+'.attempt.json'),dict(reference_tolerance=reference_tol,reference_iteration_budget=60000,diagnostics=diag,previous_1e9_attempts_at_20000_and_60000_passed=False))
    assert diag['split_tolerance_passed'],name
    multiplier=ds/(engine.norm*sc.max())
    payload=np.asarray(sc,dtype='<f8').tobytes()+np.asarray(raw,dtype='<f8',order='C').tobytes()
    header=struct.pack('<8sIIIIQII16s8s',b'PSFULL1\0',1,2,126,126,len(payload),zlib.crc32(payload),0,mid,b'\0'*8)
    pfy=out/(name+'.pfy');pfy.write_bytes(header+payload)
    ref=out/(name+'.npz');np.savez_compressed(ref,h=h,reconstruction=f,reconstruction_physical=f*multiplier)
    write(out/(name+'.json'),dict(diagnostics=diag,input_quality=quality,physical_multiplier=multiplier,reference_arithmetic='independent Python NumPy/SciPy FP64; not arbitrary precision',reference_tolerance=reference_tol,reference_iteration_budget=60000,runtime_iteration_limit_unchanged=20000,tighter_reference_20000_attempt_converged=False if name=='spatial_channels' else True,reference_uses_candidate_code=False))
    original,original_diag=engine.solver(sc/sc.max()).solve(h,tol=1e-7,limit=20000)
    assert original_diag['split_tolerance_passed'],name
    acceptance=out/(name+'.acceptance.npz')
    np.savez_compressed(acceptance,h=h,reconstruction=original,reconstruction_physical=original*multiplier)
    write(out/(name+'.acceptance.json'),dict(reference='Independent frozen Python with original tolerance/limit',diagnostics=original_diag))
    records.append(dict(name=name,model=row['model'],cache=row['cache'],source_measurement=row['measurement'],source_sha256=sha(base/row['measurement']),measurement_sha256=sha(pfy),reference_sha256=sha(ref),iterations=diag['iterations'],reference_role='tighter-stopping diagnostic',acceptance_reference=acceptance.name,acceptance_reference_sha256=sha(acceptance)))
    print('NEW_REFERENCE_PASS',name,diag['iterations'],flush=True)
write(out/'manifest.json',dict(passed=True,synthetic=True,not_blind=True,no_new_sensor_acquisition=True,protocol='P0 protocol: deterministic illumination and spatial/channel modulation; no randomized tuning',cases=records))
