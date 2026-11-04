"""Export immutable frozen CPU inputs; never pass GT to target inference."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
from pathlib import Path
import sys,json,struct,zlib,hashlib,argparse
import numpy as np
from scipy.io import loadmat
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--base',type=Path,required=True,help='Frozen Preboard_Final package')
p.add_argument('--output-dir',type=Path,required=True,help='New PFM/PFY/PFG data directory')
p.add_argument('--case',help='Optional single case, otherwise all cases')
a=p.parse_args();base=a.base.resolve();out=a.output_dir.resolve()
rows=json.loads((base/'input_manifest.json').read_text(encoding='utf8'))
if a.case:
    rows=[r for r in rows if r['key']==a.case]
    if not rows:p.error('Unknown case: '+a.case)
if out==base or base in out.parents:p.error('Do not write into the frozen package')
out.mkdir(parents=True,exist_ok=False)
from preboard_inference import PreparedReconstructor
from spatial_difference_cpu import difference_operator
def save(name,kind,rows,cols,mid,parts):
    payload=b''.join(parts)
    header=struct.pack('<8sIIIIQII16s8s',b'PSFULL1\0',1,kind,rows,cols,len(payload),zlib.crc32(payload),0,mid,b'\0'*8)
    p=out/name;p.write_bytes(header+payload)
    return dict(file=name,bytes=p.stat().st_size,sha256=hashlib.sha256(header+payload).hexdigest())
def f64(x):return np.asarray(x,dtype='<f8',order='C').tobytes()
manifest=[];engines={};models={}
for row in rows:
    g=row['geometry'];key=row['key'];mid=bytes.fromhex(row['model_sha256'])[:16]
    if g not in engines:
        eng=PreparedReconstructor(base/row['model'],base/row['cache']);engines[g]=eng
        q=np.asarray(eng.package['model'][:,eng.canonical].T,dtype='<i2',order='C')
        scales=eng.package['column_scales'][eng.canonical]
        c=eng.cache
        models[g]=save(g+'.pfm',1,29,36,mid,[f64([eng.norm]),f64(scales),q.tobytes(),
            f64(c['k']),f64(c['k'].T),f64(c['ev']),f64(c['rspec']),f64(c['factor'])])
        print('MODEL',g,flush=True)
    eng=engines[g]
    m=loadmat(str(base/row['measurement']));y=m['test_image1'];sc=m['scales'].ravel()
    raw=y.reshape(-1,3,order='F');ds=np.max(np.abs(difference_operator(raw,eng.wall,'author')))
    h=eng.h_from_difference(difference_operator(raw,eng.wall,'spatial2d')/ds,sc/sc.max())
    meas=save(key+'.pfy',2,126,126,mid,[f64(sc),f64(raw)])
    with np.load(base/'baseline'/(key+'.npz')) as ref:
        gold=save(key+'.pfg',3,29,36,mid,[f64(h),f64(ref['reconstruction']),f64(ref['reconstruction_physical'])])
    manifest.append(dict(key=key,geometry=g,model_id=mid.hex(),model=models[g],measurement=meas,golden=gold,
        source_measurement_sha256=row['measurement_sha256']))
    print('EXAMPLE',key,flush=True)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
