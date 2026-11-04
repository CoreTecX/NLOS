"""Independent NumPy/SciPy references for expanded inputs; PC only."""
import os
for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[k]='1'
from pathlib import Path
import argparse,concurrent.futures,json,sys,time,traceback
import numpy as np
from prepare_dataset import decode_measurement,blob,sha,dump

def job(args):
    case,package,base,code=args;package=Path(package);base=Path(base)
    sys.path.insert(0,code)
    from preboard_inference import PreparedReconstructor
    from spatial_difference_cpu import difference_operator
    start=time.monotonic();key=case['key'];report_path=package/'references'/(key+'.json')
    if report_path.exists():
        old=json.loads(report_path.read_text('utf8'))
        if old.get('measurement_sha256')==case['measurement']['sha256'] and old.get('model_sha256')==case['model']['sha256'] and old.get('passed') and sha(package/old['golden']['file'])==old['golden']['sha256']:return old
    try:
        mid,y,sc=decode_measurement(package/case['measurement']['file'])
        geom=case['model']['geometry'];engine=PreparedReconstructor(base/'models'/(geom+'_full_int16.npz'),base/'cache'/(geom+'_prepared.npz'))
        raw=y.reshape(-1,3,order='F');ds=float(np.max(np.abs(difference_operator(raw,engine.wall,'author'))))
        h=engine.h_from_difference(difference_operator(raw,engine.wall,'spatial2d')/ds,sc/sc.max())
        f,diagnostics=engine.solver(sc/sc.max()).solve(h,lam=.001,tol=1e-7,limit=20000)
        physical=f*(ds/(engine.norm*sc.max()))
        body=b''.join(np.asarray(a,dtype='<f8',order='C').tobytes() for a in (h,f,physical))
        destination=package/'data'/(key+'.pfg');destination.write_bytes(blob(3,(29,36),mid,body))
        result=dict(key=key,passed=bool(diagnostics['split_tolerance_passed']),seconds=time.monotonic()-start,
                    diagnostics=diagnostics,model_sha256=case['model']['sha256'],measurement_sha256=case['measurement']['sha256'],
                    golden=dict(file='data/'+destination.name,sha256=sha(destination),bytes=destination.stat().st_size),
                    origin='Independent NumPy/SciPy solver and streamed original model; unchanged original tolerance; no GT input',physical_board_run=False)
    except Exception as e:result=dict(key=key,passed=False,error=str(e),traceback=traceback.format_exc())
    dump(report_path,result);return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--package',type=Path,required=True);p.add_argument('--base',type=Path,required=True);p.add_argument('--code',type=Path,required=True);p.add_argument('--workers',type=int,default=3);a=p.parse_args()
    m=json.loads((a.package/'dataset_manifest.json').read_text('utf8'))
    jobs=[(r,str(a.package),str(a.base),str(a.code)) for r in m['cases'] if r['reference_status']=='pending']
    results=[]
    with concurrent.futures.ProcessPoolExecutor(max_workers=a.workers) as executor:
        futures=[executor.submit(job,item) for item in jobs]
        for future in concurrent.futures.as_completed(futures):
            result=future.result();results.append(result)
            print('REFERENCE',len(results),'/',len(jobs),result['key'],result['passed'],round(result.get('seconds',0),2),flush=True)
    by_key={r['key']:r for r in results}
    for case in m['cases']:
        if case['key'] not in by_key:continue
        ref=by_key[case['key']]
        case['reference_status']='independent_reference_passed' if ref['passed'] else 'reference_failed'
        if 'golden' in ref:case['golden']=ref['golden']
        case['reference_report']='references/'+case['key']+'.json'
    m['state']='REFERENCES_COMPLETE' if all(r['passed'] for r in results) else 'REFERENCES_WITH_FAILURES'
    dump(a.package/'dataset_manifest.json',m)
    dump(a.package.parent/'verification/reference_summary.json',dict(jobs=len(jobs),passed=sum(r['passed'] for r in results),failures=[r for r in results if not r['passed']],physical_board_run=False))

if __name__=='__main__':main()
