"""Standalone release validation. Uses examples and frozen baseline, no original dataset or GT."""
from pathlib import Path
import argparse,json,tempfile,subprocess,sys,hashlib
import numpy as np
from scipy.io import loadmat,savemat
from preboard_inference import PreparedReconstructor,input_quality


def main(root,output,all_cases=False):
    entries=json.loads((root/'input_manifest.json').read_text(encoding='utf8'));lookup={r['key']:r for r in entries}
    records=[]
    for r in entries:
        for field,hashfield in [('measurement','measurement_sha256'),('model','model_sha256')]:
            assert hashlib.sha256((root/r[field]).read_bytes()).hexdigest()==r[hashfield]
    chosen=entries if all_cases else [lookup['D11_rgb']]
    for r in chosen:
        x=loadmat(str(root/r['measurement']));engine=PreparedReconstructor(root/r['model'],root/r['cache'])
        a,meta=engine.reconstruct(x['test_image1'],x['scales'].ravel())
        ref=np.load(root/'baseline'/f"{r['key']}.npz")['reconstruction_physical']
        delta=float(np.linalg.norm(a['reconstruction']-ref)/np.linalg.norm(ref))
        assert meta['diagnostics']['split_tolerance_passed'] and delta<1e-6
        assert not meta['high_confidence'] and meta['confidence_probability'] is None
        records.append(dict(key=r['key'],relative_array_difference=delta,passed=True))
    rejected=[]
    for name,a in [('zeros',np.zeros((126,126,3))),('constant',np.ones((126,126,3))),('wrong_grid',np.ones((96,96,3))),
                   ('nan',np.full((126,126,3),np.nan)),('negative',-np.ones((126,126,3))),('overflow_range',np.full((126,126,3),1e200))]:
        try:input_quality(a)
        except ValueError:rejected.append(name)
        else:raise AssertionError(name+' not rejected')
    r=lookup['D11_rgb'];x=loadmat(str(root/r['measurement']));e=PreparedReconstructor(root/r['model'],root/r['cache'])
    for name,sc in [('zero_scale',[1,0,1]),('nan_scale',[1,np.nan,1]),('wrong_scale_length',[1,1])]:
        try:e.reconstruct(x['test_image1'],sc)
        except ValueError:rejected.append(name)
        else:raise AssertionError(name+' not rejected')
    try:PreparedReconstructor(root/r['model'],root/lookup['Chair_colbar']['cache'])
    except ValueError:rejected.append('mismatched_cache')
    else:raise AssertionError('Mismatched cache accepted')
    with tempfile.TemporaryDirectory(prefix='penumbrasight_check_') as temp:
        tmp=Path(temp);cli=Path(__file__).with_name('preboard_inference.py');out=tmp/'valid.npz'
        base=[sys.executable,str(cli),'--model',str(root/r['model']),'--cache',str(root/r['cache'])]
        valid=base+['--measurement-mat',str(root/r['measurement']),'--output',str(out)]
        done=subprocess.run(valid,capture_output=True,text=True)
        if done.returncode!=0:raise RuntimeError(done.stderr+done.stdout)
        m=json.loads(out.with_suffix('.json').read_text(encoding='utf8'));assert m['diagnostics']['split_tolerance_passed']
        a=np.load(out)['reconstruction'];ref=np.load(root/'baseline'/'d11_rgb.npz')['reconstruction_physical']
        cli_delta=float(np.linalg.norm(a-ref)/np.linalg.norm(ref));assert cli_delta<1e-6
        before=hashlib.sha256(out.read_bytes()).hexdigest()
        repeat=subprocess.run(valid,capture_output=True,text=True)
        assert repeat.returncode!=0 and hashlib.sha256(out.read_bytes()).hexdigest()==before
        rejected.append('existing_output')
        for label,arr in [('zero',np.zeros((126,126,3))),('wrong_grid',np.ones((96,96,3)))]:
            inp=tmp/(label+'.mat');bad=tmp/(label+'.npz');savemat(inp,dict(test_image1=arr,scales=np.ones(3)))
            run=subprocess.run(base+['--measurement-mat',str(inp),'--output',str(bad)],capture_output=True,text=True)
            assert run.returncode==2 and not bad.exists()
            assert json.loads(bad.with_suffix('.json').read_text(encoding='utf8'))['status']=='rejected'
            rejected.append('cli_'+label)
    result=dict(passed=True,manifest_entries_verified=len(entries),reconstructions=records,rejections=rejected,cli_success=True,cli_delta=cli_delta,
        scope='Standalone inference package, input guards, model/cache pairing, CLI exit/output and preservation checks. No board or calibrated confidence verification.')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2),encoding='utf8')
    print('RELEASE_SELFCHECK_PASS',len(records),len(rejected),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parent.parent);p.add_argument('--output',type=Path,required=True);p.add_argument('--all-cases',action='store_true');a=p.parse_args();main(a.root.resolve(),a.output,a.all_cases)
