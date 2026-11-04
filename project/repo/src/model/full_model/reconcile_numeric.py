"""Read-only provenance audit for historical numeric measurements."""
from pathlib import Path
import argparse,json
import numpy as np
from scipy.io import loadmat
from scipy.ndimage import zoom
from prepare_dataset import preprocess,decode_measurement,dump,sha

def main():
    p=argparse.ArgumentParser();p.add_argument('--project',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    ws=a.project/'PenumbraSight工作区';paper=a.project/'论文数据集/Ordinary-Camera-master/Data';repo=ws/'02_代码仓库/penumbrasight'
    summary=json.loads((a.output/'inventory/disposition_summary.json').read_text('utf8'));records=[]
    script=ws/'03_任务产出/Algorithm_Sprint_20261013/sprint_joint.py'
    for row in summary['numeric_review']:
        path=Path(row['path']);d11=path.parent.name=='D11';stem=path.stem
        source=paper/'TestPosD11'/('image_test_'+stem+'20.mat') if d11 else ws/'03_任务产出/S8-26_Board_Verified_20261006'/(stem+'_matlab_input.mat')
        raw=loadmat(source,variable_names=['image' if d11 else 'raw'])['image' if d11 else 'raw'].astype(float)
        ch=[raw[::2,::2],(raw[1::2,::2]+raw[::2,1::2])/2,raw[1::2,1::2]]
        img=np.stack([zoom(c,(32/c.shape[0],32/c.shape[1]),order=1) if d11 else c.reshape(32,30,32,30).mean((1,3)) for c in ch],axis=2)
        d=np.diff(img,axis=0).reshape(-1,3);d/=max(np.max(abs(d)),1e-30)
        with np.load(path,allow_pickle=False) as z:actual=z['measurement']
        exact=np.array_equal(actual,d)
        records.append(dict(path=str(path),sha256=sha(path),source=str(source),source_sha256=sha(source),generator=str(script),generator_sha256=sha(script),shape=list(actual.shape),bit_exact=bool(exact),disposition='HISTORICAL_LOCALIZATION_DERIVATIVE' if exact else 'HISTORICAL_DERIVATIVE_UNVERIFIED',reason='Normalized signed 31x32 RGB differences from historical 32x32 localization experiment, not a 126x126 wall image. Canonical Chair/D11 raw sources are already covered; do not pad or upsample these differences into PFY.'))
    path=repo/'sim/vectors/paper_meas.npy';actual=np.load(path,allow_pickle=False)
    source=paper/'TestPosD11/image_test_bur20.mat';d=loadmat(source);y=preprocess(d['image'])
    ratio=float(np.vdot(y,actual)/np.vdot(y,y));rel=float(np.linalg.norm(actual-ratio*y)/np.linalg.norm(actual))
    records.append(dict(path=str(path),sha256=sha(path),source=str(source),source_sha256=sha(source),generator=str(repo/'src/model/scripts/paper_forward.py'),shape=list(actual.shape),source_multiplier=ratio,relative_error_after_multiplier=rel,bit_exact=bool(np.array_equal(actual,ratio*y)),disposition='HISTORICAL_EXPOSURE_NORMALIZED_MEASUREMENT' if rel<1e-12 else 'UNBOUND_LEGACY_MEASUREMENT',reason='Historical paper_forward measurement; current raw preprocessing and frozen matching calibration supersede this copy. Keep historical exposure convention separate.'))
    path=repo/'sim/vectors/paper_meas_synth.npy';actual=np.load(path,allow_pickle=False)
    model=repo/'sim/vectors/paper_A_va1.npy';gt=repo/'sim/vectors/paper_gt.npy'
    expected=np.load(model,mmap_mode='r',allow_pickle=False)@np.load(gt,allow_pickle=False)[:,:,0].astype(float).ravel(order='F')
    rel=float(np.linalg.norm(actual-expected)/np.linalg.norm(actual))
    records.append(dict(path=str(path),sha256=sha(path),generator=str(repo/'src/model/scripts/synth_test.py'),model=str(model),model_sha256=sha(model),ground_truth=str(gt),ground_truth_sha256=sha(gt),shape=list(actual.shape),relative_error=rel,disposition='HISTORICAL_MONO_SYNTHETIC_MEASUREMENT' if rel<1e-12 else 'HISTORICAL_SYNTHETIC_MODEL_MISMATCH',reason='Generator describes single-channel A @ red ground truth (15876 samples), but stored fixture differs from currently saved A and GT by the recorded relative error. Original generating model/version cannot be established; retain legacy test and flag blocked. Do not invent RGB channels or feed GT as a sensor observation.'))
    dump(a.output/'inventory/numeric_provenance.json',records)
    for r in records:print(json.dumps({k:v for k,v in r.items() if k in ('path','disposition','bit_exact','relative_error','relative_error_after_multiplier','source_multiplier')},ensure_ascii=False))
    m=json.loads((a.output/'dataset/dataset_manifest.json').read_text('utf8'))
    for r in m['cases']:
        if r['reference_status']=='reference_failed':print('FAILED_REFERENCE_SOURCE',r['source'])

if __name__=='__main__':main()
