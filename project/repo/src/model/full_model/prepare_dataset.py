"""Offline inventory and exact input conversion for the current full-model release.

Never opens a serial port, loads an FPGA, or changes the frozen releases.
"""
from __future__ import annotations
import os
for _key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_key]='1'
import argparse, collections, csv, hashlib, json, re, shutil, struct, sys, zlib
from pathlib import Path
import numpy as np
from scipy.io import loadmat, whosmat

SOFTWARE='20261017-p789-p1011-v1'
HARDWARE='20261016-rp2-p56-v1'
SCHEMA='psight-dataset-v1'
EXTENSIONS={'.mat','.npz','.npy','.psf','.pfy','.png','.jpg','.jpeg','.tif','.tiff','.bmp'}

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

def dump(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')

def csv_write(path,rows,fields):
    with Path(path).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader()
        for row in rows:
            w.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict)) else v for k,v in row.items() if k in fields})

def preprocess(raw):
    raw=np.asarray(raw,dtype=np.float64)
    if raw.shape!=(2016,2016) or not np.isfinite(raw).all() or np.any(raw<0):
        raise ValueError('Expected finite nonnegative 2016x2016 RGGB sensor measurement')
    rgb=np.stack([raw[::2,::2],(raw[1::2,::2]+raw[::2,1::2])/2,raw[1::2,1::2]],axis=-1)
    for _ in range(3):
        rgb=(rgb[::2,::2]+rgb[::2,1::2]+rgb[1::2,::2]+rgb[1::2,1::2])/4
    return rgb

def payload(y,scales):
    y=np.asarray(y,dtype=np.float64);sc=np.asarray(scales,dtype=np.float64)
    if y.shape!=(126,126,3) or sc.shape!=(3,) or not np.isfinite(y).all() or not np.isfinite(sc).all() or np.any(y<0) or np.any(sc<=0):
        raise ValueError('Invalid current-runtime measurement or RGB scales')
    with np.errstate(over='ignore',invalid='ignore'):
        yn=float(np.linalg.norm(y));dn=float(np.linalg.norm(np.diff(y,axis=0))+np.linalg.norm(np.diff(y,axis=1)))
    if not np.isfinite([yn,dn]).all():raise ValueError('Measurement range overflows FP64 norm')
    if dn<=1e-12*max(1.,yn):
        raise ValueError('No useful spatial variation')
    return sc.astype('<f8').tobytes()+np.asarray(y.reshape(-1,3,order='F'),dtype='<f8',order='C').tobytes()

def blob(kind,shape,identity,body):
    return struct.pack('<8sIIIIQII16s8s',b'PSFULL1\0',1,kind,*shape,len(body),zlib.crc32(body),0,bytes.fromhex(identity),bytes(8))+body

def decode_measurement(path):
    raw=Path(path).read_bytes()
    if len(raw)!=64+(3+126*126*3)*8 or raw[:8]!=b'PSFULL1\0' or struct.unpack_from('<IIII',raw,8)!=(1,2,126,126):raise ValueError('Invalid PFY shape/header')
    if struct.unpack_from('<Q',raw,24)[0]!=len(raw)-64 or raw[36:40]!=bytes(4) or raw[56:64]!=bytes(8) or zlib.crc32(raw[64:])!=struct.unpack_from('<I',raw,32)[0]:raise ValueError('PFY integrity failure')
    a=np.frombuffer(raw[64:],dtype='<f8');y=a[3:].reshape(126*126,3).reshape(126,126,3,order='F');sc=a[:3]
    if payload(y,sc)!=raw[64:]:raise ValueError('PFY roundtrip failure')
    return raw[40:56].hex(),y,sc

def original_mapping():
    m={f'Chair_{k}':f'TestPosChair/image_test_{k}.mat' for k in ['mushroom','smilehat','colbar','bur']}
    m.update(S14_colbar='TestPosChairRealscene/image_test_colbar.mat',S14_res3='TestPosChairRealscene/image_test_res3.mat',S15_image3='TestPosChair3DScene/image_test_3.mat')
    for suffix,stem in [('bu','bur'),('rgb','colbar'),('mushroom','mushroom'),('tommy','smilehat')]:
        m['D11_'+suffix]=f'TestPosD11/image_test_{stem}20.mat'
        m['D11_new_'+stem]=f'TestPosD11/image_test_{stem}.mat'
    return m

def paper_links(rel):
    if rel.startswith('TestPosD11/') and any(x+'20.mat' in rel for x in ['bur','colbar','mushroom','smilehat']):
        return ['Figure 4 C/D/E (same source data; algorithms differ)','Figures S1/S2','Table S1 (tommy filename typo recorded separately)']
    if rel.startswith('TestPosChair/') and Path(rel).name in {'image_test_'+s+'.mat' for s in ['mushroom','smilehat','colbar','bur']}:
        return ['Figure S9','Figure S16']
    if rel in ['TestPosChairRealscene/image_test_colbar.mat','TestPosChairRealscene/image_test_res3.mat']:return ['Figure S14']
    if rel=='TestPosChair3DScene/image_test_3.mat':return ['Figure S15']
    return []

def pin_releases(ws,out):
    releases=ws/'03_任务产出/P7_P11_Final_20261017/release'
    pins={}
    for name,profile in [('performance','p56_150_b128'),('fallback','p56_100')]:
        root=releases/name;m=json.loads((root/'release_manifest.json').read_text('utf8'))
        if m['software_revision']!=SOFTWARE or m['hardware_release']!=HARDWARE or m['hardware_profile']!=profile or m['layout_version']!=0x32525001:raise ValueError('Release identity mismatch')
        failures=[]
        for rel,v in m['files'].items():
            p=root/rel
            if not p.is_file() or p.stat().st_size!=v['bytes'] or sha(p)!=v['sha256']:failures.append(rel)
        if failures:raise ValueError('Frozen release integrity failure: '+str(failures))
        pins[name]=dict(software_revision=SOFTWARE,hardware_release=HARDWARE,profile=profile,
                        release_manifest_sha256=sha(root/'release_manifest.json'),verified_files=len(m['files']),
                        source=str(root),files=m['files'])
        print('PINNED',name,len(m['files']),flush=True)
    lock=dict(schema=SCHEMA,software_revision=SOFTWARE,hardware_release=HARDWARE,layout_abi='0x32525001',packages=pins)
    dump(out/'release_lock.json',lock)
    return lock

def mat_metadata(path):
    try:return dict(variables=[dict(name=n,shape=list(s),dtype=t) for n,s,t in whosmat(path)])
    except Exception as e:
        # MATLAB class objects in ancillary files can defeat whosmat; selected fields remain readable.
        try:
            d=loadmat(path,variable_names=['image','test_image1','measurements','ground_truth','scales','sr','sg','sb'])
            return dict(variables=[dict(name=n,shape=list(np.shape(v)),dtype=str(np.asarray(v).dtype)) for n,v in d.items() if not n.startswith('__')],header_warning=str(e))
        except Exception as second:return dict(variables=[],inspection_error=str(second))

def inventory(project,ws,out):
    paper=project/'论文数据集/Ordinary-Camera-master'
    scopes=[('paper',paper),('workspace',ws),('competition',project/'NLOS赛题')]
    assets=[];errors=[];seen_hash={}
    for scope,root in scopes:
        for directory,dirs,files in os.walk(root,onerror=lambda e:errors.append(str(e))):
            dirs[:]=sorted(d for d in dirs if d not in {'.git','.Xil','__pycache__','node_modules','Board_Steps_1_4_20261017','board_steps_1_4_20261017'})
            for name in sorted(files):
                path=Path(directory)/name
                if path.suffix.lower() not in EXTENSIONS:continue
                rel=path.relative_to(root).as_posix();ext=path.suffix.lower()
                row=dict(scope=scope,path=str(path),relative=rel,bytes=path.stat().st_size,sha256=sha(path))
                row['duplicate_of']=seen_hash.get(row['sha256']);seen_hash.setdefault(row['sha256'],row['path'])
                if scope=='paper' and rel.startswith('Data/'):
                    row.update(mat_metadata(path));v={x['name']:x for x in row['variables']}
                    row['role']='raw_measurement' if 'image' in v else 'auxiliary_localization_or_model'
                    row['paper_figures']=paper_links(rel[5:])
                elif ext=='.pfy':row['role']='prepared_measurement_candidate'
                elif ext=='.psf':row['role']='legacy_quantized_input_requires_original_source'
                elif ext=='.mat':
                    row.update(mat_metadata(path));v={x['name']:x for x in row['variables']}
                    row['role']='raw_or_prepared_candidate' if any(n in v for n in ['image','test_image1','measurements']) else 'derived_or_auxiliary'
                elif ext in {'.png','.jpg','.jpeg','.tif','.tiff','.bmp'}:row['role']='display_reference_or_document_image_not_sensor_input'
                elif ext=='.npy':
                    try:row['array_shape']=list(np.load(path,mmap_mode='r',allow_pickle=False).shape)
                    except Exception as e:row['inspection_error']=str(e)
                    row['role']='legacy_numeric_array_requires_geometry_and_scale_contract'
                else:
                    try:
                        with np.load(path,allow_pickle=False) as a:row['array_keys']=a.files
                    except Exception as e:row['inspection_error']=str(e)
                    row['role']='derived_model_reference_or_solver_state'
                assets.append(row)
                if len(assets)%200==0:print('INVENTORY',len(assets),flush=True)
    dump(out/'inventory/assets.json',assets)
    csv_write(out/'inventory/assets.csv',assets,['scope','relative','path','role','bytes','sha256','duplicate_of','paper_figures','inspection_error'])
    summary=dict(total_files=len(assets),unique_content=len(seen_hash),roles=dict(collections.Counter(r['role'] for r in assets)),
                 scope_counts=dict(collections.Counter(r['scope'] for r in assets)),scan_errors=errors,
                 excluded='Top-level zip/rar and dated snapshots are archival containers, not active dataset sources; source and output copies are retained in this inventory.')
    dump(out/'inventory/summary.json',summary)
    if errors:raise RuntimeError('Inventory traversal errors recorded; do not claim complete coverage')
    print('INVENTORY_DONE',json.dumps(summary,ensure_ascii=False),flush=True)
    return assets

def prepare(ws,project,out,assets):
    package=out/'dataset'
    if (package/'package_manifest.json').exists():raise ValueError('Dataset is already sealed; reproduce into a new output directory')
    (package/'data').mkdir(parents=True,exist_ok=True)
    paper=project/'论文数据集/Ordinary-Camera-master';base=ws/'03_任务产出/Preboard_Final_20261014'
    release=ws/'03_任务产出/P7_P11_Final_20261017/release/performance'
    rows=json.loads((release/'data/manifest.json').read_text('utf8'));by_key={r['key']:r for r in rows}
    originals=original_mapping();reverse={v:k for k,v in originals.items()}
    model_by_id={r['model_id']:r for r in rows}
    model_files={}
    for row in rows:
        if row['model_id'] in model_files:continue
        source=release/'data'/row['model']['file'];dst=package/'data'/row['model']['file'];shutil.copy2(source,dst)
        model_files[row['model_id']]=dict(file='data/'+dst.name,sha256=sha(dst),bytes=dst.stat().st_size,geometry=row['geometry'])
    frame_evidence={}
    for folder,key in [('Mush','Chair_mushroom'),('tommy','Chair_smilehat')]:
        files=sorted((paper/'Data/TestPosChair'/folder).glob('image_test_it*.mat'))
        total=None
        for path in files:
            raw=np.asarray(loadmat(path,variable_names=['image'])['image'],dtype=np.float64)
            total=raw.copy() if total is None else total+raw
        aggregate=loadmat(paper/'Data'/originals[key],variable_names=['image'])['image']
        if len(files)!=35 or not np.array_equal(total,aggregate):raise ValueError('Single-frame accumulation provenance failed: '+folder)
        frame_evidence[folder]=dict(count=len(files),aggregate_source=originals[key],exact_pixel_sum=True,
                                   scale_policy='Frozen aggregate RGB scales divided by verified number of component frames; same scene/geometry, no GT fitting')
    dump(out/'adaptation/frame_accumulation.json',frame_evidence)
    statuses=[];cases=[];identity_to_key={};baseline_checks=[]

    def add_case(key,model_id,y,sc,source,policy,figures=None,baseline=None):
        body=payload(y,sc);content=hashlib.sha256(bytes.fromhex(model_id)+body).hexdigest()
        if content in identity_to_key:return identity_to_key[content]
        path=package/'data'/(key+'.pfy');path.write_bytes(blob(2,(126,126),model_id,body))
        decoded=decode_measurement(path)
        if not np.array_equal(decoded[1],y) or not np.array_equal(decoded[2],sc):raise ValueError('Conversion roundtrip failed')
        entry=dict(key=key,model_id=model_id,model=model_files[model_id],measurement=dict(file='data/'+path.name,sha256=sha(path),bytes=path.stat().st_size),
                   input_kind='paper_measurement' if source.startswith('TestPos') else 'synthetic_or_engineering_test_input',
                   source=source,adaptation_policy=policy,paper_figures=figures or [],baseline_case=baseline,reference_status='pending',physical_board_run=False)
        if baseline:
            gold=release/'data'/by_key[baseline]['golden']['file'];dest=package/'data'/(key+'.pfg');shutil.copy2(gold,dest)
            entry['golden']=dict(file='data/'+dest.name,sha256=sha(dest),bytes=dest.stat().st_size)
            entry['reference_status']='frozen_independent_reference'
        cases.append(entry);identity_to_key[content]=key
        return key

    for asset in assets:
        if asset['scope']!='paper' or not asset['relative'].startswith('Data/'):continue
        rel=asset['relative'][5:];status=dict(source=rel,sha256=asset['sha256'],role=asset['role'],paper_figures=asset.get('paper_figures',[]))
        if asset['role']!='raw_measurement':
            status.update(status='AUXILIARY',reason='Localization/model data without raw image field; retained as supporting evidence')
        else:
            data=loadmat(asset['path'],variable_names=['image','pixel_size','shutter_speed','iterations','averaged_iterations'])
            try:y=preprocess(data['image'])
            except ValueError as e:status.update(status='BLOCKED',reason=str(e));statuses.append(status);continue
            key=reverse.get(rel)
            if key:
                row=by_key[key];mid,existing,sc=decode_measurement(release/'data'/row['measurement']['file'])
                if not np.array_equal(y,existing):raise ValueError('Original/raw preprocessing differs from frozen input: '+key)
                baseline_checks.append(dict(key=key,bit_exact=True,source=rel))
                case_key=add_case(key,mid,y,sc,rel,'Exact frozen case with bit-exact original RAW preprocessing',status['paper_figures'],key)
                status.update(status='READY',case=case_key,calibration='frozen_case')
            elif rel.startswith(('TestPosChair/Mush/','TestPosChair/tommy/')):
                folder=rel.split('/')[1];basekey='Chair_mushroom' if folder=='Mush' else 'Chair_smilehat';row=by_key[basekey]
                mid,_,sc=decode_measurement(release/'data'/row['measurement']['file'])
                key='Chair_'+folder+'_'+Path(rel).stem
                case_key=add_case(key,mid,y,sc/35,rel,frame_evidence[folder]['scale_policy'])
                status.update(status='READY',case=case_key,calibration='derived_from_exact_35_frame_sum',quality_boundary='Single-frame noise and physical intensity calibration not independently measured')
            else:
                group=rel.split('/')[0]
                if group=='TestPosD11Video':reason='Different D11Video FOV/occluder; no matching frozen full model or per-sequence RGB calibration'
                elif group=='TestPosChairAmb':reason='ChairAmb geometry differs; missing matching full model, RGB calibration and documented background pairing'
                elif group=='TestPosChair3DScene':reason='Only image_test_3 has verified S15 calibration; other captures need geometry/exposure/model applicability evidence'
                elif group=='TestPosChairRealscene':reason='Only colbar/res3 have frozen S14 source-to-calibration mapping; extra real-scene captures need calibration evidence'
                else:reason='No verified source-to-model/RGB-scale association; ambient/background/unknown captures cannot inherit a scene calibration by filename alone'
                status.update(status='BLOCKED',reason=reason)
        statuses.append(status)
    # Include existing current-format project inputs, deduplicated by model plus exact measurement payload.
    pfy_sources=[]
    for asset in assets:
        if asset['role']!='prepared_measurement_candidate':continue
        rec=dict(source=asset['path'],sha256=asset['sha256'])
        try:
            mid,y,sc=decode_measurement(asset['path'])
            if mid not in model_by_id:raise ValueError('PFY model identity has no pinned current full model')
            content=hashlib.sha256(bytes.fromhex(mid)+payload(y,sc)).hexdigest()
            if content in identity_to_key:key=identity_to_key[content]
            else:key=add_case('Project_'+asset['sha256'][:16],mid,y,sc,asset['path'],'Existing PFY identity, CRC, shape and semantic validation; source synthesis/measurement provenance remains explicit')
            rec.update(status='READY',case=key)
        except (OSError,ValueError,KeyError) as e:rec.update(status='BLOCKED',reason=str(e))
        pfy_sources.append(rec)
    dump(out/'adaptation/paper_cases.json',statuses);dump(out/'adaptation/project_pfy_sources.json',pfy_sources)
    csv_write(out/'adaptation/paper_cases.csv',statuses,['source','role','status','case','calibration','paper_figures','reason','sha256'])
    dump(out/'adaptation/preprocess_baseline_checks.json',baseline_checks)
    manifest=dict(schema=SCHEMA,software_revision=SOFTWARE,hardware_release=HARDWARE,cases=cases,
                  blocked_paper=[s for s in statuses if s['status']=='BLOCKED'],paper_assets=len(statuses),
                  paper_status_counts=dict(collections.Counter(s['status'] for s in statuses)),
                  state='INPUTS_PREPARED_REFERENCES_PENDING',physical_board_run=False)
    dump(package/'dataset_manifest.json',manifest)
    dump(out/'adaptation/summary.json',dict(paper=manifest['paper_status_counts'],unique_cases=len(cases),
                                           preprocessing_exact_baselines=len(baseline_checks),frame_groups=frame_evidence,
                                           project_pfy_status=dict(collections.Counter(r['status'] for r in pfy_sources))))
    print('PREPARED',len(cases),manifest['paper_status_counts'],flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['pin','inventory','prepare'])
    p.add_argument('--project',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    ws=a.project/'PenumbraSight工作区';a.output.mkdir(parents=True,exist_ok=True)
    if a.action=='pin':pin_releases(ws,a.output)
    elif a.action=='inventory':inventory(a.project,ws,a.output)
    else:prepare(ws,a.project,a.output,json.loads((a.output/'inventory/assets.json').read_text('utf8')))

if __name__=='__main__':main()
