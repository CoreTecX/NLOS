"""Durable ordered complete reconstruction with verified resume and per-item status."""
from pathlib import Path
from types import SimpleNamespace
import argparse,json,time
from run import execute,REPO
from contracts import read_json,pinned,require,sha,InputError
from native_backend import within
from resources import VerifiedResources
from batch_store import atomic_json,WriterLock,seal_result,verify_seal,verify_inputs,append_event,file_hash,safe_child
from batch_gallery import publish_gallery

def implementation_hash():
 paths=[Path(__file__),Path(__file__).with_name('run.py'),Path(__file__).with_name('batch_store.py'),Path(__file__).with_name('batch_gallery.py')]
 for name in ('contracts.py','native_backend.py','resources.py','inputs.py'):paths.append(REPO/'src/software/camera_service/production_input'/name)
 paths.append(REPO/'src/software/meas_norm/raw_measurement.py')
 return sha(json.dumps([(p.name,file_hash(p)) for p in paths],sort_keys=True).encode())

def run_batch_session(manifest,config,workspace,output,resume=False,retry_failed=False,stop_requested=None,timeout=1800,on_commit=None):
 manifest=Path(manifest).resolve();config=Path(config).resolve();workspace=Path(workspace).resolve();output=Path(output).resolve()
 raw=manifest.read_bytes();spec=json.loads(raw);cfg=read_json(config);ids=[r['id'] for r in spec['items']]
 require(spec['schema_version']==1 and 0<len(ids)<=100000 and all(isinstance(x,str) and x for x in ids) and len(set(ids))==len(ids),'BATCH_MANIFEST','Bounded distinct identities required')
 require(not within(output,workspace) or within(output,workspace/'99_临时工作区'),'OUTPUT_LOCATION','Use workspace temporary output area')
 identity=dict(manifest_sha256=sha(raw),config_sha256=file_hash(config),implementation_sha256=implementation_hash(),descriptor_base=str(manifest.parent),workspace=str(workspace))
 if not resume:output.mkdir(parents=True,exist_ok=False)
 else:require(output.is_dir(),'BATCH_RESUME','No previous batch directory')
 started=time.monotonic();resources=None
 with WriterLock(output):
  checkpoint_path=output/'checkpoint.json'
  if checkpoint_path.exists():
   require(resume,'BATCH_EXISTS','Use resume for existing checkpoint');checkpoint=read_json(checkpoint_path)
   require(checkpoint['identity']==identity,'BATCH_IDENTITY','Manifest/config/implementation/source base changed; use a new batch')
   require(len(checkpoint['items'])==len(ids) and all(r['id']==ids[i] and r['index']==i for i,r in enumerate(checkpoint['items'])),'BATCH_CHECKPOINT','Checkpoint item identities differ')
  else:
   require(not resume,'BATCH_CHECKPOINT','Missing checkpoint; keep incomplete files and use a new output')
   checkpoint=dict(schema_version=1,identity=identity,items=[dict(id=x,index=i,status='PENDING',attempts=0) for i,x in enumerate(ids)])
   atomic_json(checkpoint_path,checkpoint)
  try:
   if cfg.get('resource_cache_enabled',False):resources=VerifiedResources(cfg.get('resource_cache_bytes',64*1024*1024))
   for i,item in enumerate(spec['items']):
    row=checkpoint['items'][i]
    if stop_requested and stop_requested():break
    desc=None;base=None
    try:
     desc=json.loads(pinned(manifest.parent,item['input']));base=(manifest.parent/item['input']['path']).resolve().parent
     if row['status']=='COMPLETE':
      verify_inputs(desc,base);verify_seal(output,row);continue
     if row['status'] in ('FAILED','BLOCKED') and not retry_failed:continue
    except (InputError,OSError,ValueError,KeyError,TypeError) as error:
     # A previously complete item is no longer trustworthy; keep its old files.
     row.update(status='FAILED',error_code=getattr(error,'code',type(error).__name__),error=str(error),previous_output=row.get('output'))
     row.pop('seal',None);row.pop('output',None);atomic_json(checkpoint_path,checkpoint)
     append_event(output/'ledger.jsonl',dict(row,event='INPUT_OR_RESUME_REJECTED'))
     continue
    row.update(status='RUNNING',attempts=row['attempts']+1);row.pop('error',None);row.pop('error_code',None)
    folder=output/('item_%06d'%i)/('attempt_%03d'%row['attempts'])
    while folder.exists():
     row['attempts']+=1;folder=output/('item_%06d'%i)/('attempt_%03d'%row['attempts'])
    folder.mkdir(parents=True,exist_ok=False)
    row['output']=(folder/'result').relative_to(output).as_posix();row.pop('seal',None)
    atomic_json(checkpoint_path,checkpoint);append_event(output/'ledger.jsonl',dict(row,event='STARTED'))
    try:
     result=execute(SimpleNamespace(config=config,workspace=workspace,output=folder/'result',case=None,raw_sample=None,input=manifest.parent/item['input']['path'],
                         action='reconstruct',runtime_root=None,timeout=timeout),resources=resources,input_snapshot=(desc,base))
     require(result['status'] in ('COMPLETE','FAILED','BLOCKED'),'BATCH_RESULT','Unexpected pipeline status')
     row.update(status=result['status'],error_code=result.get('error_code'),hardware_accessed=result.get('hardware_accessed',False))
     if row['status']=='COMPLETE':row['seal']=seal_result(folder/'result',desc)
    except (InputError,OSError,ValueError,KeyError,TypeError) as error:
     row.update(status='FAILED',error_code=getattr(error,'code',type(error).__name__),error=str(error));row.pop('seal',None)
    atomic_json(folder/'receipt.json',row);atomic_json(checkpoint_path,checkpoint)
    append_event(output/'ledger.jsonl',dict(row,event='COMMITTED'))
    try:checkpoint['display']=publish_gallery(output,checkpoint)
    except Exception as error:checkpoint['display']=dict(status='FAILED',error=str(error))
    atomic_json(checkpoint_path,checkpoint)
    if on_commit:on_commit(dict(row))
  finally:
   # Repair a missed gallery update after process interruption without rerunning a good solve.
   try:checkpoint['display']=publish_gallery(output,checkpoint)
   except Exception as error:checkpoint['display']=dict(status='FAILED',error=str(error))
   counts={k:sum(r['status']==k for r in checkpoint['items']) for k in ('COMPLETE','FAILED','BLOCKED','PENDING','RUNNING')}
   finished=sum(counts[k] for k in ('COMPLETE','FAILED','BLOCKED'))
   summary=dict(declared=len(ids),finished=finished,counts=counts,all_accounted=finished==len(ids),all_complete=counts['COMPLETE']==len(ids),
       status='FINISHED' if finished==len(ids) else 'INTERRUPTED',elapsed_this_session_seconds=time.monotonic()-started,
       hardware_accessed=any(r.get('hardware_accessed',False) for r in checkpoint['items']),identity=identity,display=checkpoint['display'],
       resources=resources.status() if resources else dict(enabled=False))
   atomic_json(checkpoint_path,checkpoint);atomic_json(output/'batch_summary.json',summary)
   if resources:resources.clear()
 return summary

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--workspace',type=Path,required=True)
 p.add_argument('--config',type=Path,default=REPO/'configs/offline_engineering.json');p.add_argument('--output',type=Path,required=True)
 p.add_argument('--resume',action='store_true');p.add_argument('--retry-failed',action='store_true');p.add_argument('--stop-file',type=Path)
 a=p.parse_args();r=run_batch_session(a.manifest,a.config,a.workspace,a.output,a.resume,a.retry_failed,
     stop_requested=(lambda:a.stop_file.exists()) if a.stop_file else None,on_commit=lambda row:print(json.dumps(dict(id=row['id'],status=row['status'],attempt=row['attempts'])),flush=True))
 print(json.dumps(r,ensure_ascii=False,indent=2));raise SystemExit(0 if r['all_complete'] and r['display']['status']=='PUBLISHED' else 2)
