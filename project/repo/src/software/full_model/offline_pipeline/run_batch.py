"""Ordered bounded full-algorithm batch runner; every input receives a ledger row."""
from pathlib import Path
from types import SimpleNamespace
import argparse,json,time
from run import execute
from contracts import read_json,write_json,pinned,require,sha,InputError
from native_backend import within
from resources import VerifiedResources


def run_batch(manifest,config,workspace,output,timeout=1800):
    manifest=Path(manifest);output=Path(output).resolve();workspace=Path(workspace).resolve()
    raw=manifest.read_bytes();spec=json.loads(raw)
    require(spec['schema_version']==1 and isinstance(spec['items'],list) and 0<len(spec['items'])<=100000,
            'BATCH_MANIFEST','Bounded nonempty ordered item list required')
    ids=[row['id'] for row in spec['items']]
    require(all(isinstance(x,str) and x for x in ids) and len(ids)==len(set(ids)), 'BATCH_ID','Distinct item identities required')
    require(not within(output,workspace) or within(output,workspace/'99_临时工作区'),'OUTPUT_LOCATION','Batch output must use workspace temporary area')
    output.mkdir(parents=True,exist_ok=False)
    cfg=read_json(config);resources=VerifiedResources(cfg.get('resource_cache_bytes',64*1024*1024)) if cfg.get('resource_cache_enabled',False) else None
    counts=dict(COMPLETE=0,FAILED=0,BLOCKED=0);finished=0;started=time.monotonic_ns();hardware_accessed=False
    try:
        with (output/'ledger.jsonl').open('x',encoding='utf8') as ledger:
            for index,item in enumerate(spec['items']):
                row=dict(index=index,id=item['id'],descriptor=item['input'],status='STARTED')
                ledger.write(json.dumps(row,ensure_ascii=False)+'\n');ledger.flush()
                folder=output/('item_%06d'%index);folder.mkdir()
                try:
                    descriptor=json.loads(pinned(manifest.parent,item['input']))
                    # Keep original base for relative pinned measurement/model/calibration paths.
                    source=(manifest.parent/item['input']['path']).resolve()
                    result=execute(SimpleNamespace(config=config,workspace=workspace,output=folder/'result',case=None,
                        raw_sample=None,input=source,action='reconstruct',runtime_root=None,timeout=timeout),resources=resources,
                        input_snapshot=(descriptor,source.parent))
                    hardware_accessed=hardware_accessed or result.get('hardware_accessed',False)
                    status=result['status'];require(status in counts,'BATCH_RESULT','Unexpected pipeline result')
                    row.update(status=status,error_code=result.get('error_code'),output=('item_%06d/result'%index))
                except (InputError,OSError,ValueError,KeyError,TypeError) as error:
                    row.update(status='FAILED',error_code=getattr(error,'code',type(error).__name__),error=str(error))
                write_json(folder/'receipt.json',row);counts[row['status']]+=1;finished+=1
                ledger.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');ledger.flush()
    finally:
        summary=dict(declared=len(ids),finished=finished,counts=counts,all_accounted=finished==len(ids),
            all_complete=counts['COMPLETE']==len(ids),status='FINISHED' if finished==len(ids) else 'INTERRUPTED',
            input_manifest_sha256=sha(raw),ordering='MANIFEST_ORDER_NO_DROP',hardware_accessed=hardware_accessed,
            elapsed_ms=(time.monotonic_ns()-started)/1e6,resources=resources.status() if resources else dict(enabled=False))
        write_json(output/'batch_summary.json',summary)
        if resources:resources.clear()
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[4]/'configs/offline_engineering.json')
    p.add_argument('--workspace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=run_batch(a.manifest,a.config,a.workspace,a.output);print(json.dumps(r,ensure_ascii=False,indent=2))
    raise SystemExit(0 if r['all_complete'] else 2)
