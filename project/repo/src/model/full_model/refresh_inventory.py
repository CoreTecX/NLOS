"""Read-only current input census; reuse pinned data bindings, never guess models."""
from pathlib import Path
import argparse,collections,csv,hashlib,json,os

EXTENSIONS={'.mat','.npz','.npy','.psf','.pfy','.pfm','.pfg','.pfo','.pfs','.raw','.png','.jpg','.jpeg','.tif','.tiff','.bmp'}
IMAGES={'.png','.jpg','.jpeg','.tif','.tiff','.bmp'}
def digest(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def read(path):return json.loads(Path(path).read_text('utf-8-sig'))
def dump(path,value):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n','utf8')
def csv_dump(path,rows,fields):
 with Path(path).open('w',encoding='utf-8-sig',newline='') as stream:
  w=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');w.writeheader()
  for row in rows:w.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict)) else v for k,v in row.items()})
def record(path):return dict(path=str(path),bytes=path.stat().st_size,sha256=digest(path))

def reviewed_outputs(workspace):
 known={};evidence=[]
 exports=workspace/'03_任务产出/Board_Steps_13_14_20261017';manifest=read(exports/'package_manifest.json')
 for rel,expected in manifest['files'].items():
  if rel.startswith('04_浮点结果/'):
   if digest(exports/rel)!=expected:raise ValueError('Changed historical numerical export: '+rel)
   known[expected]=dict(disposition='VERIFIED_HISTORICAL_RECONSTRUCTION_EXPORT',evidence=str(exports/'package_manifest.json'))
 evidence.append(record(exports/'package_manifest.json'))
 prior=workspace/'02_代码仓库/penumbrasight/report/o01_o03_offline_engineering_20261019'
 for rel in ('runs/d11_rgb','verification/cli/pfy_prepare'):
  folder=prior/rel;summary=read(folder/'summary.json');desc=read(folder/'input_descriptor.json')
  for name,expected in summary['output_files'].items():
   if name.endswith(('.npy','.pfy')):
    if digest(folder/name)!=expected:raise ValueError('Changed historical output: '+name)
    role='VERIFIED_SYNTHETIC_PREPROCESSING_FIXTURE' if desc['origin']=='SYNTHETIC' else 'VERIFIED_HISTORICAL_DERIVED_ARRAY'
    known[expected]=dict(disposition=role,evidence=str(folder/'summary.json'))
  evidence.extend([record(folder/'summary.json'),record(folder/'input_descriptor.json')])
 return known,evidence

def census(workspace,project,output):
 workspace=Path(workspace).resolve();project=Path(project).resolve();output=Path(output).resolve()
 if output.exists():raise ValueError('Choose a new census output')
 output.mkdir(parents=True)
 prior=workspace/'03_任务产出/Board_Steps_1_4_20261017';package=prior/'dataset'
 manifest=read(package/'dataset_manifest.json');seal=read(package/'package_manifest.json')
 for r in seal['files']:
  path=(package/r['file']).resolve()
  if package not in path.parents or path.stat().st_size!=r['bytes'] or digest(path)!=r['sha256']:raise ValueError('Changed sealed dataset: '+r['file'])
 old_assets=read(prior/'inventory/assets.json');old_dispositions=read(prior/'inventory/dispositions.json')
 old_by_path={r['path']:r for r in old_assets};old_by_hash={}
 for r in old_dispositions:old_by_hash.setdefault(r['sha256'],r)
 paper_rows=read(prior/'adaptation/paper_cases.json');paper_root=project/'论文数据集/Ordinary-Camera-master'
 for row in paper_rows:
  if digest(paper_root/'Data'/row['source'])!=row['sha256']:raise ValueError('Paper source drift: '+row['source'])
 paper_by_hash={r['sha256']:r for r in paper_rows};cases_by_hash=collections.defaultdict(list)
 descriptors=[];ready=[];blocked=[]
 for i,row in enumerate(manifest['cases']):
  cases_by_hash[row['measurement']['sha256']].append(row['key'])
  def pin(role):
   r=row[role];return dict(path=str(package/r['file']),bytes=r['bytes'],sha256=r['sha256'])
  item=dict(id=row['key'],source=row['source'],input_kind=row['input_kind'],model_id=row['model_id'],
            measurement_sha256=row['measurement']['sha256'],model_sha256=row['model']['sha256'],paper_figures=row.get('paper_figures',[]),
            reference_status=row['reference_status'],output_relative='batch/item_%06d/result'%len(ready))
  if row['reference_status'] not in ('frozen_independent_reference','independent_reference_passed'):
   # Accept only the actual archived success labels; inspect unknown labels explicitly.
   if row['reference_status']!='independent_reference_converged':
    item.update(status='BLOCKED_REFERENCE',reason='Original complete reference did not meet the frozen acceptance rule');blocked.append(item);continue
  desc=dict(schema_version=1,kind='PFY',sample_id=row['key'],complete=True,origin='HISTORICAL_MATCHED_MEASUREMENT',
            file=pin('measurement'),model=pin('model'),model_id=row['model_id'],golden=pin('golden'),capture={})
  p=output/'descriptors'/(row['key']+'.json');dump(p,desc)
  item.update(status='READY',descriptor='descriptors/'+p.name);ready.append(item)
  descriptors.append(dict(id=row['key'],input=dict(path='descriptors/'+p.name,bytes=p.stat().st_size,sha256=digest(p))))
 for row in paper_rows:
  if row['status']=='BLOCKED':blocked.append(dict(row,id='paper:'+row['source'],status='BLOCKED_MODEL_CALIBRATION'))
 # Real camera captures remain distinct from the paper RAW and synthetic fixtures.
 raw_catalog=read(workspace/'03_任务产出/L02_Offline_Inputs_20261019/replay_samples.json')
 real_raw=[]
 for row in raw_catalog['samples']:
  r=row['raw'];path=workspace/r['workspace_path']
  if digest(path)!=r['sha256'] or path.stat().st_size!=r['bytes']:raise ValueError('Real RAW drift')
  real_raw.append(dict(id=row['sample_id'],file=record(path),status='BLOCKED_REAL_CALIBRATION',reason='No registered real geometry/radiometry/model'))
 r12=workspace/'03_任务产出/R12_First_Real_RAW_20261020/data';r=read(r12/'sample_manifest.json')
 if digest(r12/'first_real_frame.raw')!=r['raw_sha256']:raise ValueError('R12 RAW drift')
 real_raw.append(dict(id=r['sample_id'],file=record(r12/'first_real_frame.raw'),status='BLOCKED_REAL_CALIBRATION',reason='R14 real calibration incomplete'))
 raw_hashes={r['file']['sha256']:r['id'] for r in real_raw};blocked.extend(real_raw)
 derived,derived_evidence=reviewed_outputs(workspace)
 scopes=[('paper',paper_root),('workspace',workspace),('competition',project/'NLOS赛题')]
 assets=[];errors=[];by_hash={};skipped=[];unknown=[]
 for scope,root in scopes:
  if not root.is_dir():raise ValueError('Missing census root '+str(root))
  for directory,dirs,files in os.walk(str(root),followlinks=False,onerror=lambda e:errors.append(str(e))):
   kept=[]
   for name in sorted(dirs):
    p=Path(directory)/name
    if name in ('.git','.Xil','__pycache__','node_modules') or p.is_symlink() or p.resolve()==output:
     skipped.append(str(p));continue
    kept.append(name)
   dirs[:]=kept
   for name in sorted(files):
    path=Path(directory)/name;ext=path.suffix.lower()
    if ext not in EXTENSIONS:continue
    try:
     before=path.stat();sha=digest(path);after=path.stat()
     if before.st_size!=after.st_size or before.st_mtime_ns!=after.st_mtime_ns:raise ValueError('File changed during scan')
     relative=path.relative_to(root).as_posix();old=old_by_hash.get(sha)
     row=dict(scope=scope,path=str(path),relative=relative,bytes=after.st_size,sha256=sha,duplicate_of=by_hash.get(sha),extension=ext)
     by_hash.setdefault(sha,str(path));row['prior_same_path_unchanged']=str(path) in old_by_path and old_by_path[str(path)]['sha256']==sha
     if sha in cases_by_hash:row.update(disposition='REGISTERED_MEASUREMENT_COPY',cases=cases_by_hash[sha])
     elif sha in paper_by_hash:row.update(disposition='PAPER_'+paper_by_hash[sha]['status'],paper_source=paper_by_hash[sha]['source'])
     elif sha in raw_hashes:row.update(disposition='REGISTERED_REAL_RAW_COPY',sample_id=raw_hashes[sha])
     elif sha in derived:row.update(derived[sha])
     elif old:row.update(disposition='PRIOR_REVIEWED_CONTENT',prior_disposition=old['disposition'],prior_path=old['path'])
     elif ext in ('.pfo','.pfs','.pfg','.pfm'):row['disposition']='MODEL_REFERENCE_OR_SOLVER_OUTPUT'
     elif ext in IMAGES:
      generated=scope=='workspace' and any(part in relative.lower() for part in ('03_任务产出/','sim/results/','report/','99_临时工作区/','tests/','docs/'))
      row['disposition']='DISPLAY_OR_DOCUMENT_ASSET' if generated or scope=='paper' else 'RASTER_PROVENANCE_REQUIRED'
     elif scope=='workspace' and any(part in relative.lower() for part in ('nb08_nb10','nb11_nb12','o04_o07','tests/','acceptance','test_','fixture','synthetic','reproduce/','before_publication/')):
      row['disposition']='DECLARED_TEST_OR_DERIVED_ARTIFACT'
     else:row['disposition']='UNREGISTERED_NUMERIC_OR_RAW_PROVENANCE_REQUIRED'
     if row['disposition'].endswith('PROVENANCE_REQUIRED'):unknown.append(row)
     assets.append(row)
     if len(assets)%1000==0:print('Scanned',len(assets),'files',flush=True)
    except (OSError,ValueError) as error:errors.append(dict(path=str(path),error=str(error)))
 summary=dict(scope_roots=[dict(scope=k,path=str(p)) for k,p in scopes],total_files=len(assets),unique_contents=len(by_hash),
              counts=dict(collections.Counter(r['disposition'] for r in assets)),scan_errors=errors,skipped_directories=skipped,
              ready_cases=len(ready),blocked_paper=sum(r['status']=='BLOCKED' for r in paper_rows),
              blocked_reference=sum(r['status']=='BLOCKED_REFERENCE' for r in blocked),blocked_real_raw=len(real_raw),
              unresolved_paths=len(unknown),unresolved_unique_contents=len(set(r['sha256'] for r in unknown)),
              sealed_dataset_verified_files=len(seal['files']),hardware_accessed=False,
              exclusions='Top-level compressed archives and dated snapshots outside declared roots; embedded document illustrations are not independent sensor measurements.')
 dump(output/'assets.json',assets);csv_dump(output/'assets.csv',assets,['scope','relative','path','bytes','sha256','duplicate_of','disposition','prior_disposition','cases'])
 dump(output/'ready_cases.json',ready);dump(output/'blocked.json',blocked);dump(output/'paper_cases.json',paper_rows);dump(output/'unresolved.json',unknown)
 csv_dump(output/'coverage.csv',ready+blocked,['id','status','source','reason','input_kind','paper_figures','model_id','reference_status','descriptor','output_relative'])
 dump(output/'batch_manifest.json',dict(schema_version=1,items=descriptors));dump(output/'summary.json',summary)
 dump(output/'derived_provenance.json',dict(content_hashes=derived,evidence=derived_evidence))
 dump(output/'source_pins.json',dict(dataset_manifest=record(package/'dataset_manifest.json'),package_manifest=record(package/'package_manifest.json'),
     prior_assets=record(prior/'inventory/assets.json'),prior_dispositions=record(prior/'inventory/dispositions.json')))
 print(json.dumps(summary,ensure_ascii=False),flush=True)
 if errors:raise RuntimeError('Census incomplete: scan errors')
 return summary

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workspace',type=Path,required=True);p.add_argument('--project',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 a=p.parse_args();census(a.workspace,a.project,a.output)
