"""Prepare complete pinned packages; content hardlinks reduce transfer size only."""
import hashlib,json,os,tarfile,time
from pathlib import Path
task=Path(__file__).resolve().parent
ws=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区')
base=ws/'03_任务产出/P7_P11_Final_20261017/release'
dataset=ws/'03_任务产出/Board_Steps_1_4_20261017/dataset'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
rows=[]
for name,root in [('performance',base/'performance'),('fallback',base/'fallback'),('dataset',dataset)]:
 if name=='dataset':items=json.loads((root/'package_manifest.json').read_text('utf8'))['files'];manifest='package_manifest.json'
 else:
  manifest='release_manifest.json';items=[dict(file=k,**v) for k,v in json.loads((root/manifest).read_text('utf8'))['files'].items()]
 items.append(dict(file=manifest,sha256=sha(root/manifest),bytes=(root/manifest).stat().st_size))
 for r in items:
  p=root/r['file']
  if p.stat().st_size!=r['bytes'] or sha(p)!=r['sha256']:raise ValueError('Source package mismatch: '+str(p))
  rows.append(dict(name=name+'/'+r['file'],source=str(p),sha256=r['sha256'],bytes=r['bytes']))
archive=task/'psight_20261017_complete.tar.gz';seen={};tic=time.monotonic()
with tarfile.open(archive,'w:gz',compresslevel=1,format=tarfile.PAX_FORMAT) as tar:
 for r in rows:
  path=Path(r['source']);info=tar.gettarinfo(str(path),arcname=r['name']);info.uid=info.gid=0;info.uname=info.gname='root'
  info.mode=0o755 if '/bin/' in r['name'] and path.suffix not in ('.bit','.xsa','.exe') else 0o644
  if r['sha256'] in seen:
   info.type=tarfile.LNKTYPE;info.linkname=seen[r['sha256']];info.size=0;tar.addfile(info)
  else:
   seen[r['sha256']]=r['name']
   with path.open('rb') as f:tar.addfile(info,f)
result=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),files=len(rows),unique_file_contents=len(seen),expanded_bytes=sum(r['bytes'] for r in rows),seconds=time.monotonic()-tic,files_manifest=rows)
(task/'transfer_manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k!='files_manifest'},ensure_ascii=False),flush=True)
