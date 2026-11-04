"""Small immutable evidence export over serial, with path and SHA256 verification."""
from pathlib import Path
import argparse,base64,hashlib,io,json,tarfile
from board_link import serial,ROOT,REMOTE
p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--path',action='append',required=True);a=p.parse_args()
assert a.name.replace('_','').isalnum()
dest=ROOT/'downloads'/a.name;dest.mkdir(parents=True,exist_ok=False)
code="""from pathlib import Path
import hashlib,json,tarfile
s=Path(REMOTE_VALUE);files={}
for rel in PATHS_VALUE:
 p=s/rel
 assert s in p.resolve().parents and p.exists() and not p.is_symlink()
 for f in ([p] if p.is_file() else p.rglob('*')):
  if f.is_file():
   assert not f.is_symlink();files[f.relative_to(s).as_posix()]=f
out=s/'exports'/NAME_VALUE;out.mkdir(parents=True,exist_ok=False);archive=out/'snapshot.tgz'
with tarfile.open(str(archive),'w:gz',compresslevel=6) as t:
 for rel,f in sorted(files.items()):t.add(str(f),arcname=rel,recursive=False)
data=archive.read_bytes();r={'path':str(archive),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data),'files':{rel:hashlib.sha256(f.read_bytes()).hexdigest() for rel,f in files.items()}}
(out/'manifest.json').write_text(json.dumps(r,indent=2));print('PS_TRANSFER_JSON='+json.dumps(r))
""".replace('REMOTE_VALUE',repr(REMOTE)).replace('PATHS_VALUE',repr(a.path)).replace('NAME_VALUE',repr(a.name))
m=serial(code,'export_'+a.name);(dest/'manifest.json').write_text(json.dumps(m,indent=2),'utf-8');data=bytearray()
for offset in range(0,m['bytes'],24000):
 code="from pathlib import Path\nimport json,base64\np=Path("+repr(m['path'])+")\nwith p.open('rb') as f:\n f.seek("+str(offset)+");data=f.read(24000)\nprint('PS_TRANSFER_JSON='+json.dumps({'data':base64.b64encode(data).decode()}))\n"
 part=serial(code,'download_'+a.name+'_'+str(offset),timeout=15);data.extend(base64.b64decode(part['data'],validate=True))
 if offset%96000==0:print('Received',len(data),'/',m['bytes'],flush=True)
assert len(data)==m['bytes'] and hashlib.sha256(data).hexdigest()==m['sha256']
(dest/'snapshot.tgz').write_bytes(data)
with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as t:
 for item in t.getmembers():
  p=dest/'content'/item.name
  assert item.isfile() and p.resolve().is_relative_to((dest/'content').resolve())
  b=t.extractfile(item).read();assert hashlib.sha256(b).hexdigest()==m['files'][item.name];p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
(dest/'verified.json').write_text(json.dumps({'passed':True,'files':len(m['files']),'archive_sha256':m['sha256']},indent=2),'utf-8')
print('VERIFIED',str(dest),flush=True)
