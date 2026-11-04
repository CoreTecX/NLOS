"""Read-only naming, module, data and manifest contract checks. No hardware I/O."""
from pathlib import Path
import json,hashlib,re,sys
r=Path(__file__).resolve().parents[1];errors=[];cache={}
def sha(p):
 k=str(p.resolve())
 if k not in cache:cache[k]=hashlib.sha256(p.read_bytes()).hexdigest()
 return cache[k]
for p in r.rglob('*'):
 if '.git' in p.parts:continue
 if p.name!='README.md' and not re.fullmatch(r'[a-z0-9_.-]+',p.name):errors.append('NAME '+str(p.relative_to(r)))
for paths in json.loads((r/'modules.json').read_text('utf8')).values():
 for p in paths:
  if not (r/p).exists():errors.append('MODULE '+p)
index=json.loads((r/'project/data/batch_manifest.json').read_text('utf8'))
def verify(base,rec):
 p=(base/rec.get('path',rec.get('file',''))).resolve()
 if not p.is_file() or not p.is_relative_to(r.resolve()):errors.append('MISSING '+str(p));return
 if p.stat().st_size!=rec['bytes'] or sha(p)!=rec['sha256']:errors.append('HASH '+str(p))
for row in index['items']:
 verify(r/'project/data',row['input']);p=r/'project/data'/row['input']['path'];v=json.loads(p.read_text('utf8'))
 for role in ('file','model','golden'):verify(p.parent,v[role])
release=r/'project/releases/performance';mf=release/'release_manifest.json';m=json.loads(mf.read_text('utf8'))
for name,rec in m['files'].items():verify(release,dict(rec,path=name))
for f in (r/'project/configs').glob('*.json'):
 c=json.loads(f.read_text('utf8'))
 if c.get('release_manifest_sha256') and c['release_manifest_sha256']!=sha(mf):errors.append('RELEASE PIN '+f.name)
 for rec in c.get('host_libraries',[]):verify(r/'project',rec)
for d in ('src','sim','build','board','skill','report'):
 if not (r/d/'README.md').exists() and d!='report':errors.append('DIRECTORY '+d)
print('\n'.join(errors) if errors else 'PASS: naming, modules, '+str(len(index['items']))+' cases and release/config pins')
sys.exit(bool(errors))
