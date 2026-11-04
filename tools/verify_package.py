"""Read-only manifest verification; does not access hardware or execute reconstruction."""
from pathlib import Path
import hashlib,json,sys
r=Path(__file__).resolve().parents[1];m=json.loads((r/'MANIFEST.json').read_text('utf8'));errors=[]
for item in m['files']:
 p=r/item['path']
 if not p.is_file():errors.append('Missing: '+item['path']);continue
 b=p.read_bytes()
 if b.startswith(b'version https://git-lfs.github.com/spec/v1'):errors.append('LFS content needed: '+item['path']);continue
 if len(b)!=item['bytes'] or hashlib.sha256(b).hexdigest()!=item['sha256']:errors.append('Mismatch: '+item['path'])
print('\n'.join(errors) if errors else 'PASS: '+str(len(m['files']))+' files verified')
sys.exit(bool(errors))
