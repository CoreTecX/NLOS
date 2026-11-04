"""Upload one small exact file to this session, without using algorithm DDR."""
from pathlib import Path
import argparse,base64,hashlib,json
from jtag_transfer import serial,ROOT,REMOTE
p=argparse.ArgumentParser();p.add_argument('file',type=Path);p.add_argument('relative');a=p.parse_args()
rel=Path(a.relative)
if rel.is_absolute() or '..' in rel.parts or ':' in a.relative:raise ValueError('Invalid destination')
data=a.file.read_bytes();base="import json,hashlib,base64\nfrom pathlib import Path\np=Path('"+REMOTE+"')/"+repr(a.relative)+"\np.parent.mkdir(parents=True,exist_ok=True)\n"
for off in range(0,len(data),1350):
    block=data[off:off+1350]
    code=base+"b=base64.b64decode("+repr(base64.b64encode(block).decode())+")\nwith p.open('xb' if "+str(off)+"==0 else 'ab') as f:\n if f.tell()!="+str(off)+":raise RuntimeError('Upload length mismatch')\n f.write(b)\nprint('PS_TRANSFER_JSON='+json.dumps(dict(bytes=p.stat().st_size)),flush=True)\n"
    result=serial(code,'file_upload_'+a.file.stem+'_'+str(off))
    if result['bytes']!=off+len(block):raise RuntimeError('Upload acknowledgement mismatch')
code=base+"print('PS_TRANSFER_JSON='+json.dumps(dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())),flush=True)\n"
r=serial(code,'file_upload_verify_'+a.file.stem)
if r['sha256']!=hashlib.sha256(data).hexdigest():raise RuntimeError('Upload hash mismatch')
print('SERIAL_FILE_VERIFIED',a.relative,r,flush=True)
