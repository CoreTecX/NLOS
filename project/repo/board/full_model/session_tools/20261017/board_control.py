"""Serial control after bulk transfer. All remote work uses explicit script/job paths."""
from pathlib import Path
import argparse,base64,hashlib,json,zlib
from jtag_transfer import serial,REMOTE,ROOT

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['upload','start','poll','job']);p.add_argument('--file',type=Path);p.add_argument('--name');p.add_argument('--spec',type=Path);a=p.parse_args()
    base="import json,os,subprocess,time\nfrom pathlib import Path\ns=Path('"+REMOTE+"')\n"
    if a.action=='upload':
        data=a.file.read_bytes();name=a.file.name
        if not name.replace('_','').replace('.','').isalnum():raise ValueError('Invalid tool filename')
        compressed=base64.b64encode(zlib.compress(data,9)).decode()
        code=base+"import base64,zlib,hashlib\np=s/'tools'/"+repr(name)+"\np.parent.mkdir(exist_ok=True)\ndata=zlib.decompress(base64.b64decode("+repr(compressed)+"))\nif p.exists() and p.read_bytes()!=data:raise RuntimeError('Different tool exists')\np.write_bytes(data)\nprint('PS_TRANSFER_JSON='+json.dumps(dict(file=str(p),sha256=hashlib.sha256(data).hexdigest())),flush=True)\n"
        # The console wrapper base64 adds overhead; use a second compact source envelope.
        if len(code.encode())>2500:
            code="import base64,zlib\nexec(zlib.decompress(base64.b64decode("+repr(base64.b64encode(zlib.compress(code.encode(),9)).decode())+")))\n"
        result=serial(code,'upload_'+a.file.stem)
        if result['sha256']!=hashlib.sha256(data).hexdigest():raise RuntimeError('Tool upload hash mismatch')
    elif a.action=='start':
        if a.name not in ('board_unpack',):raise ValueError('Unsupported standalone job')
        code=base+"p=s/'tools'/"+repr(a.name+'.py')+"\nwith (s/"+repr(a.name+'.log')+").open('x') as log:r=subprocess.Popen(['/usr/bin/python3.7','-B',str(p)],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)\nprint('PS_TRANSFER_JSON='+json.dumps(dict(pid=r.pid,started=True)),flush=True)\n"
        result=serial(code,'start_'+a.name)
    elif a.action=='poll':
        pattern=(a.name+'*') if a.name else '*'
        code=base+"r={}\nfor p in [s/'deployment_complete.json',s/'deployment_failed.json']+list((s/'jobs').glob("+repr(pattern+'.status.json')+")):\n if p.exists():r[p.name]=json.loads(p.read_text())\nfor p in [s/'board_unpack.log']+list((s/'jobs').glob("+repr(pattern+'.log')+")):\n if p.exists():r[p.name]=p.read_text(errors='replace')[-1000:]\nfor p in (s/'results').glob("+repr(pattern+'/target_suite.json')+"):\n r[str(p.relative_to(s))]=json.loads(p.read_text())\nr['transfer_lease_released']=(s/'transfer_lease_released').exists()\nprint('PS_TRANSFER_JSON='+json.dumps(r),flush=True)\n"
        result=serial(code,'poll_current')
    else:
        spec=json.loads(a.spec.read_text('utf8'));name=spec['name']
        if not name.replace('_','').isalnum():raise ValueError('Invalid job name')
        code=base+"spec=json.loads("+repr(json.dumps(spec))+ ")\np=s/'jobs'/"+repr(name+'.spec.json')+"\np.parent.mkdir(exist_ok=True)\nwith p.open('x') as f:json.dump(spec,f)\nwith (s/'jobs'/"+repr(name+'.wrapper.log')+").open('x') as log:r=subprocess.Popen(['/usr/bin/python3.7','-B',str(s/'tools/board_job.py'),str(p)],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)\nprint('PS_TRANSFER_JSON='+json.dumps(dict(pid=r.pid,job="+repr(name)+")),flush=True)\n"
        result=serial(code,'start_job_'+name)
    print(json.dumps(result,indent=2,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
