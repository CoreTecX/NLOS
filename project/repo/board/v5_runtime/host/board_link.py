"""Boot-bound serial control; preserves commands and replies in this step's directory."""
from pathlib import Path
import base64,hashlib,json,os,re,subprocess,sys,zlib
ROOT=Path(__file__).resolve().parent
SERIAL=ROOT/'serial_command.ps1'
REMOTE='/home/root/ps_runs_20261018/v5_step02_01'
OLD='/home/root/ps_runs_20261017/steps5_12_01'
ENV={k.upper():v for k,v in os.environ.items()}

def serial(code,name,timeout=50,guard=True):
    if guard:
        boot=json.loads((ROOT/'session.json').read_text('utf8'))['boot_id']
        code="from pathlib import Path\nif Path('/proc/sys/kernel/random/boot_id').read_text().strip()!="+repr(boot)+":raise RuntimeError('Boot changed: re-inspect first')\n"+code
    (ROOT/(name+'.source.py')).write_text(code,encoding='utf8')
    compressed="import base64,zlib\nexec(zlib.decompress(base64.b64decode("+repr(base64.b64encode(zlib.compress(code.encode(),9)).decode())+")))\n"
    assert len(base64.b64encode(compressed.encode()))<3800,'Split command or upload by chunks'
    script=ROOT/(name+'.py');script.write_text(compressed,encoding='utf8')
    run=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(SERIAL),'-Script',str(script),'-Log',str(ROOT/(name+'.serial.log')),'-TimeoutSeconds',str(timeout)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=ENV,encoding='utf8',errors='replace',timeout=timeout+15)
    if run.returncode:raise RuntimeError(run.stdout[-2200:])
    matches=re.findall(r'PS_TRANSFER_JSON=(\{[^\r\n]*\})',run.stdout)
    if not matches:raise RuntimeError('Missing response '+run.stdout[-1000:])
    obj=json.loads(matches[-1]);(ROOT/(name+'.json')).write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf8');return obj

def upload(path):
    path=Path(path);data=path.read_bytes();digest=hashlib.sha256(data).hexdigest()
    dest=REMOTE+'/tools/'+path.name
    init="from pathlib import Path\nimport json\np=Path("+repr(dest)+")\np.parent.mkdir(parents=True,exist_ok=True)\nif p.exists() or p.with_suffix(p.suffix+'.part').exists():raise RuntimeError('Upload target exists')\np.with_suffix(p.suffix+'.part').write_bytes(b'')\nprint('PS_TRANSFER_JSON='+json.dumps({'prepared':True}))\n"
    serial(init,'upload_init_'+path.stem)
    packed=zlib.compress(data,9)
    for off in range(0,len(packed),1200):
        chunk=packed[off:off+1200]
        code="from pathlib import Path\nimport base64,json\np=Path("+repr(dest+'.part')+")\nif p.stat().st_size!="+str(off)+":raise RuntimeError('Chunk offset')\nwith p.open('ab') as f:f.write(base64.b64decode("+repr(base64.b64encode(chunk).decode())+"))\nprint('PS_TRANSFER_JSON='+json.dumps({'offset':p.stat().st_size}))\n"
        serial(code,'upload_'+path.stem+'_'+str(off))
    code="from pathlib import Path\nimport zlib,hashlib,json\np=Path("+repr(dest)+")\ndata=zlib.decompress(Path(str(p)+'.part').read_bytes())\nassert hashlib.sha256(data).hexdigest()=="+repr(digest)+"\nwith p.open('xb') as f:f.write(data)\nprint('PS_TRANSFER_JSON='+json.dumps({'path':str(p),'sha256':hashlib.sha256(data).hexdigest()}))\n"
    return serial(code,'upload_finish_'+path.stem)

if __name__=='__main__':
    p=Path(sys.argv[1]);result=serial(p.read_text('utf8'),p.stem+'_result',guard='--no-guard' not in sys.argv)
    print(json.dumps(result,ensure_ascii=False,indent=2))
