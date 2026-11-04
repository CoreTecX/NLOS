"""Windows SC03 controls. Uses PowerShell/.NET serial, not pyserial; no solver."""
from pathlib import Path
import argparse,base64,datetime,hashlib,io,json,re,subprocess,sys,tarfile,uuid

REMOTE = r'''
from pathlib import Path
import base64,hashlib,json,os,subprocess,time,uuid
cfg=CONFIG
root=Path('/home/root/ps_sc03_20261101');code=root/'code_v1'
if cfg['action']=='start':
    if (root/'current.txt').exists():
        current=Path((root/'current.txt').read_text().strip());s=json.loads((current/'state.json').read_text())
        if s['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip() and s['state']=='LIVE' and 0<=time.monotonic()-s['last_state_monotonic']<5:
            raise RuntimeError('SC03 is already live')
    current=root/('session_'+uuid.uuid4().hex)
    with Path(str(current)+'.log').open('w') as log:
        subprocess.Popen(['python3','-B',str(code/'sc03_runtime.py'),'--output',str(current)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    for i in range(40):
        time.sleep(.5)
        if (current/'state.json').exists():
            s=json.loads((current/'state.json').read_text())
            if s['state']=='ERROR':raise RuntimeError(s)
            if s.get('displayed_frames',0)>0:break
    else:raise RuntimeError('Preview startup deadline exceeded; inspect session state')
    result=s
else:
    current=Path((root/'current.txt').read_text().strip())
    if cfg['action']=='capture':
        req=json.loads(subprocess.check_output(['python3',str(code/'capture_cli.py'),'capture','--frames',str(cfg['frames']),'--pc-time',cfg['pc_time']],universal_newlines=True))
        f=current/'jobs'/req['job_id']/'capture.json'
        for i in range(240):
            time.sleep(.5)
            if f.exists():
                result=json.loads(f.read_text())
                if result['state']=='ERROR':raise RuntimeError(result)
                if result['state']=='COMPLETE':break
        else:raise RuntimeError('Capture pending; do not resubmit; use status')
    elif cfg['action']=='stop':
        subprocess.check_output(['python3',str(code/'capture_cli.py'),'stop'])
        for i in range(60):
            time.sleep(.5);result=json.loads((current/'state.json').read_text())
            if result['state'] in ('STOPPED','ERROR'):break
        if result['state']!='STOPPED' or not result.get('transport_restored'):raise RuntimeError(result)
    elif cfg['action']=='status':
        result=json.loads(subprocess.check_output(['python3',str(code/'capture_cli.py'),'status'],universal_newlines=True))
    else:
        out=root/('export_'+uuid.uuid4().hex+'.tar.gz')
        cmd=['python3',str(code/'capture_cli.py'),'export','--output',str(out)]
        if cfg['job']:cmd+=['--job',cfg['job']]
        if cfg['action']=='export-raw':cmd+=['--include-raw']
        result=json.loads(subprocess.check_output(cmd,universal_newlines=True))
        if cfg['action']=='export-summary':
            if result['bytes']>4*1024*1024:raise RuntimeError('Summary exceeds serial limit; select one --job')
            b=out.read_bytes();print('',flush=True)
            for i in range(0,len(b),384):print('SC03_DATA',i,base64.b64encode(b[i:i+384]).decode(),flush=True)
print('\nSC03_RESULT',json.dumps(result,separators=(',',':')),flush=True)
'''

def verify_bundle(data):
    with tarfile.open(fileobj=io.BytesIO(data),mode='r:gz') as tf:
        m=json.load(tf.extractfile('bundle_manifest.json'))
        for row in m['files']:
            content=tf.extractfile(row['name']).read()
            if len(content)!=row['bytes'] or hashlib.sha256(content).hexdigest()!=row['sha256']:
                raise ValueError('Bundle member integrity failure: '+row['name'])
        return m

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('action',choices=['start','status','capture','stop','export-summary','export-raw'])
    ap.add_argument('--frames',type=int,default=3);ap.add_argument('--port',default='COM14');ap.add_argument('--job')
    ap.add_argument('--output-dir',type=Path,default=Path(r'D:\Temp\SC03_Control'))
    args=ap.parse_args()
    if not 1<=args.frames<=64:ap.error('--frames must be 1..64')
    if args.job and not re.fullmatch('[a-f0-9]{32}',args.job):ap.error('--job must be a 32 character job ID')
    now=datetime.datetime.now(datetime.timezone.utc).isoformat()
    folder=args.output_dir/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]);folder.mkdir(parents=True,exist_ok=False)
    cfg=dict(action=args.action,frames=args.frames,job=args.job,pc_time=now)
    script=REMOTE.replace('CONFIG',repr(cfg));(folder/'request.py').write_text(script,encoding='utf-8')
    log=folder/'serial.log'
    process=subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(Path(__file__).with_name('serial_console.ps1')),
                            '-Script',str(folder/'request.py'),'-Log',str(log),'-PortName',args.port,'-TimeoutSeconds','700'],
                           stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    if process.returncode:raise RuntimeError('Serial operation failed. Inspect '+str(log)+'; '+process.stderr.decode(errors='replace'))
    text=log.read_text(encoding='utf-8')
    matches=re.findall(r'^SC03_RESULT (\{.*\})\r?$',text,re.M)
    if len(matches)!=1:raise RuntimeError('Missing/unreliable result marker: '+str(log))
    result=json.loads(matches[0])
    if args.action=='export-summary':
        chunks=re.findall(r'^SC03_DATA (\d+) ([A-Za-z0-9+/=]+)\r?$',text,re.M)
        data=bytearray()
        for offset,chunk in chunks:
            if int(offset)!=len(data):raise ValueError('Missing/corrupt serial chunk; board archive retained')
            data.extend(base64.b64decode(chunk,validate=True))
        if len(data)!=result['bytes'] or hashlib.sha256(data).hexdigest()!=result['sha256']:raise ValueError('Archive checksum mismatch; board archive retained')
        m=verify_bundle(data);(folder/'summary.tar.gz').write_bytes(data)
        result.update(pc_copy=str(folder/'summary.tar.gz'),pc_sha256_verified=True,member_hashes_verified=True,raw_included=m['raw_included'])
    result['pc_operation_time_utc']=now
    (folder/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2));print('PC_RECORD',str(folder))

if __name__=='__main__':main()
