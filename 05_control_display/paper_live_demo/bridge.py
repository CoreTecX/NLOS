"""Live preview on board, fresh paper-measurement reconstruction by frozen PC solver."""
from pathlib import Path
import argparse,base64,datetime,hashlib,json,os,re,subprocess,sys,uuid,time,tempfile
ROOT=Path(__file__).resolve().parent
CANDIDATE=ROOT.parents[1]/'project'
REMOTE_ROOT='/home/root/ps_paper_live_20261103'

def remote(action,folder,port='COM14',payload=None):
    if not re.fullmatch(r'COM\d+',port,re.I):raise ValueError('Invalid COM port')
    script="""from pathlib import Path
import json,sys,time,os,base64,hashlib
root=Path(%r)
sys.path.insert(0,str(root))
action=%r;data=%r
if action=='start':
 from start_board import start
 start()
s=Path((root/'current.txt').read_text().strip());camera=json.loads((s/'state.json').read_text())
boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assert camera['boot_id']==boot and camera['state']=='LIVE' and 0<=time.monotonic()-camera['last_state_monotonic']<6,'Camera preview not live'
os.kill(camera['pid'],0)
state=json.loads((s/'paper_state.json').read_text()) if (s/'paper_state.json').exists() else dict(state='WAITING')
def save(v):
 t=s/'paper_state.tmp';t.write_text(json.dumps(v,indent=2));t.replace(s/'paper_state.json')
if action=='start':
 assert state['state']!='RUNNING','BUSY'
 state=dict(state='WAITING',boot_id=boot,job_id=None,source='PAPER_ORIGINAL_MEASUREMENT',camera_used_as_input=False);save(state)
elif action=='begin':
 assert state['state']!='RUNNING','BUSY'
 assert len(data['job_id'])==32 and all(c in '0123456789abcdef' for c in data['job_id'])
 assert data['case']=='Chair_mushroom'
 (s/'paper_jobs'/data['job_id']).mkdir(parents=True,exist_ok=False)
 state=dict(data,state='RUNNING',boot_id=boot,requested_monotonic=time.monotonic(),source='PAPER_ORIGINAL_MEASUREMENT',backend='PC_CPU_FULL_FP64',camera_used_as_input=False)
 save(state)
elif action in ('complete','error'):
 assert state.get('job_id')==data['job_id'] and state['boot_id']==boot and state['state']=='RUNNING','Job context changed'
 if action=='complete':
  b=base64.b64decode(data.pop('image_base64'));assert hashlib.sha256(b).hexdigest()==data['image_sha256']
  (s/'paper_jobs'/data['job_id']/'reconstruction.png').write_bytes(b)
 if action=='complete':
  delay=max(0,min(30,data.get('display_not_before_monotonic',0)-time.monotonic()))
  time.sleep(delay)
  data['display_release_monotonic']=time.monotonic()
 state.update(data,state='COMPLETE' if action=='complete' else 'ERROR');save(state)
 (s/'paper_jobs'/data['job_id']/'result.json').write_text(json.dumps(state,indent=2))
 if action=='complete':
  for _ in range(60):
   a=s/'paper_display_ack.json'
   if a.exists():
    ack=json.loads(a.read_text())
    if ack.get('job_id')==state['job_id'] and ack['image_sha256']==state['image_sha256']:break
   time.sleep(.2)
  else:raise RuntimeError('Computed image saved, display acknowledgment pending')
  state['display_ack']=ack
elif action=='reset':
 assert state['state']!='RUNNING','BUSY'
 state=dict(state='WAITING',boot_id=boot,job_id=None,source='PAPER_ORIGINAL_MEASUREMENT',camera_used_as_input=False);save(state)
print('\\nPAPER_CONTROL '+json.dumps(dict(session=str(s),camera=camera,command=state)),flush=True)
"""%(REMOTE_ROOT,action,payload or {})
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    request=folder/(action+'.py');log=folder/(action+'.log');request.write_text(script,encoding='utf8')
    si=subprocess.STARTUPINFO();si.dwFlags|=subprocess.STARTF_USESHOWWINDOW
    proc=subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'serial_console.ps1'),'-PortName',port,'-Script',str(request),'-Log',str(log),'-TimeoutSeconds','100'],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,startupinfo=si,creationflags=subprocess.CREATE_NO_WINDOW)
    text=log.read_text('utf8') if log.exists() else ''
    matches=re.findall(r'^[> ]*PAPER_CONTROL (\{.*\})\r?$',text,re.M)
    if proc.returncode or len(matches)!=1:raise RuntimeError('Board command failed: '+str(log)+'\n'+text[-1200:])
    r=json.loads(matches[0]);(folder/(action+'_result.json')).write_text(json.dumps(r,indent=2),encoding='utf8');return r

def operate(action,port='COM14'):
    job=uuid.uuid4().hex;folder=ROOT/'operations'/job;folder.mkdir(parents=True)
    if action!='reconstruct':return remote(action,folder,port)
    submitted=False;started=time.monotonic()
    try:
        now=datetime.datetime.now(datetime.timezone.utc).isoformat()
        submitted=True;begin=remote('begin',folder,port,dict(job_id=job,case='Chair_mushroom',pc_request_time=now))
        deadline=begin['command']['requested_monotonic']+27-(time.monotonic()-started)
        output=folder/'solver_output'
        command=[sys.executable,'-B',str(CANDIDATE/'repo/src/software/full_model/offline_pipeline/run.py'),
            '--config',str(CANDIDATE/'configs/windows_cpu.json'),'--workspace',str(CANDIDATE),'--case','Chair_mushroom',
            '--output',str(output),'--runtime-root',str(Path(tempfile.gettempdir())/'ps_submission_solver'),'--timeout','1800']
        si=subprocess.STARTUPINFO();si.dwFlags|=subprocess.STARTF_USESHOWWINDOW
        with (folder/'pipeline.log').open('w',encoding='utf8') as log:
            process=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,startupinfo=si,creationflags=subprocess.CREATE_NO_WINDOW,timeout=1900)
        if process.returncode:raise RuntimeError('Full algorithm failed; inspect '+str(folder/'pipeline.log'))
        result=json.loads((output/'summary.json').read_text('utf8'))
        if result['status']!='COMPLETE' or not result['state_acceptance'] or not result['reference_acceptance']:
            raise RuntimeError('New result did not pass frozen reference and independent state audit')
        image=(output/'reconstruction_native.png').read_bytes()
        metadata=dict(display_not_before_monotonic=deadline,presentation_target_seconds=27,job_id=job,image_base64=base64.b64encode(image).decode(),image_sha256=hashlib.sha256(image).hexdigest(),
            seconds_solver=result['seconds_solver'],iterations=result['iterations'],full_dimension=1044,
            native_shape=result['result_shape'],reference_acceptance=True,state_acceptance=True,
            measurement_sha256=result['measurement_sha256'],model_sha256=result['model_sha256'],
            solver_sha256=result['solver_sha256'],paper_case='Chair_mushroom',camera_used_as_input=False,
            pc_output=str(output),backend='PC_CPU_FULL_FP64')
        r=remote('complete',folder,port,metadata)
        r['elapsed_click_to_ack_seconds']=time.monotonic()-started;r['pc_output']=str(output);(folder/'result.json').write_text(json.dumps(r,indent=2),encoding='utf8');return r
    except Exception as e:
        (folder/'error.json').write_text(json.dumps(dict(job_id=job,error=str(e)),indent=2),encoding='utf8')
        if submitted:
            try:remote('error',folder,port,dict(job_id=job,error=str(e)))
            except Exception:pass
        raise
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=['start','status','reset','reconstruct']);a=ap.parse_args()
    print(json.dumps(operate(a.action),ensure_ascii=False,indent=2))
