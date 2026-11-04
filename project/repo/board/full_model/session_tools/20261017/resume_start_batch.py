"""Resume exactly the 45 pending cases and re-enable the verified display."""
import hashlib,json,subprocess,sys,time
from resume_common import ROOT,REMOTE,BOOT,serial
from resume_prepare_results import run

def main():
    saved=json.loads((ROOT/'resume_saved52_ready.json').read_text('utf8'))
    if saved['passed'] is not True or saved['cases']!=52:raise RuntimeError('Saved result audit not finished')
    cp=json.loads((ROOT/'pause_checkpoint.json').read_text('utf8'))
    results=ROOT/'downloads/lunch52_snapshot/content/results/dataset97_pl_01'
    accepted={k:hashlib.sha256((results/(k+'.pfo')).read_bytes()).hexdigest() for k in cp['completed']}
    source=ROOT/'display/resume_accepted_cases_initial.json'
    source.write_text(json.dumps(accepted,sort_keys=True),encoding='utf8')
    if not (ROOT/'resume_upload_acceptance52.log').exists():
        run([ROOT/'resume_common.py','upload_serial_file.py',source,'display/resume_accepted_cases.json'],'resume_upload_acceptance52')
    code="""import json,os,fcntl,struct,subprocess
from pathlib import Path
s=Path('"""+REMOTE+"""');d=s/'display'
if (d/'resume.stop').exists():raise RuntimeError('Resume stop file exists')
if Path('/sys/class/tty/tty0/active').read_text().strip()!='tty1':raise RuntimeError('Unexpected active console')
fd=os.open('/dev/tty1',os.O_RDWR);b=bytearray(4);fcntl.ioctl(fd,0x4b3b,b,True);before=struct.unpack('<I',b)[0]
fcntl.ioctl(fd,0x4b3a,1);fcntl.ioctl(fd,0x4b3b,b,True);after=struct.unpack('<I',b)[0];os.close(fd)
if after!=1:raise RuntimeError('Graphics console mode failed')
with (d/'resume_console_mode_v2.json').open('x') as f:json.dump(dict(before=before,after=after),f)
with (d/'live_resume_v2.log').open('x') as f:p=subprocess.Popen(['/usr/bin/python3.7','-B',str(s/'tools/live_display_resume_v2.py')],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
(d/'resume_viewer_pid').write_text(str(p.pid))
print('PS_TRANSFER_JSON='+json.dumps(dict(pid=p.pid)),flush=True)
"""
    viewer=serial(code,'resume_viewer_start')
    time.sleep(2)
    code="import json,os\nfrom pathlib import Path\nd=Path('"+REMOTE+"')/'display'\nos.kill("+str(viewer['pid'])+",0)\nr=json.loads((d/'live_state.json').read_text())\nprint('PS_TRANSFER_JSON='+json.dumps(r),flush=True)\n"
    shown=serial(code,'resume_viewer_check')
    if shown.get('boot_id')!=BOOT or shown['number']!=52 or shown['independent_acceptance'] is not True:raise RuntimeError('Display did not recover saved result')
    (ROOT/'display/resume_display_verified.json').write_text(json.dumps(shown,indent=2),encoding='utf8')
    name='dataset45_pl_resume_01'
    command=['/usr/bin/python3.7','-B',REMOTE+'/deployment/dataset/scripts/run_dataset.py','--dataset',REMOTE+'/deployment/dataset','--release',REMOTE+'/deployment/performance','--backend','pl','--schedule','overlap','--execute','--output',REMOTE+'/results/'+name]
    for key in cp['remaining']:command+=['--case',key]
    spec=dict(name=name,command=command)
    path=ROOT/(name+'.spec.json');path.write_text(json.dumps(spec,indent=2),encoding='utf8')
    run([ROOT/'resume_common.py','board_control.py','job','--spec',path],'resume_launch45')
    (ROOT/'resume_batch_started.json').write_text(json.dumps(dict(boot_id=BOOT,name=name,completed_before=52,remaining=45,viewer=viewer,display=shown),indent=2),encoding='utf8')
    print('RESUMED_45_WITH_LIVE_DISPLAY',flush=True)

if __name__=='__main__':main()
