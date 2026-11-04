from pathlib import Path
import json,os,subprocess,time,uuid
ROOT=Path('/home/root/ps_paper_live_20261103')
def start():
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    pointer=ROOT/'current.txt'
    if pointer.exists():
        old=Path(pointer.read_text().strip())
        if (old/'state.json').exists():
            s=json.loads((old/'state.json').read_text())
            if s.get('boot_id')==boot and s['state']=='LIVE' and 0<=time.monotonic()-s['last_state_monotonic']<5:return old
    for root in ('/home/root/ps_sc03_20261101','/home/root/ps_dualview_20261030'):
        pp=Path(root)/'current.txt'
        if not pp.exists():continue
        old=Path(pp.read_text().strip())
        if not (old/'state.json').exists():continue
        s=json.loads((old/'state.json').read_text())
        if s.get('boot_id')!=boot or s['state']!='LIVE' or not 0<=time.monotonic()-s['last_state_monotonic']<6:continue
        for f in ('capture_state.json','command_state.json'):
            if (old/f).exists() and json.loads((old/f).read_text()).get('state') in ('CAPTURING','RUNNING'):raise RuntimeError('Previous acquisition busy; wait')
        (old/'stop').touch()
        for _ in range(150):
            s=json.loads((old/'state.json').read_text())
            if s['state'] in ('STOPPED','ERROR'):break
            time.sleep(.2)
        if s['state']!='STOPPED' or not s.get('transport_restored'):raise RuntimeError('Previous camera cleanup incomplete')
    out=Path('/var/volatile')/('ps_paper_live_'+uuid.uuid4().hex)
    with Path(str(out)+'.log').open('w') as log:
        subprocess.Popen(['python3','-B',str(ROOT/'runtime.py'),'--output',str(out)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    for _ in range(160):
        if (out/'state.json').exists():
            s=json.loads((out/'state.json').read_text())
            if s['state']=='ERROR':raise RuntimeError(s.get('error',s))
            if s['state']=='LIVE' and s.get('displayed_frames',0)>1:return out
        time.sleep(.25)
    raise RuntimeError('New preview startup timed out')
if __name__=='__main__':print(str(start()))
