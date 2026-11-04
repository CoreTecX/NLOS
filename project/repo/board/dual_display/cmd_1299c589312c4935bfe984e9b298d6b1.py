from pathlib import Path
import time,json,subprocess
r=Path('/home/root/ps_dualview_20261030')
f=open(str(r/'startup_ui.log'),'w')
q=subprocess.Popen(['/usr/bin/python3',str(r/'start_board.py')],stdout=f,stderr=subprocess.STDOUT,start_new_session=True);f.close()
for _ in range(360):
 try:
  p=Path((r/'current.txt').read_text().strip());v=json.loads((p/'state.json').read_text())
  if v.get('state')=='LIVE' and v.get('complete_frames',0)>=2 and v.get('displayed_frames',0)>=1 and v.get('boot_id')==Path('/proc/sys/kernel/random/boot_id').read_text().strip() and 0<=time.monotonic()-v.get('last_state_monotonic',0)<5:break
 except (OSError,ValueError):pass
 if q.poll() not in (None,0):raise RuntimeError((r/'startup_ui.log').read_text()[-1200:])
 time.sleep(.25)
else:raise RuntimeError('Camera startup timeout; inspect startup_ui.log')
from pathlib import Path
import json,time,os,uuid
r=Path('/home/root/ps_dualview_20261030');p=Path((r/'current.txt').read_text().strip());s=json.loads((p/'state.json').read_text());boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assert s['boot_id']==boot and s['state']=='LIVE','Preview not live; restore page first'
os.kill(s['pid'],0)
assert 0<=time.monotonic()-s['last_state_monotonic']<10,'Stale preview state'
c=json.loads((p/'command_state.json').read_text())
print('PS_CONTROL:'+json.dumps(dict(camera=s,command=c)))
