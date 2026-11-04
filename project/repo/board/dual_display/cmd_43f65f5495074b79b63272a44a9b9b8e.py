from pathlib import Path
import time,json
try:exec(compile(Path('/home/root/ps_dualview_20261030/start_board.py').read_text(),'<start>','exec'))
except SystemExit:pass
for _ in range(40):
 p=Path(Path('/home/root/ps_dualview_20261030/current.txt').read_text().strip())
 if (p/'state.json').exists() and json.loads((p/'state.json').read_text()).get('state')=='LIVE':break
 time.sleep(.25)
from pathlib import Path
import json,time,os,uuid
r=Path('/home/root/ps_dualview_20261030');p=Path((r/'current.txt').read_text().strip());s=json.loads((p/'state.json').read_text());boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assert s['boot_id']==boot and s['state']=='LIVE','Preview not live; restore page first'
os.kill(s['pid'],0)
assert time.monotonic()-s['last_state_monotonic']<10,'Stale preview state'
c=json.loads((p/'command_state.json').read_text())
print('PS_CONTROL:'+json.dumps(dict(camera=s,command=c)))
