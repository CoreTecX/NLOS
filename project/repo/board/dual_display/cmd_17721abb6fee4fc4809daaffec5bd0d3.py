from pathlib import Path
import json,time,os,uuid
r=Path('/home/root/ps_dualview_20261030');p=Path((r/'current.txt').read_text().strip());s=json.loads((p/'state.json').read_text());boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
assert s['boot_id']==boot and s['state']=='LIVE','Preview not live; restore page first'
os.kill(s['pid'],0)
assert time.monotonic()-s['last_state_monotonic']<10,'Stale preview state'
c=json.loads((p/'command_state.json').read_text())
assert c['state'] not in ('CAPTURING','RUNNING'),'Busy: wait for current reconstruction'
assert not (p/'request.json').exists(),'Command already queued'
req=dict(job_id=uuid.uuid4().hex,boot_id=boot,requested_monotonic=time.monotonic(),reset=False)
t=p/'request.tmp';t.write_text(json.dumps(req));t.replace(p/'request.json')
c.update(state='QUEUED',job_id=req['job_id'])
print('PS_CONTROL:'+json.dumps(dict(camera=s,command=c)))
