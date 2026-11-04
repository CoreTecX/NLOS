"""Bounded status monitor; does not launch or retry board work."""
import argparse,json,time
from jtag_transfer import serial,REMOTE,ROOT
p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--seconds',type=int,default=50);a=p.parse_args()
if not a.name.replace('_','').isalnum():raise ValueError('Invalid job')
end=time.monotonic()+a.seconds
while True:
    code="import json\nfrom pathlib import Path\ns=Path('"+REMOTE+"')\nr=json.loads((s/'jobs'/"+repr(a.name+'.status.json')+").read_text())\nr['uptime_now']=Path('/proc/uptime').read_text().strip()\np=s/'results'/"+repr(a.name)+"/'target_suite.json'\nif p.exists():r['suite']=json.loads(p.read_text())\nr['log_tail']=(s/'jobs'/"+repr(a.name+'.log')+").read_text(errors='replace')[-700:]\nprint('PS_TRANSFER_JSON='+json.dumps(r),flush=True)\n"
    r=serial(code,'status_'+a.name)
    (ROOT/(a.name+'.status.json')).write_text(json.dumps(r,indent=2),encoding='utf8')
    print(json.dumps(dict(job=a.name,state=r['state'],elapsed=round(float(r['uptime_now'].split()[0])-float(r['uptime_start'].split()[0]),1),cases=len(r.get('suite',{}).get('cases',[])),exit_code=r.get('exit_code'),log=r['log_tail'])),flush=True)
    if r['state']!='RUNNING' or time.monotonic()>=end:break
    time.sleep(min(20,max(0,end-time.monotonic())))
