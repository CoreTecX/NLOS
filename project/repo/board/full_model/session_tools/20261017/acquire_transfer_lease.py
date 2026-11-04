import subprocess,sys,time,json,os
from pathlib import Path
session=Path('/home/root/ps_runs_20261017/steps5_12_01')
source="""import fcntl,time,json,os
from pathlib import Path
s=Path('/home/root/ps_runs_20261017/steps5_12_01')
locks=[]
for name in ['/var/lock/psight-full-launch.lock','/home/root/ps_board/algorithm.lock']:
 p=Path(name);folder=p.parent.resolve()
 if str(folder) not in ['/var/lock','/var/volatile/lock','/run/lock','/home/root/ps_board']:raise RuntimeError('Unexpected lock directory target: '+str(folder))
 folder.mkdir(parents=True,exist_ok=True);f=p.open('a');fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(f)
for p in Path('/proc').iterdir():
 if p.name.isdigit():
  try:comm=(p/'comm').read_text().strip()
  except OSError:continue
  if comm.startswith(('ps_full','ps_board')):raise RuntimeError('Active reconstruction process')
(s/'transfer_lease.json').write_text(json.dumps(dict(pid=os.getpid(),address='0x50000000',bytes=33554432,locks_held=True)))
start=time.monotonic()
while not (s/'release_transfer_lease').exists() and time.monotonic()-start<7200:time.sleep(.25)
for f in locks:f.close()
(s/'transfer_lease_released').write_text('released')
"""
if (session/'transfer_lease.json').exists():raise RuntimeError('Inspect existing lease before reuse')
with (session/'transfer_lease.log').open('w') as log:
 p=subprocess.Popen([sys.executable or os.readlink('/proc/self/exe'),'-c',source],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
time.sleep(.4)
if p.poll() is not None or not (session/'transfer_lease.json').exists():raise RuntimeError('Transfer lease not acquired')
print((session/'transfer_lease.json').read_text(),flush=True)
