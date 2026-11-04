from pathlib import Path
import json,shutil,hashlib,os
root=Path('/home/root/ps_runs_20261017')
session=root/'steps5_12_01'
if session.exists():raise RuntimeError('Session path already exists; inspect before reuse')
session.mkdir(parents=True)
files=[]
for source in ['/home/root/ps_board/safe_debug_init.sh','/home/root/ps_board/autostart_linux.sh','/home/root/ps_board/autostart_algorithm.sh','/home/root/ps_boot.scr','/home/root/ps_runtime.dtb']:
 p=Path(source)
 if p.is_file():
  dest=session/'original_config'/p.relative_to('/');dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(str(p),str(dest))
  files.append(dict(source=source,backup=str(dest),sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
record=dict(session=str(session),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),uptime=Path('/proc/uptime').read_text().strip(),backups=files,boot_configuration_changed=False,algorithm_deployed=False)
(session/'session.json').write_text(json.dumps(record,indent=2))
print('PS_SESSION='+json.dumps(record),flush=True)
