"""Read live progress without changing clocks, boot files, or hardware state."""
from jtag_transfer import serial,REMOTE,ROOT
import json,time
code="""import json,os
from pathlib import Path
s=Path('"""+REMOTE+"""')
r={'uptime':Path('/proc/uptime').read_text().strip(),'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'active':[],'temperatures':{}}
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  comm=(p/'comm').read_text().strip();cmd=(p/'cmdline').read_bytes().replace(b'\\0',b' ').decode(errors='replace')
  if comm.startswith(('ps_full','dct_test')) or 'board_unpack.py' in cmd:
   r['active'].append(dict(pid=p.name,comm=comm,cmd=cmd,stat=(p/'stat').read_text(),memory=[x for x in (p/'status').read_text().splitlines() if x.startswith(('VmRSS','VmHWM','State'))]))
 except OSError:pass
for raw in Path('/sys/bus/iio/devices').glob('iio:device*/in_temp*_raw'):
 stem=str(raw)[:-4]
 try:r['temperatures'][raw.name]=(float(raw.read_text())+float(Path(stem+'_offset').read_text()))*float(Path(stem+'_scale').read_text())/1000
 except (OSError,ValueError):pass
p=Path('/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq');r['cpu_khz']=p.read_text().strip() if p.exists() else None
files=[p for p in (s/'deployment').rglob('*') if p.is_file()]
r['deployment_files']=len(files);r['deployment_file_bytes']=sum(p.stat().st_size for p in files)
print('PS_TRANSFER_JSON='+json.dumps(r),flush=True)
"""
result=serial(code,'observe_live')
out=ROOT/'telemetry';out.mkdir(exist_ok=True)
(out/(str(time.time_ns())+'.json')).write_text(json.dumps(result,indent=2),encoding='utf8')
print(json.dumps(result,indent=2),flush=True)
