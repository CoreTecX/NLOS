"""Run one authorized board command and save its actual status and resource evidence."""
from pathlib import Path
import json,os,resource,subprocess,sys,time,traceback
SESSION=Path('/home/root/ps_runs_20261017/steps5_12_01')
def temperatures():
    result={}
    for raw in Path('/sys/bus/iio/devices').glob('iio:device*/in_temp*_raw'):
        stem=str(raw)[:-4]
        try:result[raw.name]=((float(raw.read_text())+float(Path(stem+'_offset').read_text()))*float(Path(stem+'_scale').read_text())/1000)
        except (ValueError,OSError):pass
    return result
def main():
    specification=Path(sys.argv[1]);spec=json.loads(specification.read_text());name=spec['name'];root=SESSION/'jobs'
    root.mkdir(exist_ok=True);status=root/(name+'.status.json');log=root/(name+'.log')
    if status.exists() or log.exists():raise RuntimeError('Job output already exists')
    record=dict(name=name,command=spec['command'],pid=os.getpid(),state='RUNNING',boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),uptime_start=Path('/proc/uptime').read_text().strip(),temperatures_start=temperatures(),physical_board_run=True)
    cpufreq=Path('/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq')
    record['cpu_khz_start']=cpufreq.read_text().strip() if cpufreq.exists() else None
    status.write_text(json.dumps(record,indent=2));env=os.environ.copy();env['PATH']='/usr/sbin:/sbin:/usr/bin:/bin';env['PYTHONDONTWRITEBYTECODE']='1'
    start=time.monotonic()
    try:
        with log.open('w') as f:run=subprocess.run(spec['command'],env=env,stdout=f,stderr=subprocess.STDOUT)
        record['exit_code']=run.returncode;record['state']='FINISHED' if run.returncode==0 else 'FAILED'
    except BaseException as e:
        record.update(state='FAILED',error=str(e),traceback=traceback.format_exc());raise
    finally:
        record.update(seconds=time.monotonic()-start,peak_child_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,temperatures_end=temperatures(),uptime_end=Path('/proc/uptime').read_text().strip())
        record['cpu_khz_end']=cpufreq.read_text().strip() if cpufreq.exists() else None
        temp=status.with_suffix('.tmp');temp.write_text(json.dumps(record,indent=2));temp.replace(status)
    print(json.dumps(record),flush=True)
    return record['exit_code']
if __name__=='__main__':sys.exit(main())
