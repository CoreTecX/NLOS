"""Monitor the already-started batch, retrieve evidence, audit, then render accepted results."""
from pathlib import Path
import json,subprocess,sys,time
from jtag_transfer import ROOT,REMOTE,ENV,serial
DATASET=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\03_任务产出\Board_Steps_1_4_20261017\dataset')
RELEASE=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\03_任务产出\P7_P11_Final_20261017\release\performance')
NAME='dataset97_pl_01'

def run(args,log):
    with (ROOT/log).open('x',encoding='utf8') as f:
        p=subprocess.run(args,env=ENV,stdout=f,stderr=subprocess.STDOUT)
    print('LOCAL_STAGE',log,p.returncode,flush=True)
    if p.returncode:raise RuntimeError('Stage failed: '+log)

def main():
    deadline=time.monotonic()+10800
    while True:
        code="""import json
from pathlib import Path
s=Path('"""+REMOTE+"""');r=json.loads((s/'jobs/dataset97_pl_01.status.json').read_text())
p=s/'results/dataset97_pl_01/target_suite.json'
if p.exists():r['suite']=json.loads(p.read_text())
r['uptime_now']=Path('/proc/uptime').read_text().strip();r['temperatures_now']={}
for raw in Path('/sys/bus/iio/devices').glob('iio:device*/in_temp*_raw'):
 stem=str(raw)[:-4]
 try:r['temperatures_now'][raw.name]=(float(raw.read_text())+float(Path(stem+'_offset').read_text()))*float(Path(stem+'_scale').read_text())/1000
 except (ValueError,OSError):pass
r['cpu_khz_now']=Path('/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq').read_text().strip()
r['log_tail']=(s/'jobs/dataset97_pl_01.log').read_text(errors='replace')[-1200:]
print('PS_TRANSFER_JSON='+json.dumps(r),flush=True)
"""
        r=serial(code,'batch_progress')
        (ROOT/'batch_progress.json').write_text(json.dumps(r,indent=2),encoding='utf8')
        with (ROOT/'batch_telemetry.jsonl').open('a',encoding='utf8') as f:f.write(json.dumps(r)+'\n')
        rows=r.get('suite',{}).get('cases',[])
        print('BATCH_PROGRESS',json.dumps(dict(state=r['state'],completed=len(rows),total=97,last_case=rows[-1] if rows else None,elapsed=round(float(r['uptime_now'].split()[0])-float(r['uptime_start'].split()[0]),1),temperature_max_c=max(r['temperatures_now'].values()),cpu_khz=r['cpu_khz_now'])),flush=True)
        if r['state']!='RUNNING':break
        if time.monotonic()>deadline:raise RuntimeError('Monitor reached three-hour observation limit; inspect active board job before taking action')
        time.sleep(25)
    run([sys.executable,'-B',str(ROOT/'jtag_download.py'),'--name',NAME,'--path','results/'+NAME,'--path','jobs/'+NAME+'.status.json','--path','jobs/'+NAME+'.log'],'batch_download.log')
    if r['state']!='FINISHED' or r.get('exit_code')!=0 or len(rows)!=97 or any(x['exit_code']!=0 for x in rows):raise RuntimeError('Board batch did not pass all 97 runs; downloaded evidence retained')
    result=ROOT/'downloads'/NAME/'content/results'/NAME
    report=ROOT/'dataset97_acceptance.json'
    run([sys.executable,'-B',str(DATASET/'scripts/accept_dataset.py'),'--dataset',str(DATASET),'--release',str(RELEASE),'--results',str(result),'--report',str(report)],'dataset97_acceptance.log')
    run([sys.executable,'-B',str(ROOT/'render_results.py'),'--results',str(result),'--acceptance',str(report),'--output',str(ROOT/'images'/NAME)],'dataset97_render.log')
    (ROOT/'batch_verified.json').write_text(json.dumps(dict(passed=True,cases=97,board_results=str(result),acceptance=str(report),images=str(ROOT/'images'/NAME)),indent=2),encoding='utf8')
    print('BATCH_RECONSTRUCTION_VERIFIED',97,flush=True)

if __name__=='__main__':main()
