"""Recover and independently audit the saved 52 cases before new computation."""
from pathlib import Path
import json,subprocess,sys,time
from resume_common import ROOT,REMOTE,BOOT,serial
from jtag_transfer import ENV
DATASET=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\03_任务产出\Board_Steps_1_4_20261017\dataset')
RELEASE=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\03_任务产出\P7_P11_Final_20261017\release\performance')

def run(args,name):
    with (ROOT/(name+'.log')).open('x',encoding='utf8') as log:
        p=subprocess.run([sys.executable,'-B']+list(map(str,args)),stdout=log,stderr=subprocess.STDOUT,env=ENV)
    print('RESUME_STAGE',name,p.returncode,flush=True)
    if p.returncode:raise RuntimeError(name+' failed; inspect log')

def main():
    end=time.monotonic()+600
    while True:
        code="import json\nfrom pathlib import Path\ns=Path('"+REMOTE+"')\nr={'ready':(s/'resume_verified.json').exists(),'log_tail':(s/'resume_verify_board.log').read_text()[-800:]}\nif r['ready']:r['result']=json.loads((s/'resume_verified.json').read_text())\nprint('PS_TRANSFER_JSON='+json.dumps(r),flush=True)\n"
        r=serial(code,'resume_prepare_verify_poll')
        if r['ready']:break
        if time.monotonic()>end or 'Traceback' in r['log_tail']:raise RuntimeError('Board retention verification failed or timed out: '+str(r))
        print('RESUME_VERIFY_WAIT',flush=True);time.sleep(15)
    evidence=r['result'];cp=json.loads((ROOT/'pause_checkpoint.json').read_text('utf8'))
    if evidence['boot_id']!=BOOT or not evidence['passed'] or evidence['completed']!=cp['completed'] or evidence['remaining']!=cp['remaining']:raise RuntimeError('Checkpoint differs after power cycle')
    (ROOT/'resume_verified.json').write_text(json.dumps(evidence,indent=2),encoding='utf8')
    if not (ROOT/'downloads/lunch52_snapshot/verified.json').exists():
        run([ROOT/'resume_common.py','jtag_download.py','--name','lunch52_snapshot','--path','results/dataset97_pl_01','--path','pause_checkpoint.json','--path','resume_verified.json','--path','jobs/dataset97_pl_01.status.json','--path','jobs/dataset97_pl_01.log'],'resume_download52')
    result=ROOT/'downloads/lunch52_snapshot/content/results/dataset97_pl_01'
    report=ROOT/'lunch52_acceptance.json'
    args=[DATASET/'scripts/accept_dataset.py','--dataset',DATASET,'--release',RELEASE,'--results',result,'--report',report]
    for key in cp['completed']:args+=['--case',key]
    run(args,'resume_accept52')
    audit=json.loads(report.read_text('utf8'))
    if not audit['passed'] or {x['key'] for x in audit['cases']}!=set(cp['completed']):raise RuntimeError('Saved case audit coverage mismatch')
    run([ROOT/'render_results.py','--results',result,'--acceptance',report,'--output',ROOT/'images/lunch52'],'resume_render52')
    (ROOT/'resume_saved52_ready.json').write_text(json.dumps(dict(passed=True,cases=52,boot_id=BOOT,results=str(result),acceptance=str(report)),indent=2),encoding='utf8')
    print('SAVED_52_INDEPENDENTLY_ACCEPTED',flush=True)

if __name__=='__main__':main()
