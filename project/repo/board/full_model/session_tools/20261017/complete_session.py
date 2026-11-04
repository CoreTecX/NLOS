"""Finish evidence and stage publication after the full board batch is independently accepted."""
from pathlib import Path
import json,subprocess,sys,time
from jtag_transfer import ROOT,REMOTE,ENV,serial,XSDB

def run(args,name):
    with (ROOT/(name+'.log')).open('x',encoding='utf8') as f:r=subprocess.run(args,stdout=f,stderr=subprocess.STDOUT,env=ENV)
    print('SESSION_STAGE',name,r.returncode,flush=True)
    if r.returncode:raise RuntimeError(name+' failed; inspect log')

def main():
    end=time.monotonic()+10800
    while not (ROOT/'batch_verified.json').exists():
        if time.monotonic()>end:raise RuntimeError('Waiting for verified batch timed out; do not infer acceptance')
        time.sleep(20)
    run([sys.executable,'-B',str(ROOT/'finalize_display.py')],'finalize_display')
    run([sys.executable,'-B',str(ROOT/'capture_display.py')],'capture_final_display')
    run([sys.executable,'-B',str(ROOT/'board_control.py'),'upload','--file',str(ROOT/'final_board_evidence.py')],'upload_final_evidence')
    spec=dict(name='final_environment_01',command=['/usr/bin/python3.7','-B',REMOTE+'/tools/final_board_evidence.py'])
    p=ROOT/'final_environment_01.spec.json';p.write_text(json.dumps(spec,indent=2),encoding='utf8')
    run([sys.executable,'-B',str(ROOT/'board_control.py'),'job','--spec',str(p)],'start_final_evidence')
    for attempt in range(15):
        code="import json\nfrom pathlib import Path\np=Path('"+REMOTE+"')/'jobs/final_environment_01.status.json'\nprint('PS_TRANSFER_JSON='+p.read_text(),flush=True)\n"
        # Emit one-line JSON because the console parser is deliberately bounded.
        code=code.replace("+p.read_text()","+json.dumps(json.loads(p.read_text()))")
        r=serial(code,'final_evidence_status')
        if r['state']!='RUNNING':break
        time.sleep(2)
    if r['state']!='FINISHED' or r.get('exit_code')!=0:raise RuntimeError('Final board evidence failed')
    run([sys.executable,'-B',str(ROOT/'jtag_download.py'),'--name','final_evidence','--path','final_board_evidence.json','--path','original_config','--path','display','--path','jobs','--path','session.json','--path','transfer_complete.json','--path','deployment_complete.json'],'download_final_evidence')
    release=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\03_任务产出\P7_P11_Final_20261017\release\performance')
    results=ROOT/'downloads/dataset97_pl_01/content/results/dataset97_pl_01'
    run([sys.executable,'-B',str(release/'scripts/accept_board.py'),str(results),'--all','--report',str(ROOT/'original15_board_acceptance.json')],'original15_board_acceptance')
    tcl=ROOT/'restore_jtag_clock.tcl';tcl.write_text('connect -url tcp:127.0.0.1:33121\njtag targets -set 1\nputs "JTAG_RESTORED=[jtag frequency 10000000]"\ndisconnect\nexit\n',encoding='utf8')
    p=subprocess.run([XSDB,str(tcl)],env=ENV,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf8',errors='replace',timeout=30)
    (ROOT/'restore_jtag_clock.log').write_text(p.stdout,encoding='utf8')
    if p.returncode or 'JTAG_RESTORED=' not in p.stdout:print('JTAG_RESTORE_NEEDS_INSPECTION',flush=True)
    run([sys.executable,'-B',str(ROOT/'build_delivery.py'),'--name','delivery_final'],'build_final_delivery')
    clean=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\02_代码仓库\penumbrasight\build\scripts\clean_deliverables.py')
    run([sys.executable,'-B',str(clean),'--root',str(ROOT/'delivery_final')],'clean_final_delivery')
    run([sys.executable,'-B',str(ROOT/'publish_delivery.py'),'prepare',str(ROOT/'delivery_final')],'prepare_final_publication')
    (ROOT/'session_ready_to_publish.json').write_text(json.dumps(dict(passed=True,delivery=str(ROOT/'delivery_final'),plan=str(ROOT/'delivery_final_workspace_update/plan.json')),indent=2),encoding='utf8')
    print('SESSION_READY_TO_PUBLISH',flush=True)

if __name__=='__main__':main()
