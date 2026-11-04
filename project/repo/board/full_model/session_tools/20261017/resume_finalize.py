"""Finalize screen/evidence and stage an honest, reviewable workspace delivery."""
import hashlib,json,subprocess,sys,time
from resume_common import ROOT,REMOTE,BOOT,serial
from resume_prepare_results import run
from jtag_transfer import XSDB,ENV

def main():
    end=time.monotonic()+10800
    while not (ROOT/'batch_verified.json').exists():
        if time.monotonic()>end:raise RuntimeError('Full batch verification not ready; inspect monitor')
        time.sleep(20)
    report=json.loads((ROOT/'dataset97_acceptance.json').read_text('utf8'))
    if not report['passed'] or len(report['cases'])!=97 or not all(r['passed'] for r in report['cases']):raise RuntimeError('Full audit incomplete')
    results=ROOT/'merged/dataset97_pl_01'
    accepted={r['key']:hashlib.sha256((results/(r['key']+'.pfo')).read_bytes()).hexdigest() for r in report['cases']}
    p=ROOT/'display/resume_accepted_cases_final.json';p.write_text(json.dumps(accepted,sort_keys=True),encoding='utf8')
    run([ROOT/'resume_common.py','upload_serial_file.py',p,'display/resume_accepted_cases_final.json'],'resume_upload_final_acceptance')
    code="import json,os\nfrom pathlib import Path\nd=Path('"+REMOTE+"')/'display'\np=d/'resume_accepted_cases_final.json'\nif len(json.loads(p.read_text()))!=97:raise RuntimeError('Wrong accepted count')\nos.replace(str(p),str(d/'resume_accepted_cases.json'))\nprint('PS_TRANSFER_JSON='+json.dumps(dict(published=True)),flush=True)\n"
    serial(code,'resume_publish_final_acceptance')
    for attempt in range(10):
        code="import json\nfrom pathlib import Path\nd=Path('"+REMOTE+"')/'display'\nprint('PS_TRANSFER_JSON='+json.dumps(json.loads((d/'live_state.json').read_text())),flush=True)\n"
        state=serial(code,'resume_final_display_check')
        if state['number']==97 and state['independent_acceptance'] is True:break
        time.sleep(2)
    else:raise RuntimeError('Final display acceptance not confirmed')
    if state['boot_id']!=BOOT or state['source_sha256']!=accepted[state['case']]:raise RuntimeError('Final display identity mismatch')
    (ROOT/'display/final_display_state.json').write_text(json.dumps(state,indent=2),encoding='utf8')
    run([ROOT/'resume_common.py','capture_display.py'],'resume_capture_final_display')
    run([ROOT/'resume_common.py','board_control.py','upload','--file',ROOT/'final_board_evidence.py'],'resume_upload_final_evidence')
    code="import json,subprocess\nfrom pathlib import Path\ns=Path('"+REMOTE+"')\nwith (s/'final_board_evidence.log').open('x') as f:p=subprocess.Popen(['/usr/bin/python3.7','-B',str(s/'tools/final_board_evidence.py')],stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)\nprint('PS_TRANSFER_JSON='+json.dumps(dict(pid=p.pid)),flush=True)\n"
    serial(code,'resume_start_final_evidence')
    for attempt in range(15):
        code="import json\nfrom pathlib import Path\ns=Path('"+REMOTE+"')\nr={'ready':(s/'final_board_evidence.json').exists(),'log':(s/'final_board_evidence.log').read_text()[-800:]}\nif r['ready']:r['result']=json.loads((s/'final_board_evidence.json').read_text())\nprint('PS_TRANSFER_JSON='+json.dumps(r),flush=True)\n"
        r=serial(code,'resume_final_evidence_poll')
        if r['ready']:break
        if 'Traceback' in r['log']:raise RuntimeError(r['log'])
        time.sleep(2)
    else:raise RuntimeError('Final evidence missing')
    if r['result']['active'] or not r['result']['original_config_unchanged']:raise RuntimeError('Unexpected active algorithm or changed boot configuration')
    run([ROOT/'resume_common.py','jtag_download.py','--name','final_evidence','--path','final_board_evidence.json','--path','original_config','--path','display','--path','jobs','--path','session.json','--path','transfer_complete.json','--path','deployment_complete.json','--path','resume_verified.json','--path','pause_checkpoint.json'],'resume_download_final_evidence')
    tcl=ROOT/'restore_jtag_clock.tcl';tcl.write_text('connect -url tcp:127.0.0.1:33121\njtag targets -set 1\nputs "JTAG_RESTORED=[jtag frequency 10000000]"\ndisconnect\nexit\n',encoding='utf8')
    p=subprocess.run([XSDB,str(tcl)],env=ENV,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf8',errors='replace',timeout=30)
    (ROOT/'restore_jtag_clock.log').write_text(p.stdout,encoding='utf8')
    if p.returncode or 'JTAG_RESTORED=10000000' not in p.stdout:raise RuntimeError('JTAG restore not confirmed')
    run([ROOT/'build_delivery.py','--name','delivery_final'],'resume_build_delivery')
    run([ROOT/'publish_delivery.py','prepare',ROOT/'delivery_final'],'resume_prepare_publication')
    run([ROOT/'review_delivery.py'],'resume_review_delivery')
    repo=__import__('pathlib').Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\02_代码仓库\penumbrasight')
    run([repo/'build/scripts/workspace_check.py','--repo-root',repo,'--overlay',ROOT/'delivery_final_workspace_update/files/02_代码仓库/penumbrasight','--output',ROOT/'workspace_staged_integrity.json'],'resume_workspace_integrity')
    (ROOT/'session_ready_to_publish.json').write_text(json.dumps(dict(passed=True,delivery=str(ROOT/'delivery_final'),plan=str(ROOT/'delivery_final_workspace_update/plan.json')),indent=2),encoding='utf8')
    print('RESUME_DELIVERY_READY_TO_PUBLISH',flush=True)

if __name__=='__main__':main()
