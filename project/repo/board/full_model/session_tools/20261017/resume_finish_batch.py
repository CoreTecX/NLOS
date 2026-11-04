"""Observe the resumed job, then recover, audit and merge both boot sessions."""
import hashlib,json,shutil,sys,time
from resume_common import ROOT,REMOTE,BOOT,serial
from resume_prepare_results import DATASET,RELEASE,run
NAME='dataset45_pl_resume_01'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    cp=json.loads((ROOT/'pause_checkpoint.json').read_text('utf8'))
    deadline=time.monotonic()+10800
    while True:
        code="""import json,os
from pathlib import Path
s=Path('"""+REMOTE+"""');r=json.loads((s/'jobs/"""+NAME+""".status.json').read_text())
p=s/'results/"""+NAME+"""/target_suite.json'
r['suite']=json.loads(p.read_text()) if p.exists() else {'cases':[]}
r['uptime_now']=Path('/proc/uptime').read_text().strip();r['temperatures_now']={}
for raw in Path('/sys/bus/iio/devices').glob('iio:device*/in_temp*_raw'):
 stem=str(raw)[:-4]
 try:r['temperatures_now'][raw.name]=(float(raw.read_text())+float(Path(stem+'_offset').read_text()))*float(Path(stem+'_scale').read_text())/1000
 except (ValueError,OSError):pass
r['cpu_khz_now']=Path('/sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq').read_text().strip()
r['log_tail']=(s/'jobs/"""+NAME+""".log').read_text(errors='replace')[-1200:]
r['display']=json.loads((s/'display/live_state.json').read_text())
try:os.kill(int((s/'display/resume_viewer_pid').read_text()),0);r['viewer_alive']=True
except ProcessLookupError:r['viewer_alive']=False
print('PS_TRANSFER_JSON='+json.dumps(r),flush=True)
"""
        r=serial(code,'resume_batch_progress_remote')
        if r['boot_id']!=BOOT:raise RuntimeError('Job boot changed')
        (ROOT/'resume_batch_progress.json').write_text(json.dumps(r,indent=2),encoding='utf8')
        with (ROOT/'resume_batch_telemetry.jsonl').open('a',encoding='utf8') as f:f.write(json.dumps(r)+'\n')
        rows=r['suite']['cases']
        print('RESUMED_PROGRESS',json.dumps(dict(state=r['state'],completed=52+len(rows),total=97,last=rows[-1]['key'] if rows else cp['completed'][-1],displayed=r['display']['number'],viewer_alive=r['viewer_alive'],temperature_max_c=max(r['temperatures_now'].values()),elapsed=round(float(r['uptime_now'].split()[0])-float(r['uptime_start'].split()[0]),1))),flush=True)
        if not r['viewer_alive'] and r['state']=='RUNNING':raise RuntimeError('Live viewer stopped; inspect before continuing')
        if r['state']!='RUNNING':break
        if time.monotonic()>deadline:raise RuntimeError('Three-hour observation limit reached; inspect ongoing job')
        time.sleep(25)
    run([ROOT/'resume_common.py','jtag_download.py','--name',NAME,'--path','results/'+NAME,'--path','jobs/'+NAME+'.status.json','--path','jobs/'+NAME+'.log'],'resume_download45')
    if r['state']!='FINISHED' or r.get('exit_code')!=0 or [x['key'] for x in rows]!=cp['remaining'] or any(x['exit_code'] for x in rows):raise RuntimeError('Continuation did not complete exactly 45 selected cases')
    first=ROOT/'downloads/lunch52_snapshot/content/results/dataset97_pl_01'
    second=ROOT/'downloads'/NAME/'content/results'/NAME
    report45=ROOT/'resume45_acceptance.json'
    args=[DATASET/'scripts/accept_dataset.py','--dataset',DATASET,'--release',RELEASE,'--results',second,'--report',report45]
    for key in cp['remaining']:args+=['--case',key]
    run(args,'resume_accept45')
    merged=ROOT/'merged/dataset97_pl_01';merged.mkdir(parents=True,exist_ok=False)
    suites=[json.loads((p/'target_suite.json').read_text('utf8')) for p in (first,second)]
    plans=[json.loads((p/'execution_plan.json').read_text('utf8')) for p in (first,second)]
    for field in ('backend','schedule','dct','release_manifest_sha256','dataset_package_sha256'):
        if suites[0][field]!=suites[1][field]:raise RuntimeError('Different batch contract: '+field)
    for field in ('profile','software_revision','hardware_release','dataset_manifest_sha256','dataset_package_sha256'):
        if plans[0][field]!=plans[1][field]:raise RuntimeError('Different execution plan: '+field)
    if sha(DATASET/'dataset_manifest.json')!=plans[0]['dataset_manifest_sha256'] or sha(RELEASE/'release_manifest.json')!=suites[0]['release_manifest_sha256']:raise RuntimeError('Local frozen data or release changed')
    if plans[0]['cases']!=cp['completed']+cp['remaining'] or plans[1]['cases']!=cp['remaining']:raise RuntimeError('Plan coverage mismatch')
    manifest={x['key']:x for x in json.loads((DATASET/'dataset_manifest.json').read_text('utf8'))['cases']}
    provenance=[];combined=[]
    for src,suite,expected,boot,export in zip((first,second),suites,(cp['completed'],cp['remaining']),(cp['boot_id'],BOOT),('lunch52_snapshot',NAME)):
        if [x['key'] for x in suite['cases']]!=expected:raise RuntimeError('Saved suite count/order changed')
        for row in suite['cases']:
            k=row['key'];data=manifest[k]
            if row['input_sha256']!=data['measurement']['sha256'] or row['model_sha256']!=data['model']['sha256'] or row['exit_code']!=0:raise RuntimeError('Input identity failed: '+k)
            record=dict(key=k,execution_boot_id=boot,download_export=export,board_result_directory=REMOTE+'/results/'+src.name,files=[])
            for ext in ('.pfo','.pfs','.json','.ppm','.log'):
                p=src/(k+ext);dest=merged/p.name;shutil.copy2(p,dest)
                if sha(p)!=sha(dest):raise RuntimeError('Merged copy differs')
                record['files'].append(dict(name=p.name,sha256=sha(p),bytes=p.stat().st_size))
            provenance.append(record);combined.append(row)
    if len(combined)!=97 or len({x['key'] for x in combined})!=97:raise RuntimeError('Merged coverage is not 97 unique cases')
    suite=dict(suites[0],cases=combined,host_merged=True,source_boot_ids=[cp['boot_id'],BOOT],provenance='merge_provenance.json')
    (merged/'target_suite.json').write_text(json.dumps(suite,indent=2),encoding='utf8')
    (merged/'execution_plan.json').write_text(json.dumps(dict(plans[0],host_merged=True),indent=2),encoding='utf8')
    (merged/'merge_provenance.json').write_text(json.dumps(dict(passed=True,count=97,cases=provenance),indent=2),encoding='utf8')
    audits=[json.loads((ROOT/'lunch52_acceptance.json').read_text('utf8')),json.loads(report45.read_text('utf8'))]
    if not all(x['passed'] for x in audits):raise RuntimeError('One audit failed')
    cases={x['key']:x for a in audits for x in a['cases']}
    if set(cases)!={x['key'] for x in combined} or not all(x['passed'] for x in cases.values()):raise RuntimeError('Independent audit coverage mismatch')
    audit=dict(passed=True,cases=[cases[x['key']] for x in combined],paper_blocked=47,hardware_origin='Separate boot-bound jobs, verified transport and merge_provenance.json',source_audits=['lunch52_acceptance.json','resume45_acceptance.json'])
    report=ROOT/'dataset97_acceptance.json'
    with report.open('x',encoding='utf8') as f:json.dump(audit,f,indent=2)
    run([ROOT/'render_results.py','--results',merged,'--acceptance',report,'--output',ROOT/'images/dataset97_pl_01'],'resume_render97')
    run([RELEASE/'scripts/accept_board.py',merged,'--all','--report',ROOT/'original15_board_acceptance.json'],'resume_original15_acceptance')
    progress=dict(r,state='FINISHED',suite=suite,completed_before_pause=52,completed_after_resume=45,source_boot_ids=[cp['boot_id'],BOOT])
    (ROOT/'batch_progress_before_resume_final.json').write_bytes((ROOT/'batch_progress.json').read_bytes())
    (ROOT/'batch_progress.json').write_text(json.dumps(progress,indent=2),encoding='utf8')
    (ROOT/'batch_verified.json').write_text(json.dumps(dict(passed=True,cases=97,board_results=str(merged),acceptance=str(report),images=str(ROOT/'images/dataset97_pl_01'),boot_ids=[cp['boot_id'],BOOT]),indent=2),encoding='utf8')
    print('RESUMED_BATCH_97_INDEPENDENTLY_VERIFIED',flush=True)

if __name__=='__main__':main()
