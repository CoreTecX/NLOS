"""Reproduce P1 observations serially; all paths and measurements are archived."""
from pathlib import Path
import json, hashlib, shutil, subprocess, argparse, sys
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('action',choices=['check','benchmark'])
parser.add_argument('--verification',type=Path,required=True,help='Completed full_model.py verify --profile output directory')
parser.add_argument('--output',type=Path,required=True,help='New output for check; same directory for subsequent benchmark')
args=parser.parse_args()
ROOT=args.verification.resolve().parent
VERIFICATION=args.verification.resolve()
OUT=args.output.resolve()
verified=json.loads((VERIFICATION/'verification.json').read_text('utf-8'))
assert verified['passed']
CAND=Path(verified['repo_root']);M8=Path(verified['frozen_release'])
STAGE=Path(verified['staging'])
HOST=verified['build']['jobs'][0]['command'][0]
if OUT==M8 or M8 in OUT.parents:raise ValueError('Cannot write into frozen release')
if args.action=='check':OUT.mkdir(parents=True,exist_ok=False)
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def write(p,x):
    p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(x,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
def cp(s,d):
    d.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(s,d)
if args.action=='check':
    write(OUT/'protocol.json',dict(representative=['D11_tommy','D11_rgb','S14_colbar'],repeats=5,warmups=1,all_existing_cases_previously_seen=True))

import os, time, statistics, platform, importlib.util

def command(args,log,env=None,allowed=(0,),cwd=STAGE):
    start=time.perf_counter()
    p=subprocess.run([str(x) for x in args],cwd=cwd,env=env,capture_output=True,encoding='utf-8',errors='replace',timeout=2400)
    elapsed=time.perf_counter()-start
    log.parent.mkdir(parents=True,exist_ok=True);log.write_text(p.stdout+p.stderr,encoding='utf-8')
    assert p.returncode in allowed,(p.returncode,str(log),(p.stdout+p.stderr)[-1000:])
    return elapsed,p.returncode
def check_equal(prefix,ref):
    assert sha(prefix.with_suffix('.pfo'))==sha(ref.with_suffix('.pfo')),str(prefix)
    a=json.loads(prefix.with_suffix('.json').read_text());b=json.loads(ref.with_suffix('.json').read_text())
    del a['seconds_solver'];del b['seconds_solver'];assert a==b,str(prefix)
def check_profile(prefix):
    p=json.loads(prefix.with_suffix('.profile.json').read_text())['phases']
    r=json.loads(prefix.with_suffix('.json').read_text());n=r['iterations']
    for k in ['iteration','rhs','dct','kt_gemv','spectral_scale','k_gemv','vector_update']:assert p[k]['calls']==n,(k,n)
    assert p['residual_check']['calls']==len(r['trace'])
    for v in p.values():assert 0<=v['exclusive_seconds']<=v['inclusive_seconds']
    assert abs(sum(v['exclusive_seconds'] for v in p.values())-p['main']['inclusive_seconds'])<1e-7
    return p
def solve(row,kind,prefix,profile):
    env=os.environ.copy();env.pop('PS_PROFILE',None)
    if profile:env['PS_PROFILE']='1'
    exe='ps_full_'+('host' if kind=='native' else 'packed_host')+('_profile' if profile else '')+'.exe'
    duration,code=command([STAGE/'bin'/exe,STAGE/'data'/row['model']['file'],STAGE/'data'/row['measurement']['file'],prefix,'cpu'],prefix.with_suffix('.log'),env)
    check_equal(prefix,STAGE/'results'/kind/row['key'])
    if profile:check_profile(prefix)
    r=json.loads(prefix.with_suffix('.json').read_text())
    return dict(case=row['key'],variant=kind,profile=profile,process_seconds=duration,solver_seconds=r['seconds_solver'],iterations=r['iterations'],prefix=str(prefix),binary_sha256=sha(STAGE/'bin'/exe))
def main():
    verified=json.loads((VERIFICATION/'verification.json').read_text('utf-8'));assert verified['passed']
    # Updated launcher is Python-only. Re-run its tests after copying it into the assembled stage.
    cp(CAND/'board/full_model/run_target.py',STAGE/'scripts/run_target.py')
    command([sys.executable,'-B',STAGE/'tests/launcher_checks.py'],OUT/'launcher_checks.log')
    cp(STAGE/'reports/launcher_checks.json',OUT/'launcher_checks.json')
    assert (STAGE/'bin/profile_tests.exe').exists(), 'Build with --profile first'
    command([STAGE/'bin/profile_tests.exe'],OUT/'timer_calibration.json')
    rows=json.loads((STAGE/'data/manifest.json').read_text('utf-8'))
    records=[]
    for kind in ['native','packed']:
        dest=OUT/'profile_regression'/kind;dest.mkdir(parents=True)
        for row in rows:
            records.append(solve(row,kind,dest/row['key'],True));print('PROFILE_EXACT',kind,row['key'],flush=True)
        command([sys.executable,'-B',STAGE/'scripts/accept_board.py',dest,'--all','--report',dest/'comparison.json'],dest/'comparison.log')
    write(OUT/'profile_regression.json',dict(passed=True,runs=records,bit_exact=True,non_timing_json_exact=True))
    # Functional check: compiled-in profiler with runtime switch OFF writes no profile file.
    row=rows[0];dest=OUT/'runtime_disabled';dest.mkdir()
    command([STAGE/'bin/ps_full_host_profile.exe',STAGE/'data'/row['model']['file'],STAGE/'data'/row['measurement']['file'],dest/row['key'],'cpu'],dest/'run.log',dict(os.environ,PS_PROFILE='0'))
    check_equal(dest/row['key'],STAGE/'results/native'/row['key']);assert not (dest/(row['key']+'.profile.json')).exists()
    # Legacy source and frozen sources are not changed by this observation pass.
    write(OUT/'functional_done.json',dict(passed=True,profile_cases=30,runtime_disabled=True))
    print('P1_FUNCTIONAL_PASS',flush=True)
def benchmark():
    assert json.loads((OUT/'functional_done.json').read_text())['passed']
    # Refuse to overlap performance runs with known project compilation/simulation processes.
    processes=subprocess.check_output(['powershell','-NoProfile','-Command',"Get-Process | Where-Object {$_.ProcessName -match '^(xelab|xsim|xsimk|cc1plus|g\\+\\+|ps_full.*)$'} | Select-Object -ExpandProperty ProcessName"],encoding='utf-8',errors='replace')
    assert not processes.strip(),processes
    command([STAGE/'bin/profile_tests.exe'],OUT/'timer_calibration.json')
    protocol=json.loads((OUT/'protocol.json').read_text('utf-8'));rows=json.loads((STAGE/'data/manifest.json').read_text('utf-8'))
    selected=[next(r for r in rows if r['key']==k) for k in protocol['representative']]
    dest=OUT/'benchmark';dest.mkdir()
    records=[]
    for profile in [False,True]:
        d=dest/('warmup_'+str(int(profile)));d.mkdir()
        rec=solve(selected[1],'native',d/selected[1]['key'],profile);rec['warmup']=True;records.append(rec)
    for repeat in range(5):
        for index in range(3):
            row=selected[(index+repeat)%3]
            for profile in ([False,True] if repeat%2==0 else [True,False]):
                d=dest/('rep%d_%s'%(repeat,'on' if profile else 'off'));d.mkdir(exist_ok=True)
                rec=solve(row,'native',d/row['key'],profile);rec.update(repeat=repeat,warmup=False);records.append(rec)
                write(OUT/'benchmark_progress.json',records)
                print('TIMED',repeat,row['key'],'on' if profile else 'off',round(rec['process_seconds'],4),flush=True)
    def summary(v):return dict(median=statistics.median(v),min=min(v),max=max(v),range_fraction=(max(v)-min(v))/statistics.median(v),values=v)
    grouped={}
    for row in selected:
        key=row['key'];grouped[key]={}
        for profile in [False,True]:
            samples=[r for r in records if not r['warmup'] and r['case']==key and r['profile']==profile]
            grouped[key]['on' if profile else 'off']={name:summary([r[name] for r in samples]) for name in ['process_seconds','solver_seconds']}
        grouped[key]['instrumentation_ratio']=grouped[key]['on']['process_seconds']['median']/grouped[key]['off']['process_seconds']['median']
    # Host package preflight/selfcheck is a separate one-time protocol; never executes target setup.
    spec=importlib.util.spec_from_file_location('frozen_launcher',M8/'scripts/run_target.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    timings=[]
    for i in range(6):
        t=time.perf_counter();mod.preflight('cpu',selected);preflight=time.perf_counter()-t
        duration,_=command([STAGE/'bin/dct_test.exe'],OUT/'selfcheck'/('dct_%d.log'%i))
        timings.append(dict(warmup=i==0,package_preflight_seconds=preflight,host_dct_selftest_seconds=duration))
    write(OUT/'benchmark.json',dict(passed=True,host_only=True,platform=platform.platform(),processor=platform.processor(),runs=records,summary=grouped,one_time_launcher=timings,cache_state='uncontrolled OS cache; every solver is a new process',concurrent_project_work=False))
    print('P1_BENCHMARK_PASS',flush=True)
if __name__=='__main__':
    import sys
    benchmark() if args.action=='benchmark' else main()
