"""Validate/plan by default. --execute is explicitly required on AArch64 Linux.

Use only after the separate board environment and release acceptance steps.
All data remains outside the immutable 15-case release package.
"""
from pathlib import Path
import argparse,importlib.util,json,os,platform,sys,time
sys.dont_write_bytecode=True
from dataset_contract import validate,select_cases,validate_release,READY,sha

def load_launcher(release):
    sys.path.insert(0,str(release/'scripts'))
    spec=importlib.util.spec_from_file_location('pinned_launcher',str(release/'scripts/run_target.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

def write_json(path,value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf8')

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--release',type=Path,required=True)
    p.add_argument('--backend',choices=['cpu','pl'],default='cpu');p.add_argument('--schedule',choices=['sync','overlap'],default='sync')
    p.add_argument('--dct',choices=['optimized','legacy'],default='optimized')
    select=p.add_mutually_exclusive_group(required=True);select.add_argument('--case',action='append');select.add_argument('--all-ready',action='store_true')
    p.add_argument('--execute',action='store_true');p.add_argument('--output',type=Path);a=p.parse_args(argv)
    root=a.dataset.resolve();release=a.release.resolve()
    manifest,lock=validate(root);selected=select_cases(manifest,a.case,a.all_ready)
    package,pin=validate_release(release,lock)
    launcher=load_launcher(release)
    release_manifest=launcher.preflight(a.backend,[],a.dct,False)
    plan=dict(mode='EXECUTE_REQUESTED' if a.execute else 'PREFLIGHT_ONLY',backend=a.backend,schedule=a.schedule,
              release=package,profile=pin['profile'],software_revision=pin['software_revision'],hardware_release=pin['hardware_release'],
              cases=[r['key'] for r in selected],paper_blocked=len(manifest.get('blocked_paper',[])),
              reference_failed=[r['key'] for r in manifest['cases'] if r['reference_status'] not in READY],board_access=False)
    plan['dataset_manifest_sha256']=sha(root/'dataset_manifest.json')
    plan['dataset_package_sha256']=sha(root/'package_manifest.json')
    plan['dct']=a.dct
    print(json.dumps(plan,indent=2),flush=True)
    if not a.execute:return 0
    if platform.system()!='Linux' or platform.machine() not in ('aarch64','arm64'):raise RuntimeError('Execution requires AArch64 Linux; host preflight never initializes hardware')
    if a.output is None:raise ValueError('--execute requires a new --output directory')
    output=a.output.resolve()
    if output==root or root in output.parents or output==release or release in output.parents:raise ValueError('Results cannot modify dataset or release')
    output.mkdir(parents=True,exist_ok=False)
    import fcntl
    launch_lock=open('/var/lock/psight-full-launch.lock','a');old_lock=None
    results=[]
    try:
        fcntl.flock(launch_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if a.dct=='optimized':launcher.dct_selftest(output)
        if a.backend=='pl':old_lock=launcher.setup_and_load(release_manifest)
        env=os.environ.copy();env['PS_PL_SCHEDULE']=a.schedule;env['PS_EXPORT_STATE']='1'
        if a.backend=='pl':env['PS_FULL_LOADED']=launcher.TOKEN
        binary='ps_full_aarch64' if a.dct=='optimized' else 'ps_full_legacy_aarch64'
        write_json(output/'execution_plan.json',dict(plan,mode='BOARD_RUN',board_access=True))
        for row in selected:
            prefix=output/row['key'];args=[str(release/'bin'/binary),str(root/row['model']['file']),str(root/row['measurement']['file']),str(prefix),a.backend]
            started=time.monotonic()
            with prefix.with_suffix('.log').open('w') as log:
                result=launcher.run_case(args,log,env,a.backend)
            results.append(dict(key=row['key'],exit_code=result.returncode,seconds=time.monotonic()-started,
                                input_sha256=row['measurement']['sha256'],model_sha256=row['model']['sha256']))
            write_json(output/'target_suite.json',dict(physical_board_run=True,backend=a.backend,schedule=a.schedule,
                       dct=a.dct,release_manifest_sha256=pin['release_manifest_sha256'],dataset_package=str(root),
                       dataset_package_sha256=plan['dataset_package_sha256'],cases=results))
            if result.returncode:raise RuntimeError('Case failed; batch stopped. Preserve output and use a new directory after recovery: '+row['key'])
            print('BOARD_CASE_DONE',row['key'],flush=True)
    finally:
        if old_lock:old_lock.close()
        launch_lock.close()
    return 0

if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:print('REJECTED:',e,file=sys.stderr);sys.exit(2)
