"""Board-side extraction and complete hash checks; no algorithm or FPGA execution."""
from pathlib import Path
import hashlib,json,os,subprocess,tarfile,time,traceback
SESSION=Path('/home/root/ps_runs_20261017/steps5_12_01')
DEST=SESSION/'deployment'
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def main():
    transfer=json.loads((SESSION/'transfer_complete.json').read_text())
    archive=Path(transfer['path'])
    if sha(archive)!=transfer['sha256']:raise RuntimeError('Archive changed after transfer')
    DEST.mkdir(exist_ok=False)
    with tarfile.open(str(archive),'r:gz') as tar:
        members=tar.getmembers()
        for member in members:
            target=(DEST/member.name).resolve()
            if DEST not in target.parents or member.name.split('/')[0] not in ['performance','fallback','dataset']:raise RuntimeError('Archive path outside deployment')
            if not(member.isfile() or member.islnk()):raise RuntimeError('Unsupported archive member')
            if member.islnk() and DEST not in (DEST/member.linkname).resolve().parents:raise RuntimeError('Archive hardlink outside deployment')
        tar.extractall(str(DEST),members=members)
    result={'packages':[],'physical_board_hash_check':True,'algorithm_executed':False}
    for name in ['performance','fallback','dataset']:
        root=DEST/name
        if name=='dataset':entries=json.loads((root/'package_manifest.json').read_text())['files']
        else:entries=[dict(file=k,**v) for k,v in json.loads((root/'release_manifest.json').read_text())['files'].items()]
        for row in entries:
            p=root/row['file']
            if p.stat().st_size!=row['bytes'] or sha(p)!=row['sha256']:raise RuntimeError('Installed file mismatch: '+str(p))
        result['packages'].append(dict(package=name,files=len(entries),passed=True))
        print('PACKAGE_HASH_PASS',name,len(entries),flush=True)
    env=os.environ.copy();env['PATH']='/usr/sbin:/sbin:/usr/bin:/bin';env['PYTHONDONTWRITEBYTECODE']='1'
    for name in ['performance','fallback']:
        with (SESSION/('preflight_'+name+'.log')).open('w') as log:
            run=subprocess.run(['/usr/bin/python3.7','-B',str(DEST/name/'scripts/release_selfcheck.py')],stdout=log,stderr=subprocess.STDOUT,env=env)
        if run.returncode:raise RuntimeError('Installed release preflight failed: '+name)
    with (SESSION/'dataset_check.log').open('w') as log:
        run=subprocess.run(['/usr/bin/python3.7','-B',str(DEST/'dataset/scripts/check_dataset.py'),str(DEST/'dataset')],stdout=log,stderr=subprocess.STDOUT,env=env)
    if run.returncode:raise RuntimeError('Installed dataset check failed')
    result['passed']=True;result['deployment']=str(DEST)
    (SESSION/'deployment_complete.json').write_text(json.dumps(result,indent=2))
    (SESSION/'release_transfer_lease').write_text('complete')
    print(json.dumps(result),flush=True)

if __name__=='__main__':
    try:main()
    except BaseException as e:
        (SESSION/'deployment_failed.json').write_text(json.dumps(dict(error=str(e),traceback=traceback.format_exc())))
        raise
