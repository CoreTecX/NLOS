"""Hold reconstruction locks while exporting immutable snapshots via reserved DDR."""
from pathlib import Path
import fcntl,hashlib,json,os,sys,tarfile,time,traceback
SESSION=Path('/home/root/ps_runs_20261017/steps5_12_01')

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

def main():
    spec=json.loads(Path(sys.argv[1]).read_text());name=spec['name']
    if not name.replace('_','').isalnum():raise ValueError('Invalid export name')
    out=SESSION/'exports'/name;out.mkdir(parents=True,exist_ok=False)
    try:
        locks=[]
        for p in ['/var/lock/psight-full-launch.lock','/home/root/ps_board/algorithm.lock']:
            f=open(p,'a');fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(f)
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():continue
            try:comm=(p/'comm').read_text().strip()
            except OSError:continue
            if comm.startswith(('ps_full_','ps_board_app')):raise RuntimeError('Reconstruction active')
        files={}
        for rel in spec['paths']:
            p=SESSION/rel
            if SESSION not in p.resolve().parents or p.is_symlink():raise ValueError('Invalid export path')
            if not p.exists():raise ValueError('Missing export path: '+rel)
            for item in ([p] if p.is_file() else p.rglob('*')):
                if item.is_symlink():raise ValueError('Symlink in export')
                if item.is_file():files[str(item.relative_to(SESSION))]=item
        archive=out/'results.tar.gz'
        with tarfile.open(str(archive),'w:gz',compresslevel=3) as tar:
            for rel,p in sorted(files.items()):tar.add(str(p),arcname=rel,recursive=False)
        record=dict(name=name,path=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),pid=os.getpid(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),files=[dict(path=k,bytes=p.stat().st_size,sha256=sha(p)) for k,p in sorted(files.items())])
        (out/'ready.json').write_text(json.dumps(record,indent=2))
        end=time.monotonic()+1800
        while time.monotonic()<end and not (out/'release').exists():time.sleep(.25)
        (out/'released').write_text('requested' if (out/'release').exists() else 'timeout')
    except BaseException as e:
        (out/'failed.json').write_text(json.dumps(dict(error=str(e),traceback=traceback.format_exc())))
        raise

if __name__=='__main__':main()
