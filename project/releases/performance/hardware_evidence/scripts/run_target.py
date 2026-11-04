"""AXU9EGB Linux launcher for the isolated full-model release. Run on the board only."""
from pathlib import Path
import argparse,hashlib,json,os,platform,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from hardware_profile import TOKEN,profile_for,configure_clock,verify_binding
def verify(path,expected):
    if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:raise RuntimeError('SHA256 mismatch: '+str(path))
def preflight(backend,selected,dct='optimized',profile=False):
    manifest=json.loads((ROOT/'release_manifest.json').read_text('utf-8'))
    if manifest.get('release_id')!=TOKEN or manifest.get('layout_version')!=0x32525001:
        raise RuntimeError('Wrong release/layout manifest')
    profile_for(manifest)
    binary='ps_full_aarch64' if dct=='optimized' else 'ps_full_legacy_aarch64'
    if profile:
        if dct!='optimized':raise RuntimeError('Profiling is available for the current optimized binary only')
        binary+='_profile'
        if 'bin/'+binary not in manifest['files']:raise RuntimeError('Profile binary absent from candidate manifest; do not modify frozen package')
    required=['bin/'+binary,'scripts/run_target.py','scripts/hardware_profile.py','data/manifest.json']
    if dct=='optimized':required+=['bin/dct_test_aarch64']
    if backend=='pl':required+=['bin/full_board_fan_safe.bit','configs/hardware_profiles.json']
    for r in selected:required += ['data/'+r['model']['file'],'data/'+r['measurement']['file']]
    for name in sorted(set(required)):verify(ROOT/name,manifest['files'][name]['sha256'])
    if backend=='pl':verify_binding(manifest,json.loads((ROOT/'configs/hardware_profiles.json').read_text('utf-8')),binary)
    elf=(ROOT/'bin'/binary).read_bytes()[:64]
    if elf[:6]!=b'\x7fELF\x02\x01' or int.from_bytes(elf[18:20],'little')!=183:raise RuntimeError('Invalid AArch64 ELF')
    return manifest
def setup_and_load(manifest):
    import mmap,struct,fcntl
    oldlock=Path('/home/root/ps_board/algorithm.lock')
    if oldlock.parent.exists():
        lock=oldlock.open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    else:lock=None
    # Old scripts which predate their own lock may still have a live application.
    for p in Path('/proc').iterdir():
        if p.name.isdigit():
            try:comm=(p/'comm').read_text().strip()
            except OSError:continue
            if comm.startswith('ps_board_app') or comm.startswith('ps_full_'):raise RuntimeError('Another reconstruction is running: '+comm)
    fd=os.open('/dev/mem',os.O_RDWR|os.O_SYNC)
    def read(addr):
        with mmap.mmap(fd,4096,offset=addr&~4095) as m:
            return struct.unpack_from('<I',m,addr&4095)[0]
    def mask(addr,bits,value):
        with mmap.mmap(fd,4096,offset=addr&~4095) as m:
            off=addr&4095;old=struct.unpack_from('<I',m,off)[0];struct.pack_into('<I',m,off,(old&~bits)|(value&bits))
    profile=profile_for(manifest)
    # Match BD's IOPLL source. Validate it before changing any PS/PL state.
    from hardware_profile import check_iopll
    check_iopll(read)
    mask(0xff0a0344,0x80000000,0x80000000);mask(0xff0a0348,0x80000000,0x80000000)
    mask(0xff0a0054,0x80000000,0);time.sleep(.01)
    configure_clock(read,mask,time.sleep,profile)
    mask(0xfd615000,0x300,0)
    for addr in (0xfd380000,0xfd390000,0xfd380014,0xfd390014):mask(addr,3,0)
    if read(0xfd615000)&0x300 or any(read(a)&3 for a in (0xfd380000,0xfd390000,0xfd380014,0xfd390014)):
        raise RuntimeError('PS AXI width readback mismatch; keep PL reset')
    firmware=Path('/lib/firmware')/('ps_'+TOKEN+'_'+profile['name']+'.bit');source=ROOT/'bin'/'full_board_fan_safe.bit'
    if firmware.exists() and firmware.read_bytes()!=source.read_bytes():raise RuntimeError('A different full-model firmware already occupies '+str(firmware))
    if not firmware.exists():firmware.write_bytes(source.read_bytes())
    manager=Path('/sys/class/fpga_manager/fpga0')
    (manager/'flags').write_text('0');(manager/'firmware').write_text(firmware.name)
    if (manager/'state').read_text().strip()!='operating':raise RuntimeError('FPGA manager did not enter operating state')
    # PL logic reset does not change the statically enabled J10 cooling output.
    mask(0xff0a0344,0x80000000,0x80000000);mask(0xff0a0348,0x80000000,0x80000000)
    mask(0xff0a0054,0x80000000,0);time.sleep(.01);mask(0xff0a0054,0x80000000,0x80000000);time.sleep(.01)
    os.close(fd);return lock
def dct_selftest(output):
    check=subprocess.run([str(ROOT/'bin/dct_test_aarch64')],capture_output=True,text=True,timeout=180)
    (output/'dct_selftest.log').write_text(check.stdout+check.stderr)
    if check.returncode:raise RuntimeError('ARM DCT self-test failed; hardware not initialized')
    result=json.loads(check.stdout)
    if not isinstance(result,dict) or result.get('passed') is not True or result.get('bit_exact') is not True or result.get('uses_neon') is not True or result.get('cases')!=3220:
        raise RuntimeError('Invalid ARM DCT self-test result; hardware not initialized')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',choices=['cpu','pl'],default='cpu')
    p.add_argument('--dct',choices=['optimized','legacy'],default='optimized')
    group=p.add_mutually_exclusive_group();group.add_argument('--case',default=None);group.add_argument('--all',action='store_true')
    p.add_argument('--profile',action='store_true',help='Use manifest-verified optional phase profiling binary')
    p.add_argument('--output',type=Path);p.add_argument('--preflight-only',action='store_true');a=p.parse_args()
    rows=json.loads((ROOT/'data'/'manifest.json').read_text('utf-8'));key=a.case or 'D11_rgb'
    selected=rows if a.all else [r for r in rows if r['key']==key]
    if not selected:raise RuntimeError('Unknown case '+key)
    timings={};tic=time.monotonic()
    manifest=preflight(a.backend,selected,a.dct,a.profile)
    timings['package_preflight_seconds']=time.monotonic()-tic
    if a.preflight_only:print('PACKAGE_PREFLIGHT_PASS',a.backend,len(selected));return 0
    if platform.system()!='Linux' or platform.machine() not in ('aarch64','arm64'):raise RuntimeError('Execution requires AArch64 Linux')
    if a.output is None:raise RuntimeError('--output must name a new result directory')
    a.output.mkdir(parents=True,exist_ok=False)
    # Hold a launcher lock through programming and the entire suite.
    import fcntl
    launch_lock=open('/var/lock/psight-full-launch.lock','a');fcntl.flock(launch_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    tic=time.monotonic()
    if a.dct=='optimized':dct_selftest(a.output)
    timings['dct_selftest_seconds']=time.monotonic()-tic
    tic=time.monotonic()
    old_lock=setup_and_load(manifest) if a.backend=='pl' else None
    timings['hardware_setup_seconds']=time.monotonic()-tic
    if a.profile:(a.output/'launcher_profile.json').write_text(json.dumps(dict(physical_board_run=True,one_time=timings),indent=2))
    env=os.environ.copy()
    if a.profile:env['PS_PROFILE']='1'
    if a.backend=='pl':env['PS_FULL_LOADED']=TOKEN
    results=[]
    for r in selected:
        prefix=a.output/r['key'];args=[str(ROOT/'bin'/(('ps_full_aarch64_profile' if a.profile else 'ps_full_aarch64') if a.dct=='optimized' else 'ps_full_legacy_aarch64')),str(ROOT/'data'/r['model']['file']),str(ROOT/'data'/r['measurement']['file']),str(prefix),a.backend]
        tic=time.monotonic()
        with prefix.with_suffix('.log').open('w') as log:
            proc=subprocess.run(args,stdout=log,stderr=subprocess.STDOUT,env=env,timeout=1800)
        results.append(dict(key=r['key'],exit_code=proc.returncode,seconds=time.monotonic()-tic))
        (a.output/'target_suite.json').write_text(json.dumps(dict(backend=a.backend,dct=a.dct,physical_board_run=True,cases=results),indent=2))
        print(r['key'],proc.returncode,flush=True)
        if proc.returncode:raise RuntimeError('Stopped at failed case; preserve log and reload PL before retry')
    # Keep references alive until all child processes have ended.
    if old_lock:old_lock.close()
    launch_lock.close();return 0
if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:print('REJECTED:',e,file=sys.stderr);sys.exit(2)
