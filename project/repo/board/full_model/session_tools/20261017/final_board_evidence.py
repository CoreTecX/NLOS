"""Record final board state and verify that original boot configuration is unchanged."""
from pathlib import Path
import hashlib,json,mmap,os,struct,subprocess
S=Path('/home/root/ps_runs_20261017/steps5_12_01')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
r=dict(boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),uptime=Path('/proc/uptime').read_text(),meminfo=Path('/proc/meminfo').read_text(),fpga_state=Path('/sys/class/fpga_manager/fpga0/state').read_text().strip(),configurations=[],active=[],registers={},temperatures={})
for backup in json.loads((S/'session.json').read_text())['backups']:
    old=Path(backup['backup']);live=Path(backup['source'])
    r['configurations'].append(dict(path=str(live),backup_exists=old.exists(),backup_sha256=sha(old) if old.exists() else None,current_sha256=sha(live),unchanged=old.exists() and sha(old)==sha(live)))
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        comm=(p/'comm').read_text().strip()
        if comm.startswith(('ps_full','ps_board_app','dct_test')):r['active'].append(dict(pid=p.name,comm=comm))
    except OSError:pass
for raw in Path('/sys/bus/iio/devices').glob('iio:device*/in_temp*_raw'):
    stem=str(raw)[:-4]
    try:r['temperatures'][raw.name]=(float(raw.read_text())+float(Path(stem+'_offset').read_text()))*float(Path(stem+'_scale').read_text())/1000
    except (ValueError,OSError):pass
fd=os.open('/dev/mem',os.O_RDONLY|os.O_SYNC)
try:
    for name,addr in [('IOPLL_CTRL',0xff5e0020),('PL0_REF_CTRL',0xff5e00c0),('EMIO31_DATA',0xff0a0054),('EMIO31_DIRM',0xff0a0344),('EMIO31_OEN',0xff0a0348),('PS_AXI_WIDTH',0xfd615000)]:
        with mmap.mmap(fd,4096,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ,offset=addr&~4095) as m:r['registers'][name]=hex(struct.unpack_from('<I',m,addr&4095)[0])
finally:os.close(fd)
r['cpu_frequency']={p.name:p.read_text().strip() for p in Path('/sys/devices/system/cpu/cpufreq/policy0').glob('*') if p.name in ['scaling_cur_freq','scaling_governor','cpuinfo_max_freq']}
env=os.environ.copy();env['PATH']='/usr/sbin:/sbin:/usr/bin:/bin'
for name,cmd in [('dmesg',['dmesg']),('disk',['df','-h']),('network',['ip','addr'])]:
    p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,env=env);r[name]=p.stdout.decode(errors='replace')
r['original_config_unchanged']=all(x['unchanged'] for x in r['configurations'])
out=S/'final_board_evidence.json'
with out.open('x') as f:json.dump(r,f,indent=2)
print(json.dumps(dict(saved=str(out),active=r['active'],config_unchanged=r['original_config_unchanged'])),flush=True)
