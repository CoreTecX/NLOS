"""Restore the previously verified FPGA image on the board; boot files are untouched."""
from pathlib import Path
import argparse,hashlib,os,platform,fcntl
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--old-bit',type=Path,default=Path('/lib/firmware/algorithm_fan_safe.bit'))
a=p.parse_args()
if platform.system()!='Linux' or platform.machine() not in ('aarch64','arm64'):raise SystemExit('Requires AArch64 Linux board')
expected='901009ae9620037679f692f9afe888e98005f0cc80f716b85ed5d7c7924695e1'
if hashlib.sha256(a.old_bit.read_bytes()).hexdigest()!=expected:raise SystemExit('Old firmware SHA256 differs from the physically verified release')
locks=[]
for path in ('/var/lock/psight-full-launch.lock','/var/lock/psight-full.lock','/home/root/ps_board/algorithm.lock'):
    if Path(path).parent.exists():
        lock=open(path,'a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);locks.append(lock)
for d in Path('/proc').iterdir():
    if d.name.isdigit():
        try:comm=(d/'comm').read_text().strip()
        except OSError:continue
        if comm.startswith('ps_board_app') or comm=='ps_full_aarch64':raise SystemExit('A reconstruction process is still running')
dest=Path('/lib/firmware/ps_previous_verified_20261006.bit')
if dest.exists() and hashlib.sha256(dest.read_bytes()).hexdigest()!=expected:raise SystemExit('Rollback staging name already contains different firmware')
if not dest.exists():dest.write_bytes(a.old_bit.read_bytes())
manager=Path('/sys/class/fpga_manager/fpga0');(manager/'flags').write_text('0');(manager/'firmware').write_text(dest.name)
if (manager/'state').read_text().strip()!='operating':raise SystemExit('FPGA restore failed')
print('PREVIOUS_VERIFIED_FPGA_RESTORED; use the original run_board.py for old rank64 tests')
