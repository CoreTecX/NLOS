"""Exclusive, repeatable AXU9EGB algorithm launch with independent fan control."""
import os, mmap, struct, time, fcntl, subprocess, sys
lock=open('/home/root/ps_board/algorithm.lock','w')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
fd=os.open('/dev/mem',os.O_RDWR|os.O_SYNC)
def mask(addr,bits,value):
 with mmap.mmap(fd,4096,offset=addr&~4095) as m:
  off=addr&4095
  old=struct.unpack_from('<I',m,off)[0]
  struct.pack_into('<I',m,off,(old&~bits)|(value&bits))
mask(0xfd615000,0x300,0)
for addr in [0xfd380000,0xfd390000,0xfd380014,0xfd390014]:mask(addr,3,0)
mask(0xff5e00c0,0x013f3f07,0x01010a02)
# Only reset PL logic; fan is a static OBUF and remains enabled.
mask(0xff0a0344,0x80000000,0x80000000)
mask(0xff0a0348,0x80000000,0x80000000)
mask(0xff0a0054,0x80000000,0)
time.sleep(.01)
mask(0xff0a0054,0x80000000,0x80000000)
time.sleep(.01)
os.close(fd)
print('ALGORITHM_RESET_READY',flush=True)
args=['/home/root/ps_board/ps_board_app_v2','--process-model','/mnt/boot/ps_data/model/model_b.psm',
 '--geometry-hash','0xD3BFE1C0','--uio','auto','--reserved-ddr','dt-no-map',
 '--fbdev','/dev/fb0','--diagnostic-preview','yes','--timeout-ms','60000']
sys.exit(subprocess.call(args+sys.argv[1:]))
