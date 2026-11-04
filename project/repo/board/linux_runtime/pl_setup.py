"""Run on target before accessing algorithm AXI registers, no DDR changes."""
import os,mmap,struct
fd=os.open('/dev/mem',os.O_RDWR|os.O_SYNC)
def read(addr):
 page=addr&~4095
 with mmap.mmap(fd,4096,offset=page) as m:return struct.unpack_from('<I',m,addr-page)[0]
def write(addr,value):
 page=addr&~4095
 with mmap.mmap(fd,4096,offset=page) as m:struct.pack_into('<I',m,addr-page,value)
def mask(addr,mask,value):write(addr,(read(addr)&~mask)|(value&mask))
print('AFI_BEFORE',hex(read(0xfd615000)),flush=True)
mask(0xfd615000,0x300,0)
for addr in [0xfd380000,0xfd390000,0xfd380014,0xfd390014]:mask(addr,3,0)
mask(0xff5e00c0,0x013f3f07,0x01010a02)
print('AFI_AFTER',hex(read(0xfd615000)),'PL_CLK',hex(read(0xff5e00c0)),flush=True)
os.close(fd)
