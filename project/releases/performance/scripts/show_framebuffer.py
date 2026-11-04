"""Display a reconstruction PPM on the verified 1280x800 RGB565 framebuffer."""
import argparse,mmap,os
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('ppm',type=Path);a=p.parse_args()
b=a.ppm.read_bytes().split(b'\n',3)
if len(b)!=4 or b[0]!=b'P6' or b[2]!=b'255':raise SystemExit('Expected binary P6 PPM')
w,h=map(int,b[1].split());pixels=b[3]
if w!=36 or h!=29 or len(pixels)!=w*h*3:raise SystemExit('Expected 36x29 reconstruction preview')
sysfs=Path('/sys/class/graphics/fb0')
if (sysfs/'virtual_size').read_text().strip()!='1280,800' or (sysfs/'bits_per_pixel').read_text().strip()!='16' or int((sysfs/'stride').read_text())!=2560:raise SystemExit('Framebuffer differs from the verified RGB565 display')
fd=os.open('/dev/fb0',os.O_RDWR)
# Check bit fields through FBIOGET_VSCREENINFO before packing RGB565.
import fcntl,struct
var=bytearray(160);fcntl.ioctl(fd,0x4600,var,True)
if tuple(struct.unpack_from('<II',var,o) for o in (32,44,56))!=((11,5),(5,6),(0,5)):raise SystemExit('Framebuffer channel layout is not RGB565')
frame=bytearray(2560*800);scale=24;x0=(1280-w*scale)//2;y0=(800-h*scale)//2
for y in range(h*scale):
    for x in range(w*scale):
        i=((y//scale)*w+x//scale)*3;r,g,bb=pixels[i:i+3];v=((r>>3)<<11)|((g>>2)<<5)|(bb>>3)
        struct.pack_into('<H',frame,(y+y0)*2560+(x+x0)*2,v)
with mmap.mmap(fd,len(frame)) as m:m[:]=frame
os.close(fd);print('DISPLAYED',a.ppm)
