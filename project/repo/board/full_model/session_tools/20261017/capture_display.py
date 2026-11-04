"""Capture actual framebuffer pixels, consistent with the viewer's verified state."""
import base64,hashlib,json,zlib,time
import numpy as np
from PIL import Image
from jtag_transfer import serial,ROOT,REMOTE
code="""import os,mmap,hashlib,base64,zlib,json,time
from pathlib import Path
d=Path('"""+REMOTE+"""')/'display'
for attempt in range(3):
 state=json.loads((d/'live_state.json').read_text());fd=os.open('/dev/fb0',os.O_RDONLY)
 try:
  stride=state.get('stride',2048);height=state.get('height',768);offset=state.get('offset',0)
  with mmap.mmap(fd,offset+stride*height,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ) as m:data=m[offset:offset+stride*height]
 finally:os.close(fd)
 if hashlib.sha256(data).hexdigest()==state['framebuffer_sha256']:break
 time.sleep(.2)
else:raise RuntimeError('Display changed during capture')
print('PS_TRANSFER_JSON='+json.dumps(dict(state=state,frame=base64.b64encode(zlib.compress(data,9)).decode())),flush=True)
"""
r=serial(code,'capture_screen_remote',60);data=zlib.decompress(base64.b64decode(r.pop('frame')))
if hashlib.sha256(data).hexdigest()!=r['state']['framebuffer_sha256']:raise RuntimeError('Frame capture hash mismatch')
p=np.frombuffer(data,dtype='<u2').reshape(r['state'].get('height',768),r['state'].get('stride',2048)//2)[:,:r['state'].get('width',1024)];rgb=np.stack([((p>>11)&31)*255//31,((p>>5)&63)*255//63,(p&31)*255//31],axis=2).astype(np.uint8)
out=ROOT/'display'/('framebuffer_'+str(time.time_ns())+'.png');Image.fromarray(rgb,'RGB').save(out)
out.with_suffix('.json').write_text(json.dumps(r,indent=2),encoding='utf8')
print(json.dumps(dict(file=str(out),state=r['state']),indent=2),flush=True)
