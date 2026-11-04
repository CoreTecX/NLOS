"""Transfer a prepared screen by serial and verify framebuffer readback."""
from pathlib import Path
import base64,hashlib,json,zlib
from jtag_transfer import serial,ROOT,REMOTE
directory=ROOT/'display';manifest=json.loads((directory/'screen_manifest.json').read_text('utf8'));data=(directory/'screen_d11_rgb.rgb565.z').read_bytes()
base="import os,json,hashlib,base64\nfrom pathlib import Path\ns=Path('"+REMOTE+"')\nd=s/'display';d.mkdir(exist_ok=True)\np=d/'screen_d11_rgb.rgb565.z'\n"
for offset in range(0,len(data),1350):
    block=data[offset:offset+1350]
    code=base+"b=base64.b64decode("+repr(base64.b64encode(block).decode())+")\nwith p.open('xb' if "+str(offset)+"==0 else 'ab') as f:\n if f.tell()!="+str(offset)+":raise RuntimeError('Display transfer offset mismatch')\n f.write(b)\nprint('PS_TRANSFER_JSON='+json.dumps(dict(bytes=p.stat().st_size)),flush=True)\n"
    response=serial(code,'display_upload_'+str(offset))
    if response['bytes']!=offset+len(block):raise RuntimeError('Display transfer acknowledgement mismatch')
print('DISPLAY_SCREEN_UPLOADED',len(data),flush=True)
code=base+"import zlib,mmap,fcntl,struct\nmeta="+repr(manifest)+"\nblob=p.read_bytes()\nif hashlib.sha256(blob).hexdigest()!=meta['compressed_sha256'] or hashlib.sha256(Path(meta['source']).read_bytes()).hexdigest()!=meta['source_sha256']:raise RuntimeError('Display source identity mismatch')\nraw=zlib.decompress(blob)\nif hashlib.sha256(raw).hexdigest()!=meta['raw_sha256']:raise RuntimeError('Display frame hash mismatch')\nfd=os.open('/dev/fb0',os.O_RDWR)\ntry:\n info=bytearray(160);fcntl.ioctl(fd,0x4600,info,True);v=struct.unpack('<40I',info)\n if tuple(v[:2])!=(1024,768) or v[6]!=16 or list(v[8:11])!=meta['red'] or list(v[11:14])!=meta['green'] or list(v[14:17])!=meta['blue']:raise RuntimeError('Framebuffer format changed')\n stride=int(Path('/sys/class/graphics/fb0/stride').read_text())\n if stride!=2048:raise RuntimeError('Framebuffer stride changed')\n start=v[5]*stride+v[4]*2\n with mmap.mmap(fd,stride*v[3],flags=mmap.MAP_SHARED,prot=mmap.PROT_READ|mmap.PROT_WRITE) as m:\n  backup=d/'original_visible_fb.rgb565'\n  if not backup.exists():backup.write_bytes(m[start:start+len(raw)])\n  m[start:start+len(raw)]=raw\n  check=m[start:start+len(raw)]\n if check!=raw:raise RuntimeError('Framebuffer readback mismatch')\n fcntl.ioctl(fd,0x4611,0)\nfinally:os.close(fd)\nmeta['framebuffer_readback_sha256']=hashlib.sha256(check).hexdigest();meta['display_written']=True;meta['uptime']=Path('/proc/uptime').read_text().strip()\n(d/'display_state.json').write_text(json.dumps(meta,indent=2))\nprint('PS_TRANSFER_JSON='+json.dumps(meta),flush=True)\n"
if len(base64.b64encode(code.encode('utf8')))>3800:
    code="import base64,zlib\nexec(zlib.decompress(base64.b64decode("+repr(base64.b64encode(zlib.compress(code.encode('utf8'),9)).decode())+")))\n"
result=serial(code,'display_present',60)
(directory/'display_state.json').write_text(json.dumps(result,indent=2),encoding='utf8')
print('DISPLAY_VERIFIED',json.dumps(result),flush=True)
