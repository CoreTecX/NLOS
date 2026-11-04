"""Boot-independent adapter for the frozen, device-specific diagnostic preview."""
from pathlib import Path
import argparse,hashlib,importlib.util,json,struct,sys

def annotate_state_record(path,value,config):
 if Path(path).name!='state.json':return value
 result=dict(value)
 if 'full_resolution_first_frame' in result:
  result['full_resolution_first_frame']=config['historical_full_resolution_first_frame']
  result['full_resolution_reference_scope']='Historical capture; current session raw is first_frame.raw'
 result['runtime_version']=config['runtime_version']
 return result

def main(frame_sink=None):
 p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--expect-boot',required=True);p.add_argument('--output',required=True);p.add_argument('--fps',type=float,default=8);p.add_argument('--duration',type=float,default=0);a=p.parse_args()
 c=json.loads(Path(a.config).read_text());boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
 if boot!=a.expect_boot:raise RuntimeError('Boot changed; inspect status again')
 src=Path(c['camera_backend']['path'])
 if hashlib.sha256(src.read_bytes()).hexdigest()!=c['camera_backend']['sha256']:raise RuntimeError('Camera diagnostic source changed')
 spec=importlib.util.spec_from_file_location('checked_camera_backend',str(src));mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
 original=mod.Control
 class CheckedCamera(original):
  def __init__(self):
   super().__init__()
   try:
    def memory(addr,n):
     b=self.request(0x84,struct.pack('!IHH',addr,0,n));assert len(b)==n+4 and struct.unpack('!I',b[:4])[0]==addr;return b[4:]
    def text(b):return b.split(b'\0',1)[0].decode('ascii')
    mac=struct.pack('!HI',self.read(8)&0xffff,self.read(12))
    identity={'manufacturer':text(memory(0x48,32)),'model':text(memory(0x68,32)),'serial':text(memory(0xd8,16)),'mac':':'.join('%02x'%v for v in mac)}
    assert all(identity[k]==c['camera'][k] for k in identity),identity
    (Path(a.output)/'camera_identity.json').write_text(json.dumps(identity,indent=2))
   except BaseException:self.close();raise
 mod.Control=CheckedCamera;mod.BOOT=boot
 original_write=mod.write_json
 def write_session_record(path,value):
  original_write(path,annotate_state_record(path,value,c))
 mod.write_json=write_session_record
 sys.argv=[str(src),'--output',a.output,'--fps',str(a.fps),'--duration',str(a.duration)]
 mod.main(frame_sink=frame_sink)
 # The historic diagnostic reports failure through state.json rather than exit status.
 state=json.loads((Path(a.output)/'state.json').read_text())
 if state.get('state')!='STOPPED' or state.get('cleanup_errors'):raise RuntimeError('Preview failed; inspect state.json')

if __name__=='__main__':main()
