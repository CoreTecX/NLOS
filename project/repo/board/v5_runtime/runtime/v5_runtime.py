"""Version-checked board operation entry. No camera-to-reconstruction integration."""
from pathlib import Path
import argparse,fcntl,hashlib,importlib.util,json,mmap,os,signal,struct,subprocess,sys,time

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def dump(p,d):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True);q=p.with_suffix(p.suffix+'.tmp');q.write_text(json.dumps(d,indent=2));q.replace(p)
def boot():return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
def alive(pid):
 try:
  text=(Path('/proc')/str(pid)/'stat').read_text();return text[text.rfind(')')+2:].split()[0]!='Z'
 except OSError:return False
def command(pid):
 try:return (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')
 except OSError:return []
def processes():
 rows=[]
 for p in Path('/proc').iterdir():
  if p.name.isdigit():
   try:
    c=(p/'comm').read_text().strip();args=command(p.name)
    if c.startswith(('ps_full_','ps_board_app')) or any(b'run_target.py' in x or b'camera_preview_session.py' in x or b'camera_live_v' in x for x in args):rows.append({'pid':int(p.name),'comm':c,'args':[x.decode(errors='replace') for x in args if x]})
   except OSError:pass
 return rows
def reg(addr):
 f=os.open('/dev/mem',os.O_RDONLY|os.O_SYNC)
 try:
  with mmap.mmap(f,4096,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ,offset=addr&~4095) as m:return struct.unpack_from('<I',m,addr&4095)[0]
 finally:os.close(f)
def run(args,timeout=40):
 p=subprocess.run(args,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,universal_newlines=True,timeout=timeout)
 if p.returncode:raise RuntimeError('Command failed: '+repr(args)+'\n'+p.stdout)
 return p.stdout
def usb():return {str(p):int(p.read_text()) for p in Path('/sys/bus/usb/devices').glob('*:1.0/*-port*/over_current_count')}
def source_check(c):
 for item in c['runtime_sources']:
  if sha(item['path'])!=item['sha256']:raise RuntimeError('Runtime source hash mismatch: '+item['path'])
def preflight(c):
 source_check(c);result={}
 for name,entry in c['releases'].items():
  root=Path(entry['path']);manifest=root/'release_manifest.json'
  if sha(manifest)!=entry['manifest_sha256']:raise RuntimeError('Release manifest mismatch')
  d=read(manifest)
  if d['software_revision']!=c['software_revision'] or d['release_id']!=c['hardware_release'] or d['layout_version']!=int(c['layout_abi'],16):raise RuntimeError('Release binding mismatch')
  result[name]=run(['/usr/bin/python3.7','-B',str(root/'scripts/run_target.py'),'--backend','pl','--case','D11_rgb','--preflight-only'])
 return result
def status(c):
 s=Path(c['state_root']);r={'boot_id':boot(),'monotonic':time.monotonic(),'processes':processes(),'mounts':Path('/proc/mounts').read_text(),'carrier':Path('/sys/class/net/eth0/carrier').read_text().strip(),'usb_overcurrent':usb(),'fpga_state':Path('/sys/class/fpga_manager/fpga0/state').read_text().strip()}
 r['ipv4']=run(['/sbin/ip','-4','addr','show','dev','eth0']);r['free_storage_bytes']=os.statvfs(str(s)).f_bavail*os.statvfs(str(s)).f_frsize
 r['display']={'framebuffer_exists':Path('/dev/fb0').exists(),'connectors':{p.parent.name:p.read_text().strip() for p in Path('/sys/class/drm').glob('card*-*/status')}}
 r['current']=read(s/'current.json') if (s/'current.json').exists() else None
 if r['current'] and Path(r['current'].get('preview_dir','')+'/state.json').is_file():
  ps=read(Path(r['current']['preview_dir'])/'state.json');r['preview']=ps;r['preview_fresh']=ps.get('boot_id')==boot() and alive(ps.get('pid',0)) and time.monotonic()-ps.get('last_state_monotonic',-100)<5
 return r
def stop_preview(c):
 s=Path(c['state_root']);cur=read(s/'current.json') if (s/'current.json').exists() else {}
 if cur.get('boot_id')!=boot() or not cur.get('preview_dir'):return {'state':'NO_MANAGED_PREVIEW_CURRENT_BOOT'}
 out=Path(cur['preview_dir']);j=read(out/'state.json');pid=j['pid']
 if alive(pid):
  args=command(pid)
  if str(Path(c['install_root'])/'camera_preview_session.py').encode() not in args or str(out).encode() not in args:raise RuntimeError('Preview PID identity mismatch')
  (out/'stop').write_text('Requested by versioned runtime entry')
  until=time.monotonic()+25
  while alive(pid) and time.monotonic()<until:time.sleep(.2)
  if alive(pid):raise RuntimeError('Preview did not stop; no forced kill')
 j=read(out/'state.json')
 if j.get('cleanup_errors') or j.get('restored_camera_parameters') is not True or j.get('owned_ipv4_removed') is not True:raise RuntimeError('Preview cleanup incomplete; inspect '+str(out))
 return {'state':j['state'],'restored':True,'output':str(out)}
def prepare(c,profile):
 if processes():raise RuntimeError('Stop active camera/reconstruction before hardware preparation')
 print('V5_STAGE verifying release packages',file=sys.stderr,flush=True)
 result=preflight(c);s=Path(c['state_root']);s.mkdir(parents=True,exist_ok=True)
 diag=c['thread_compat'];assert sha(diag['path'])==diag['sha256'];runs=[]
 for i in range(3):
  j=json.loads(run([diag['path']],30));assert j['passed'] and j['checks']==32;runs.append(j)
 print('V5_STAGE thread diagnostic passed; checking DCT and hardware binding',file=sys.stderr,flush=True)
 root=Path(c['releases'][profile]['path']);j=json.loads(run([str(root/'bin/dct_test_aarch64')],180));assert j['passed'] and j['cases']==3220 and j['bit_exact']
 sys.path.insert(0,str(root/'scripts'));import run_target as rt
 rows=read(root/'data/manifest.json');manifest=rt.preflight('pl',[x for x in rows if x['key']=='D11_rgb'])
 lock=open('/var/lock/psight-full-launch.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 held=None
 try:
  print('V5_STAGE loading matched FPGA image',file=sys.stderr,flush=True)
  held=rt.setup_and_load(manifest)
  value=reg(0xff5e00c0);expected=0x01010a00 if profile=='performance' else 0x01010f00
  assert value&0x013f3f07==expected
 finally:
  if held:held.close()
  lock.close()
 ready={'boot_id':boot(),'profile':profile,'prepared_monotonic':time.monotonic(),'clock_register':hex(value),'preflight':result,'dct':j,'thread_compat':runs,'thread_scope':'diagnostic test binary only; future product threading must be validated','runtime_configuration_sha256':sha(c['_config_path']),'state':'READY_FOR_FILE_RECONSTRUCTION','real_camera_reconstruction_connected':False}
 dump(s/'ready.json',ready);dump(s/('ready_'+boot()+'.json'),ready);return ready
def start_preview(c,fps,duration):
 s=Path(c['state_root']);ready=read(s/'ready.json')
 if ready['boot_id']!=boot():raise RuntimeError('Run prepare for current boot')
 if ready['runtime_configuration_sha256']!=sha(c['_config_path']):raise RuntimeError('Runtime configuration changed; run prepare again')
 if not Path('/dev/fb0').exists():raise RuntimeError('DISPLAY_NOT_READY: check monitor power, input and cable, then retry; camera was not configured')
 if processes():raise RuntimeError('Another acquisition or reconstruction process exists')
 source_check(c)
 out=s/'runs'/('preview_'+boot()[:8]+'_'+str(time.monotonic_ns()));out.parent.mkdir(parents=True,exist_ok=True)
 args=['/usr/bin/python3.7','-B',str(Path(c['install_root'])/'camera_preview_session.py'),'--config',c['_config_path'],'--expect-boot',boot(),'--output',str(out),'--fps',str(fps),'--duration',str(duration)]
 with Path(str(out)+'.log').open('x') as f:p=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 cur={'boot_id':boot(),'preview_pid':p.pid,'preview_dir':str(out),'command':args};dump(s/'current.json',cur)
 until=time.monotonic()+20
 while time.monotonic()<until:
  if (out/'state.json').exists():
   j=read(out/'state.json')
   if j.get('state')=='LIVE' and j.get('complete_frames',0)>=3 and j.get('first_frame_saved') and j.get('framebuffer_readback_passed'):return cur
   if j.get('state')=='ERROR':raise RuntimeError('Preview start failed: '+str(out))
  if p.poll() is not None:raise RuntimeError('Preview exited: '+str(out)+'.log')
  time.sleep(.2)
 raise RuntimeError('Preview startup deadline; inspect process and state before retry')
def shutdown_ready(c):
 stopped=stop_preview(c)
 if processes():raise RuntimeError('Active work remains; refuse shutdown preparation')
 # Refuse to unmount the boot partition while a process is using it.
 users=[]
 for p in Path('/proc').iterdir():
  if not p.name.isdigit() or int(p.name)==os.getpid():continue
  for fd in (p/'fd').glob('*'):
   try:
    target=os.readlink(str(fd))
    if target.startswith('/mnt/boot/') or target=='/mnt/boot':users.append({'pid':p.name,'path':target})
   except OSError:pass
 if users:raise RuntimeError('Boot partition still used: '+repr(users))
 os.sync()
 mounts=Path('/proc/mounts').read_text()
 if any(x.split()[1]=='/mnt/boot' for x in mounts.splitlines()):run(['/bin/umount','/mnt/boot'])
 run(['/bin/mount','-o','remount,ro','/'])
 mounts=Path('/proc/mounts').read_text();root=[x for x in mounts.splitlines() if x.split()[1]=='/'][0]
 assert 'ro' in root.split()[3].split(',') and not any(x.split()[1]=='/mnt/boot' for x in mounts.splitlines())
 os.sync()
 with open('/dev/mmcblk1p2','rb',buffering=0) as f:f.seek(1024);sb=f.read(1024)
 assert struct.unpack_from('<H',sb,56)[0]==0xef53
 needs_recovery=bool(struct.unpack_from('<I',sb,96)[0]&4)
 if needs_recovery:raise RuntimeError('ext4 journal recovery bit remains set after read-only remount')
 return {'boot_id':boot(),'root_readonly':True,'boot_unmounted':True,'ext4_needs_recovery':False,'clean_unmount_state':hex(struct.unpack_from('<H',sb,58)[0]),'preview':stopped,'state':'SAFE_FOR_POWER_COMMAND','poweroff_performed':False}

def main():
 p=argparse.ArgumentParser();p.add_argument('--config',default=str(Path(__file__).with_name('runtime_config.json')));p.add_argument('--expect-boot');p.add_argument('action',choices=['status','preflight','prepare','start-preview','stop-preview','shutdown-ready']);p.add_argument('--profile',choices=['performance','fallback'],default='performance');p.add_argument('--fps',type=float,default=8);p.add_argument('--duration',type=float,default=0);a=p.parse_args()
 c=read(a.config);c['_config_path']=str(Path(a.config).resolve());os.environ['PATH']='/usr/sbin:/sbin:/usr/bin:/bin'
 if a.action not in ['status','preflight'] and a.expect_boot!=boot():raise RuntimeError('Use current boot_id from status as --expect-boot')
 Path(c['state_root']).mkdir(parents=True,exist_ok=True)
 # devtmpfs keeps this lock independent of the root filesystem's shutdown remount.
 lock=open('/dev/psight-v5-operation.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 if a.action=='status':r=status(c)
 elif a.action=='preflight':r=preflight(c)
 elif a.action=='prepare':r=prepare(c,a.profile)
 elif a.action=='start-preview':r=start_preview(c,a.fps,a.duration)
 elif a.action=='stop-preview':r=stop_preview(c)
 else:r=shutdown_ready(c)
 print(json.dumps(r,indent=2),flush=True)
if __name__=='__main__':main()
