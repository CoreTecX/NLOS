from pathlib import Path
import re,subprocess,shutil,time,json,sys,fcntl
root=Path('/home/root/ps_dualview_20261030');pointer=root/'current.txt'
startup_lock=open(str(root/'startup.lock'),'a');fcntl.flock(startup_lock,fcntl.LOCK_EX)
current_boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
# Recover an existing live session even if another launch left a stale pointer.
for old in sorted(root.glob('session_*'),key=lambda p:p.stat().st_mtime,reverse=True):
    if not old.is_dir():continue
    try:
        state=json.loads((old/'state.json').read_text())
        if state['boot_id']!=current_boot or state['state']!='LIVE':continue
        if not 0<=time.monotonic()-state.get('last_state_monotonic',0)<5:continue
        __import__('os').kill(state['pid'],0)
        pointer.write_text(str(old));Path('/home/root/ps_fb12_20261029/current_preview.txt').write_text(str(old))
        print('ALREADY_RUNNING',str(old));raise SystemExit(0)
    except (OSError,ValueError,KeyError):continue
boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
ready=root/'pl_ready.json'
if not ready.exists() or json.loads(ready.read_text()).get('boot_id')!=boot:
    rel=Path('/home/root/penumbrasight_app/slots/20261024-nb18-rc1/releases/performance')
    sys.path.insert(0,str(rel/'scripts'));import run_target as launch
    rows=json.loads((rel/'data/manifest.json').read_text());row=next(x for x in rows if x['key']=='D11_rgb')
    guard=open('/var/lock/psight-full-launch.lock','a');fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
    manifest=launch.preflight('pl',[row]);oldlock=launch.setup_and_load(manifest)
    ready.write_text(json.dumps(dict(boot_id=boot,bit_sha256=manifest['files']['bin/full_board_fan_safe.bit']['sha256'],initialized_before_camera=True)))
    if oldlock:oldlock.close()
    guard.close()
src=Path('/home/root/ps_fb15_20261026/tools/fb15_preview.py');s=src.read_text()
s=re.sub("BOOT='[^']+'",'BOOT='+repr(Path('/proc/sys/kernel/random/boot_id').read_text().strip()),s)
config=json.loads((root/'camera_config.json').read_text()) if (root/'camera_config.json').exists() else dict(exposure_us=200000)
exposure=int(config['exposure_us']);assert 50<=exposure<=200000
s=s.replace("change('exposure_us',3000)","change('exposure_us',%d)"%exposure)
if config.get('lighting')=='NATURAL_LIGHT':
    s=s.replace("lo,hi=np.percentile(rgb16,[.5,99.5]);span=max(hi-lo,64.)","lo,hi=0.,4095.;span=4095.")
s=s.replace('class Display:','class LegacyDisplay:',1)
s=s.replace('def main():','from dual_runtime import Session,Display\n\ndef main():',1)
s=s.replace('  display=Display()','  session=Session(out,active[\'exposure_us\'])\n  display=Display()',1)
s=s.replace('    if q.full():',"    session.feed(raw,meta,width,height,'>u2' if active['packet_size']&(1<<29) else '<u2')\n    if q.full():",1)
s=s.replace('     row=motion.process(rgb16,meta)',"     row={'event':'NONE'}")
s=s.replace('     rgb=motion.annotate(rgb,row)','     # Unannotated live camera image.')
p=root/'preview.py';compile(s,str(p),'exec');p.write_text(s);shutil.copy2(str(src.with_name('fb15_motion.py')),str(root/'fb15_motion.py'))
out=root/('session_'+time.strftime('%Y%m%d_%H%M%S')+'_'+__import__('uuid').uuid4().hex[:6]);f=open(str(out)+'.log','w')
q=subprocess.Popen(['/usr/bin/python3','-B',str(p),'--output',str(out),'--fps','4'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
pointer.write_text(str(out));Path('/home/root/ps_fb12_20261029/current_preview.txt').write_text(str(out));print('STARTED',q.pid,str(out))
