"""One complete frozen D11 board run; separate from real-camera reconstruction."""
from pathlib import Path
import argparse,hashlib,importlib.util,json,os,signal,subprocess,sys,time,traceback
def dump(p,x):
 p=Path(p);q=p.with_suffix(p.suffix+'.tmp');q.write_text(json.dumps(x,indent=2));q.replace(p)
p=argparse.ArgumentParser();p.add_argument('--expect-boot',required=True);p.add_argument('--output',required=True);a=p.parse_args()
boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip();assert boot==a.expect_boot
s=Path(a.output);s.mkdir(parents=True,exist_ok=False);state={'boot_id':boot,'state':'RUNNING','pid':os.getpid(),'scope':'complete frozen file-input D11 readiness check'};dump(s/'state.json',state)
root=Path('/home/root/ps_v5_runtime/r1');sys.path.insert(0,str(root));import v5_runtime as runtime
c=runtime.read(root/'runtime_config.json');c['_config_path']=str(root/'runtime_config.json');runtime.source_check(c)
assert not runtime.processes(),'Stop preview and other reconstruction first'
release=Path(c['releases']['performance']['path']);assert runtime.sha(release/'release_manifest.json')==c['releases']['performance']['manifest_sha256']
env=os.environ.copy();env['PATH']='/usr/sbin:/sbin:/usr/bin:/bin';start=time.monotonic()
try:
 args=['/usr/bin/python3.7','-B',str(release/'scripts/run_target.py'),'--backend','pl','--case','D11_rgb','--schedule','overlap','--export-state','--output',str(s/'results')]
 with (s/'launcher.log').open('x') as f:
  child=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  try:code=child.wait(timeout=300)
  except subprocess.TimeoutExpired:
   os.killpg(child.pid,signal.SIGINT);child.wait(timeout=30);raise RuntimeError('Readiness test timed out; launcher interrupt/reset path invoked')
 assert code==0,(s/'launcher.log').read_text()
 sys.path.insert(0,str(release/'scripts'));import accept_board
 rows=json.loads((release/'data/manifest.json').read_text());row=next(x for x in rows if x['key']=='D11_rgb')
 accept=accept_board.evaluate(row,s/'results/d11_rgb.pfo');assert accept['passed'];dump(s/'numerical_acceptance.json',accept)
 spec=importlib.util.spec_from_file_location('screen_backend',c['camera_backend']['path']);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 import cv2
 rgb=cv2.cvtColor(cv2.imread(str(s/'results/d11_rgb.ppm')),cv2.COLOR_BGR2RGB);display=m.Display()
 try:
  canvas,digest,ok=display.present(rgb,['PenumbraSight / V5-02 / D11 FILE RECONSTRUCTION','New board result / numeric PASS / independent state audit pending']);assert ok
  assert cv2.imwrite(str(s/'screen.png'),cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR));display_record={'framebuffer_sha256':digest,'readback_passed':ok}
 finally:display.close()
 state.update(state='GENERATED_NUMERIC_PASS_STATE_AUDIT_PENDING',seconds_end_to_end=time.monotonic()-start,command=args,numerical_acceptance=accept,display=display_record,clock_register=hex(runtime.reg(0xff5e00c0)),result_sha256={x.name:runtime.sha(x) for x in (s/'results').iterdir() if x.is_file()})
except BaseException:
 state.update(state='FAILED',traceback=traceback.format_exc());raise
finally:dump(s/'state.json',state)
