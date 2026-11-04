"""Durable batch recovery, result ownership and explicit fault outcomes."""
from support import *
import unittest,tempfile,os
from unittest.mock import patch
sys.path.insert(0,str(REPO/'src/software/full_model/offline_pipeline'))
sys.path.insert(0,str(REPO/'src/software/ui/offline_console'))
from batch_runner import run_batch_session
from batch_store import WriterLock,atomic_json,atomic_bytes,seal_result,verify_seal,safe_child,file_hash
from batch_gallery import publish_gallery
from contracts import InputError

class BatchRecoveryTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name);self.output=self.root/'batch';self.seen=[]
  cfg=read_json(REPO/'configs/offline_engineering.json');self.config=self.root/'config.json';atomic_json(self.config,cfg)
  self.desc=dict(kind='PFY',file=dict(path='input.pfy',sha256=sha(b'input'),bytes=5),model=dict(path='model.pfm',sha256=sha(b'model'),bytes=5),model_id='a'*32)
  (self.root/'input.pfy').write_bytes(b'input');(self.root/'model.pfm').write_bytes(b'model')
  atomic_json(self.root/'input.json',self.desc);self.manifest=self.root/'manifest.json'
  pin=dict(path='input.json',bytes=(self.root/'input.json').stat().st_size,sha256=file_hash(self.root/'input.json'))
  atomic_json(self.manifest,dict(schema_version=1,items=[dict(id='case'+str(i),input=pin) for i in range(3)]))
 def tearDown(self):self.temp.cleanup()
 def solver(self,args,**kw):
  self.seen.append(str(args.output));args.output.mkdir(parents=True)
  for name in ('result.pfo','result.pfs','reconstruction_native.png','reconstruction_physical.npy','state_acceptance.json'):(args.output/name).write_bytes(name.encode())
  result=dict(status='COMPLETE',state_acceptance=True,reference_acceptance=None,model_sha256=self.desc['model']['sha256'],model_id=self.desc['model_id'],
              measurement_sha256=self.desc['file']['sha256'],numeric_scope='TEST_STATE',output_files={p.name:file_hash(p) for p in args.output.iterdir()})
  atomic_json(args.output/'summary.json',result);return result
 def run_batch(self,**kwargs):
  with patch('batch_runner.execute',side_effect=self.solver):return run_batch_session(self.manifest,self.config,WORKSPACE,self.output,**kwargs)
 def test_order_and_verified_resume_skips_complete(self):
  self.assertTrue(self.run_batch()['all_complete']);self.assertEqual(len(self.seen),3)
  self.assertTrue(self.run_batch(resume=True)['all_complete']);self.assertEqual(len(self.seen),3)
  rows=read_json(self.output/'checkpoint.json')['items'];self.assertEqual([r['id'] for r in rows],['case0','case1','case2'])
 def test_stop_then_resume_only_remaining(self):
  first=self.run_batch(stop_requested=lambda:len(self.seen)>=1);self.assertEqual(first['status'],'INTERRUPTED')
  self.assertTrue(self.run_batch(resume=True)['all_complete']);self.assertEqual(len(self.seen),3)
 def test_interruption_after_commit_recovers_without_duplicate(self):
  def stop(row):raise KeyboardInterrupt()
  with self.assertRaises(KeyboardInterrupt):self.run_batch(on_commit=stop)
  self.assertTrue(self.run_batch(resume=True)['all_complete']);self.assertEqual(len(self.seen),3)
 def test_running_attempt_gets_new_folder(self):
  self.run_batch(stop_requested=lambda:True);cp=read_json(self.output/'checkpoint.json');cp['items'][0].update(status='RUNNING',attempts=1)
  atomic_json(self.output/'checkpoint.json',cp);old=self.output/'item_000000/attempt_001';old.mkdir(parents=True);(old/'partial').write_bytes(b'keep')
  self.run_batch(resume=True);self.assertEqual((old/'partial').read_bytes(),b'keep');self.assertIn('attempt_002',self.seen[0])
 def test_crash_between_directory_creation_and_checkpoint(self):
  self.run_batch(stop_requested=lambda:True);(self.output/'item_000000/attempt_001').mkdir(parents=True)
  self.run_batch(resume=True);self.assertIn('attempt_002',self.seen[0])
 def test_corrupt_completed_result_is_not_skipped_or_displayed(self):
  self.run_batch();row=read_json(self.output/'checkpoint.json')['items'][0]
  (self.output/row['output']/'result.pfo').write_bytes(b'corrupt')
  result=self.run_batch(resume=True);self.assertEqual(result['counts']['FAILED'],1);self.assertEqual(len(self.seen),3)
  row=read_json(self.output/'checkpoint.json')['items'][0];self.assertNotIn('seal',row)
 def test_changed_input_invalidates_completed_results(self):
  self.run_batch();(self.root/'input.pfy').write_bytes(b'WRONG');r=self.run_batch(resume=True)
  self.assertEqual(r['counts']['FAILED'],3);self.assertEqual(len(self.seen),3)
 def test_changed_config_refused_before_work(self):
  self.run_batch();cfg=read_json(self.config);cfg['adapter_version']='changed';atomic_json(self.config,cfg)
  with self.assertRaises(InputError):self.run_batch(resume=True)
  self.assertEqual(len(self.seen),3)
 def test_changed_manifest_refused(self):
  self.run_batch();spec=read_json(self.manifest);spec['items'][0]['id']='new';atomic_json(self.manifest,spec)
  with self.assertRaises(InputError):self.run_batch(resume=True)
 def test_corrupt_descriptor_accounted(self):
  (self.root/'input.json').write_bytes(b'broken');r=self.run_batch();self.assertEqual(r['counts']['FAILED'],3);self.assertEqual(len(self.seen),0)
 def test_solver_failure_does_not_drop_following_items(self):
  original=self.solver
  def solve(args,**kw):
   if 'item_000001' in str(args.output):return dict(status='FAILED',error_code='SOLVER_TIMEOUT')
   return original(args,**kw)
  self.solver=solve;r=self.run_batch();self.assertEqual(r['counts']['COMPLETE'],2);self.assertEqual(r['counts']['FAILED'],1)
 def test_failed_job_retries_only_when_requested(self):
  original=self.solver;self.solver=lambda *a,**k:dict(status='FAILED',error_code='TEST')
  self.run_batch();self.solver=original;self.assertEqual(self.run_batch(resume=True)['counts']['FAILED'],3)
  self.assertTrue(self.run_batch(resume=True,retry_failed=True)['all_complete']);self.assertEqual(len(self.seen),3)
 def test_no_complete_status_without_artifacts(self):
  self.solver=lambda *a,**k:dict(status='COMPLETE');r=self.run_batch();self.assertEqual(r['counts']['FAILED'],3)
 def test_wrong_result_model_rejected(self):
  original=self.solver
  def solve(args,**kw):
   r=original(args,**kw);r['model_id']='b'*32;atomic_json(args.output/'summary.json',r);return r
  self.solver=solve;self.assertEqual(self.run_batch()['counts']['FAILED'],3)
 def test_required_reference_not_silently_omitted(self):
  self.desc['golden']=dict(path='gold.pfg',bytes=1,sha256=sha(b'g'));(self.root/'gold.pfg').write_bytes(b'g')
  atomic_json(self.root/'input.json',self.desc);spec=read_json(self.manifest)
  for row in spec['items']:row['input'].update(bytes=(self.root/'input.json').stat().st_size,sha256=file_hash(self.root/'input.json'))
  atomic_json(self.manifest,spec);self.assertEqual(self.run_batch()['counts']['FAILED'],3)
 def test_display_failure_recorded_and_resume_repairs(self):
  with patch('batch_runner.publish_gallery',side_effect=OSError('simulated display output failure')):r=self.run_batch()
  self.assertTrue(r['all_complete']);self.assertEqual(r['display']['status'],'FAILED')
  self.assertEqual(self.run_batch(resume=True)['display']['status'],'PUBLISHED');self.assertEqual(len(self.seen),3)
 def test_exclusive_writer_and_released_lock(self):
  self.output.mkdir()
  with WriterLock(self.output):
   with self.assertRaises(InputError):
    with WriterLock(self.output):pass
  with WriterLock(self.output):pass
 def test_path_escape_rejected(self):
  with self.assertRaises(InputError):safe_child(self.root,'../elsewhere')
 def test_atomic_checkpoint_write_failure_keeps_previous(self):
  p=self.root/'atomic.json';atomic_json(p,dict(value='old'))
  with patch('batch_store.os.replace',side_effect=OSError('disk error')):
   with self.assertRaises(OSError):atomic_json(p,dict(value='new'))
  self.assertEqual(read_json(p),dict(value='old'));self.assertFalse(list(self.root.glob('*.tmp-*')))
 def test_windows_reader_sharing_retries_without_losing_checkpoint(self):
  if os.name!='nt':self.skipTest('Windows sharing semantics')
  p=self.root/'atomic.json';atomic_json(p,dict(value='old'));original=os.replace;calls=[]
  def replace(src,dst):
   calls.append(src)
   if len(calls)<3:
    error=PermissionError('reader open');error.winerror=5;raise error
   return original(src,dst)
  with patch('batch_store.os.replace',side_effect=replace):atomic_json(p,dict(value='new'))
  self.assertEqual(read_json(p),dict(value='new'));self.assertEqual(len(calls),3)

if __name__=='__main__':unittest.main(verbosity=2)
