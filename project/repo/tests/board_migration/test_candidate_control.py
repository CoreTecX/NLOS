"""Application install failure and rollback must preserve a verified previous slot."""
from pathlib import Path
import importlib.util,json,tempfile,unittest
from unittest.mock import patch
P=Path(__file__).resolve().parents[2]/'board/newboard/candidate_control.py'
spec=importlib.util.spec_from_file_location('candidate_control',P)
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)

class CandidateControlTests(unittest.TestCase):
 def setUp(self):
  self.t=tempfile.TemporaryDirectory();self.root=Path(self.t.name)
  self.src=self.root/'source';self.src.mkdir();(self.src/'candidate_manifest.json').write_text('{}')
  self.dest=self.root/'application'
 def tearDown(self):self.t.cleanup()
 def verifier(self,p):
  manifest=p/'candidate_manifest.json'
  if not manifest.is_file():raise ValueError('incomplete install')
  return dict(passed=True,candidate_id=c.read(manifest).get('candidate_id','v1'))
 def install(self):
  with patch.object(c,'verify',side_effect=self.verifier):return c.install(self.src,self.dest)
 def test_installs_separate_slot_and_no_service_start(self):
  result=self.install();self.assertFalse(result['hardware_accessed'])
  pointer=c.read(self.dest/'current.json');self.assertEqual(pointer['slot'],'slots/v1')
  self.assertFalse(pointer['services_started']);self.assertIsNone(pointer['previous'])
 def test_existing_slot_never_overwritten(self):
  self.install();before=(self.dest/'current.json').read_bytes()
  with self.assertRaises(ValueError):self.install()
  self.assertEqual(before,(self.dest/'current.json').read_bytes())
 def test_copy_failure_keeps_pointer(self):
  self.install();before=(self.dest/'current.json').read_bytes()
  (self.src/'candidate_manifest.json').write_text('{"candidate_id":"v2"}')
  with patch.object(c,'verify',side_effect=self.verifier),patch.object(c.shutil,'copy2',side_effect=OSError('full disk')):
   with self.assertRaises(OSError):c.install(self.src,self.dest)
  self.assertEqual(before,(self.dest/'current.json').read_bytes());self.assertFalse((self.dest/'slots/v2').exists())
 def test_rollback_selects_verified_previous(self):
  self.install();(self.src/'candidate_manifest.json').write_text('{"candidate_id":"v2"}');self.install()
  with patch.object(c,'verify',side_effect=self.verifier):r=c.rollback(self.dest)
  self.assertTrue(r['passed']);self.assertEqual(c.read(self.dest/'current.json')['candidate_id'],'v1')
 def test_corrupt_previous_refuses_rollback(self):
  self.install();(self.src/'candidate_manifest.json').write_text('{"candidate_id":"v2"}');self.install()
  before=(self.dest/'current.json').read_bytes();(self.dest/'slots/v1/candidate_manifest.json').write_text('changed')
  with patch.object(c,'verify',side_effect=self.verifier):
   with self.assertRaises(ValueError):c.rollback(self.dest)
  self.assertEqual(before,(self.dest/'current.json').read_bytes())
 def test_first_install_has_no_fake_factory_rollback(self):
  self.install()
  with patch.object(c,'verify',side_effect=self.verifier):
   with self.assertRaises(ValueError):c.rollback(self.dest)
 def test_source_destination_overlap_rejected(self):
  with patch.object(c,'verify',side_effect=self.verifier):
   with self.assertRaises(ValueError):c.install(self.src,self.src/'nested')
 def test_pointer_path_escape_rejected(self):
  self.install();p=c.read(self.dest/'current.json');p['slot']='../source'
  with self.assertRaises(ValueError):c.slot_from_pointer(self.dest,p)
 def test_windows_absolute_paths_rejected(self):
  for value in ['C:/Windows','/etc/passwd','../escape','slots/../../escape','slots\\v1']:
   with self.assertRaises(ValueError):c.child(self.root,value)
 def test_pointer_tamper_rejected(self):
  self.install();p=c.read(self.dest/'current.json');p['manifest_sha256']='0'*64
  with patch.object(c,'verify',side_effect=self.verifier):
   with self.assertRaises(ValueError):c.slot_from_pointer(self.dest,p)
 def test_concurrent_install_refused(self):
  self.dest.mkdir();(self.dest/'.install.lock').mkdir()
  with self.assertRaises(ValueError):self.install()
  self.assertFalse((self.dest/'current.json').exists())

if __name__=='__main__':unittest.main()
