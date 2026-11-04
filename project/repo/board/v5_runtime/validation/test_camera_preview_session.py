"""Guard metadata decoration from corrupting camera register restoration data."""
import unittest
from camera_preview_session import annotate_state_record

class RecordIsolation(unittest.TestCase):
 def setUp(self):self.config={'runtime_version':'test','historical_full_resolution_first_frame':'/history/first_frame.raw'}
 def test_register_snapshots_never_gain_state_fields(self):
  for name in ['camera_before.json','camera_active.json','camera_restored.json']:
   registers={'width':2448,'height':2048,'ccp':0};original=dict(registers)
   result=annotate_state_record(name,registers,self.config)
   self.assertEqual(result,original);self.assertEqual(registers,original)
 def test_state_metadata_does_not_mutate_runtime_dictionary(self):
  state={'state':'LIVE','full_resolution_first_frame':'../old/path'}
  result=annotate_state_record('state.json',state,self.config)
  self.assertEqual(result['full_resolution_first_frame'],'/history/first_frame.raw')
  self.assertEqual(state,{'state':'LIVE','full_resolution_first_frame':'../old/path'})
 def test_first_frame_preserves_raw_identity(self):
  frame={'sha256':'rawhash','width':612,'bytes':626688}
  self.assertEqual(annotate_state_record('first_frame.json',frame,self.config),frame)

if __name__=='__main__':unittest.main()
