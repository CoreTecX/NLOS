"""Focused data-contract and no-hardware-by-default tests."""
from pathlib import Path
import contextlib,io,json,tempfile,unittest,sys
# Supports both the standalone reproduction folder and the formal repository.
repo=Path(__file__).resolve().parents[2]
for subdir in ('src/model/full_model','board/full_model'):
    if (repo/subdir).is_dir():sys.path.insert(0,str(repo/subdir))
from unittest.mock import patch,Mock
import numpy as np
from prepare_dataset import preprocess,payload,blob,decode_measurement
from dataset_contract import contained,select_cases,validate_release
import run_dataset

class DatasetTests(unittest.TestCase):
    def test_rggb_and_three_level_averaging(self):
        raw=np.zeros((2016,2016));raw[::2,::2]=12;raw[::2,1::2]=18;raw[1::2,::2]=22;raw[1::2,1::2]=30
        y=preprocess(raw);self.assertEqual(y.shape,(126,126,3));np.testing.assert_array_equal(y,np.broadcast_to([12,20,30],y.shape))
    def test_no_resize_of_wrong_geometry(self):
        with self.assertRaises(ValueError):preprocess(np.ones((1920,1920)))
    def test_reject_negative_raw(self):
        with self.assertRaises(ValueError):preprocess(-np.ones((2016,2016)))
    def test_reject_flat_measurement(self):
        with self.assertRaises(ValueError):payload(np.ones((126,126,3)),np.ones(3))
    def test_reject_overflow(self):
        y=np.ones((126,126,3))*1e300;y[0,0]=0
        with self.assertRaises(ValueError):payload(y,np.ones(3))
    def test_reject_bad_rgb_scales(self):
        y=np.ones((126,126,3));y[0,0]=2
        with self.assertRaises(ValueError):payload(y,[1,0,1])
    def test_column_major_wall_rgb_roundtrip_and_crc(self):
        y=np.arange(126*126*3,dtype=float).reshape(126,126,3);sc=np.array([1.,2.,3.]);mid='12'*16
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'input.pfy';p.write_bytes(blob(2,(126,126),mid,payload(y,sc)))
            identity,decoded,s=decode_measurement(p);self.assertEqual(identity,mid);np.testing.assert_array_equal(decoded,y);np.testing.assert_array_equal(s,sc)
            raw=bytearray(p.read_bytes());raw[-1]^=1;p.write_bytes(raw)
            with self.assertRaises(ValueError):decode_measurement(p)
    def test_reject_parent_and_windows_paths(self):
        root=(Path(tempfile.gettempdir())/'example').resolve()
        for name in ['../outside','D:/outside','data\\input.pfy','../example2/input']:
            with self.subTest(name=name),self.assertRaises(ValueError):contained(root,name)
    def test_reject_unknown_or_unqualified_case(self):
        m={'cases':[{'key':'ok','reference_status':'independent_reference_passed'},{'key':'bad','reference_status':'reference_failed'}]}
        self.assertEqual([r['key'] for r in select_cases(m,all_ready=True)],['ok'])
        for keys in [['missing'],['bad'],['ok','ok']]:
            with self.assertRaises(ValueError):select_cases(m,keys)
    def test_unpinned_release_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t);(root/'release_manifest.json').write_text('{}')
            with self.assertRaises(ValueError):validate_release(root,{'packages':{}})
    def run_mock(self,execute=False):
        row={'key':'sample','reference_status':'independent_reference_passed'}
        m={'cases':[row],'blocked_paper':[{}]};pin={'profile':'p56_100','software_revision':'x','hardware_release':'y'}
        launcher=Mock();launcher.preflight.return_value={}
        launcher.setup_and_load.side_effect=AssertionError('Hardware must not be initialized')
        launcher.dct_selftest.side_effect=AssertionError('ARM must not be run')
        with patch.object(run_dataset,'validate',return_value=(m,{})),patch.object(run_dataset,'validate_release',return_value=('fallback',pin)),patch.object(run_dataset,'sha',return_value='0'*64),patch.object(run_dataset,'load_launcher',return_value=launcher),patch.object(run_dataset.platform,'system',return_value='Windows'),contextlib.redirect_stdout(io.StringIO()):
            args=['--release','D:/Temp/fake','--all-ready']+(['--execute'] if execute else [])
            if execute:
                with self.assertRaisesRegex(RuntimeError,'AArch64 Linux'):run_dataset.main(args)
            else:self.assertEqual(run_dataset.main(args),0)
        launcher.setup_and_load.assert_not_called();launcher.dct_selftest.assert_not_called();launcher.run_case.assert_not_called()
    def test_default_preflight_never_runs_hardware(self):self.run_mock(False)
    def test_execution_on_pc_refused(self):self.run_mock(True)

if __name__=='__main__':unittest.main(verbosity=2)
