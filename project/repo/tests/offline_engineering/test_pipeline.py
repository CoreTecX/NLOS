"""Meaningful contract, numerical mapping and refusal tests; no hardware access."""
from pathlib import Path
import copy
import json
import os
import struct
import sys
import tempfile
import unittest
import zlib

from fixture import REPO,create,certify,record,save_json
from contracts import FileSource,InputError,Frame,read_json,decode_pfy,encode_pfy,sha,blob,MODEL_BYTES
from inputs import historical_case,historical_raw
from raw_measurement import convert_raw,diagnostics
import numpy as np


class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config=read_json(REPO/'configs/offline_engineering.json')
        cls.workspace=cls.config['workspace_hint']
        cls.temp=tempfile.TemporaryDirectory(prefix='o03_tests_',dir=os.environ.get('O03_TEST_ROOT',tempfile.gettempdir()))
        cls.base=Path(cls.temp.name)
        cls.fixture=cls.base/'fixture'
        cls.desc,cls.cal,cls.expected=create(cls.fixture,cls.config,cls.workspace)
        cls.frame=FileSource().load(cls.desc,cls.fixture)
        cls.historical,cls.rel,_=historical_case(cls.config,cls.workspace,'D11_rgb')

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def fail_code(self,code,call):
        with self.assertRaises(InputError) as ctx:call()
        self.assertEqual(ctx.exception.code,code)

    def converted(self,cal=None,desc=None,frame=None,base=None):
        return convert_raw(frame or self.frame,desc or self.desc,self.cal if cal is None else cal,
                           base or self.fixture,self.desc['model']['sha256'])

    def mutated_cal(self,**fields):
        b=self.base/('change_'+self._testMethodName);b.mkdir()
        c=copy.deepcopy(self.cal);c.update(fields)
        c['arrays']['path']=str(self.fixture/'corrections.npz')
        certify(b,c)
        return c,b

    def mutated_arrays(self,**arrays):
        b=self.base/('change_'+self._testMethodName);b.mkdir()
        with np.load(self.fixture/'corrections.npz') as z:values={k:z[k].copy() for k in z.files}
        values.update(arrays);np.savez(b/'corrections.npz',**values)
        c=copy.deepcopy(self.cal);c['arrays']=record(b/'corrections.npz');certify(b,c)
        return c,b

    def test_historical_pfy_roundtrip_identity(self):
        b=Path(self.historical['file']['path']).read_bytes()
        mid,sc,rgb=decode_pfy(b)
        self.assertEqual(encode_pfy(rgb,sc,mid),b)

    def test_raw_area_against_scalar_oracle(self):
        b,context,_=self.converted()
        _,_,actual=decode_pfy(b)
        np.testing.assert_array_equal(actual,self.expected)
        self.assertEqual(b,(self.fixture/'expected.pfy').read_bytes())
        self.assertEqual(context['calibration_scope'],'SYNTHETIC_TEST_ONLY')

    def test_spatial_difference_independent_edge_enumeration(self):
        diag,actual=diagnostics(self.expected)
        flat=self.expected.reshape(-1,3,order='F')
        ds=max(abs(float(flat[p,c]-(flat[p+1,c] if p+1<len(flat) else 0))) for p in range(len(flat)) for c in range(3))
        edges=[]
        for x in range(126):
            for y in range(125):edges.append((self.expected[y,x]-self.expected[y+1,x])/np.sqrt(2)/ds)
        for x in range(125):
            for y in range(126):edges.append((self.expected[y,x]-self.expected[y,x+1])/np.sqrt(2)/ds)
        np.testing.assert_allclose(actual,edges,rtol=0,atol=1e-15)
        self.assertEqual(diag['legacy_author_ds'],ds)

    def test_original_bytes_immutable(self):
        raw=FileSource.decode_raw(self.frame.data,self.desc['format'])
        with self.assertRaises(ValueError):raw[0,0]=0
        env=self.frame.envelope;env['format']['width']=0
        self.assertEqual(self.frame.envelope['format']['width'],504)

    def test_repeated_frame_new_uid_original_id_unchanged(self):
        source=FileSource();a=source.load(self.desc,self.fixture);b=source.load(self.desc,self.fixture)
        self.assertNotEqual(a.envelope['frame_uid'],b.envelope['frame_uid'])
        self.assertEqual(a.envelope['original_sensor_frame_id'],b.envelope['original_sensor_frame_id'])
        self.assertEqual(b.envelope['replay_index'],'2')

    def test_unknown_time_is_null(self):
        d=historical_raw(self.config,self.workspace,'raw_preview_612x512')
        f=FileSource().load(d,self.workspace)
        self.assertIsNone(f.envelope['camera_tick_frequency_hz'])
        self.assertEqual(f.envelope['camera_timestamp_ticks'],'163076194600')

    def test_two_real_raw_frames_decode_and_require_calibration(self):
        for name in ('raw_full_2448x2048','raw_preview_612x512'):
            d=historical_raw(self.config,self.workspace,name);f=FileSource().load(d,self.workspace)
            self.assertEqual(f.kind,'RAW');self.assertIsNone(d['model_id'])
            self.fail_code('MISSING_CALIBRATION',lambda:convert_raw(f,d,None,self.workspace,None))

    def test_incomplete_frame(self):
        d=copy.deepcopy(self.desc);d['complete']=False
        self.fail_code('INCOMPLETE_FRAME',lambda:FileSource().load(d,self.fixture))

    def test_corrupted_hash(self):
        d=copy.deepcopy(self.desc);d['file']['sha256']='0'*64
        self.fail_code('FILE_HASH',lambda:FileSource().load(d,self.fixture))

    def test_wrong_length(self):
        d=copy.deepcopy(self.desc);d['file']['bytes']-=2
        self.fail_code('FILE_SIZE',lambda:FileSource().load(d,self.fixture))

    def test_packed_raw_not_accepted(self):
        d=copy.deepcopy(self.desc);d['format']['pixel_format']='BayerRG12Packed'
        self.fail_code('RAW_FORMAT',lambda:FileSource().load(d,self.fixture))

    def test_wrong_endian(self):
        d=copy.deepcopy(self.desc);d['format']['dtype']='>u2'
        self.fail_code('RAW_FORMAT',lambda:FileSource().load(d,self.fixture))

    def test_raw_high_bits(self):
        data=bytearray(self.frame.data);struct.pack_into('<H',data,0,4096)
        self.fail_code('RAW_BITS',lambda:FileSource.decode_raw(bytes(data),self.desc['format']))

    def test_stride_mismatch(self):
        fmt=copy.deepcopy(self.desc['format']);fmt['row_stride_bytes']+=2
        self.fail_code('RAW_SIZE',lambda:FileSource.decode_raw(self.frame.data,fmt))

    def test_pfy_crc(self):
        b=bytearray((self.fixture/'expected.pfy').read_bytes());b[-1]^=1
        self.fail_code('BLOB_CRC',lambda:decode_pfy(bytes(b)))

    def test_pfy_model_mismatch(self):
        self.fail_code('MODEL_ID',lambda:decode_pfy((self.fixture/'expected.pfy').read_bytes(),'0'*32))

    def test_pfy_reserved(self):
        b=bytearray((self.fixture/'expected.pfy').read_bytes());b[36]=1
        self.fail_code('BLOB_HEADER',lambda:decode_pfy(bytes(b)))

    def test_pfy_negative(self):
        rgb=self.expected.copy();rgb[0,0,0]=-1
        self.fail_code('MEASUREMENT_RANGE',lambda:encode_pfy(rgb,[1,1,1],self.desc['model_id']))

    def test_pfy_nan(self):
        rgb=self.expected.copy();rgb[0,0,0]=np.nan
        self.fail_code('NONFINITE',lambda:encode_pfy(rgb,[1,1,1],self.desc['model_id']))

    def test_pfy_flat(self):
        self.fail_code('NO_SPATIAL_VARIATION',lambda:encode_pfy(np.ones((126,126,3)),[1,1,1],self.desc['model_id']))

    def test_scale_invalid(self):
        self.fail_code('MEASUREMENT_RANGE',lambda:encode_pfy(self.expected,[1,0,1],self.desc['model_id']))

    def test_model_hash_binding(self):
        self.fail_code('CALIBRATION_MODEL',lambda:convert_raw(self.frame,self.desc,self.cal,self.fixture,'0'*64))

    def test_synthetic_calibration_rejected_for_real_frame(self):
        env=self.frame.envelope;env['source_kind']='REPLAY_SINGLE';f=Frame('RAW',self.frame.data,json.dumps(env))
        self.fail_code('CALIBRATION_SCOPE',lambda:self.converted(frame=f))

    def test_unregistered_geometry(self):
        d=copy.deepcopy(self.desc);d['format']['roi_sensor_offset']=None
        f=FileSource().load(d,self.fixture)
        self.fail_code('SENSOR_GEOMETRY',lambda:self.converted(desc=d,frame=f))

    def test_exposure_mismatch(self):
        d=copy.deepcopy(self.desc);d['capture']['exposure_us']*=2
        f=FileSource().load(d,self.fixture)
        self.fail_code('RADIOMETRY',lambda:self.converted(desc=d,frame=f))

    def test_context_changed_after_frame_validation(self):
        d=copy.deepcopy(self.desc);d['capture']['gain_raw']+=1
        self.fail_code('FRAME_CONTEXT',lambda:self.converted(desc=d))

    def test_stale_calibration_evidence(self):
        c=copy.deepcopy(self.cal);c['green_weights']=[.5,.5]
        self.fail_code('CALIBRATION_EVIDENCE',lambda:self.converted(cal=c))

    def test_partial_support(self):
        c,b=self.mutated_cal(roi_xywh=[2,0,504,504])
        self.fail_code('ROI_SUPPORT',lambda:self.converted(cal=c,base=b))

    def test_negative_dark_correction(self):
        c,b=self.mutated_arrays(dark=np.full((504,504),4096.))
        self.fail_code('NEGATIVE_CORRECTION',lambda:self.converted(cal=c,base=b))

    def test_negative_background(self):
        c,b=self.mutated_arrays(background=np.full((126,126,3),4096.))
        self.fail_code('NEGATIVE_BACKGROUND',lambda:self.converted(cal=c,base=b))

    def test_invalid_required_sample(self):
        valid=np.ones((504,504),dtype=bool);valid[0,0]=False
        c,b=self.mutated_arrays(valid=valid)
        self.fail_code('INVALID_SUPPORT',lambda:self.converted(cal=c,base=b))

    def test_saturation(self):
        data=bytearray(self.frame.data);struct.pack_into('<H',data,0,4095)
        env=self.frame.envelope;env['raw_sha256']=sha(data)
        f=Frame('RAW',bytes(data),json.dumps(env))
        self.fail_code('SATURATED_SUPPORT',lambda:self.converted(frame=f))

    def test_dynamic_mask_not_accepted(self):
        c=copy.deepcopy(self.cal);c['mask_id']='90_PERCENT_MASK'
        self.fail_code('MEASUREMENT_CONTRACT',lambda:self.converted(cal=c))


if __name__=='__main__':
    unittest.main(verbosity=2)
