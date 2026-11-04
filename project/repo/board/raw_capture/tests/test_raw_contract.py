from pathlib import Path
import json,sys,tempfile,time,unittest,struct
import numpy as np
base=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(base/'src' if (base/'src').exists() else base))
from raw_contract import *
def params(w=8,h=6):
    p={k:0 for k in IMAGE_KEYS};p.update(width=w,height=h,payload=w*h*2,pixel_format=FORMAT,
      binning=65537,packet_size=1500,acquisition_mode=2,stream_config=0)
    return p
def metadata(t,ts=1,w=8,h=6):
    return dict(complete=True,leader_monotonic=t,complete_monotonic=t+.01,timestamp_ticks=ts,
      block_id=ts,number=ts,width=w,height=h,pfnc=FORMAT,offset_x=0,offset_y=0)
def datagram(kind,packet_id,body,block=1):return struct.pack('!HHI',0,block,(kind<<24)|packet_id)+body
class ContractTests(unittest.TestCase):
    def test_preserves_arbitrary_even_roi_and_signed_endian(self):
        p=params(12,10);p['packet_size']|=1<<29
        self.assertEqual(validate_parameters(p)['dtype'],'>u2')
        with tempfile.TemporaryDirectory() as tmp:
            t=time.monotonic();req=dict(job_id='a'*32,boot_id='test',frames=2,requested_monotonic=t)
            s=CaptureStore(tmp,req,'test',p,{})
            a=np.arange(120,dtype='>u2').reshape(10,12)
            self.assertFalse(s.accept(a.tobytes(),metadata(t-1,w=12,h=10)))
            s.accept(a.tobytes(),metadata(t+.01,w=12,h=10))
            s.accept((a.astype('uint16')+2).astype('>u2').tobytes(),metadata(t+.02,2,12,10))
            info=s.finish(p)
            self.assertEqual(info['state'],'COMPLETE')
            np.testing.assert_array_equal(np.load(s.path/'mean_raw.npy'),a.astype('float32')+1)
            self.assertEqual((s.path/'frame_0001.raw').read_bytes(),a.tobytes())
    def test_stale_future_boot_and_path_escape_rejected(self):
        for req in [dict(job_id='a'*32,boot_id='wrong',frames=1,requested_monotonic=10),
                    dict(job_id='../bad',boot_id='ok',frames=1,requested_monotonic=10),
                    dict(job_id='a'*32,boot_id='ok',frames=1,requested_monotonic=-100),
                    dict(job_id='a'*32,boot_id='ok',frames=1,requested_monotonic=12)]:
            with self.assertRaises(ValueError):validate_request(req,'ok',11)
    def test_duplicate_and_invalid_raw_not_committed(self):
        with tempfile.TemporaryDirectory() as tmp:
            t=time.monotonic();s=CaptureStore(tmp,dict(job_id='b'*32,boot_id='ok',frames=2,requested_monotonic=t),'ok',params(),{})
            raw=np.zeros(48,dtype='<u2').tobytes();s.accept(raw,metadata(t+.1))
            with self.assertRaises(ValueError):s.accept(raw,metadata(t+.2))
            with self.assertRaises(ValueError):s.accept(b'bad',metadata(t+.3,3))
            with self.assertRaises(ValueError):s.accept(np.full(48,65535,dtype='<u2').tobytes(),metadata(t+.4,4))
            self.assertEqual(len(s.records),1)
    def test_settings_change_refuses_complete_result(self):
        with self.assertRaises(ValueError):require_same_parameters(params(),dict(params(),exposure_us=10))
        with self.assertRaises(ValueError):require_same_parameters(params(),dict(params(),frame_rate=100))
    def test_exposure_pipeline_guard(self):
        with tempfile.TemporaryDirectory() as tmp:
            t=time.monotonic();s=CaptureStore(tmp,dict(job_id='d'*32,boot_id='ok',frames=1,requested_monotonic=t,not_before_monotonic=t+2),'ok',params(),{})
            raw=np.zeros(48,dtype='<u2').tobytes()
            self.assertFalse(s.accept(raw,metadata(t+1)))
            self.assertTrue(s.accept(raw,metadata(t+2.1,2)))
            self.assertEqual(s.records[0]['quality']['zero_fraction'],1.)
    def test_missing_packet_discard_and_next_frame_recovery(self):
        p=params();a=FrameAssembler(p)
        leader=struct.pack('!HHQIIIIIHH',0,1,100,FORMAT,8,6,0,0,0,0)
        a.push(datagram(1,0,leader),1)
        a.push(datagram(3,1,b'x'*48),2)
        self.assertIsNone(a.push(datagram(2,3,b''),3));self.assertEqual(a.incomplete,1)
        a.push(datagram(1,0,leader,2),4)
        a.push(datagram(3,2,b'y'*48,2),5);a.push(datagram(3,1,b'x'*48,2),6)
        raw,meta=a.push(datagram(2,3,b'',2),7)
        self.assertEqual(raw,b'x'*48+b'y'*48);self.assertEqual(meta['block_id'],2)
    def test_leader_mismatch_and_conflicting_duplicates(self):
        a=FrameAssembler(params())
        wrong=struct.pack('!HHQIIIIIHH',0,1,100,FORMAT,10,6,0,0,0,0)
        with self.assertRaises(ValueError):a.push(datagram(1,0,wrong),0)
        good=struct.pack('!HHQIIIIIHH',0,1,100,FORMAT,8,6,0,0,0,0)
        a.push(datagram(1,0,good),0);a.push(datagram(3,1,b'x'),1)
        with self.assertRaises(ValueError):a.push(datagram(3,1,b'z'),2)
    def test_summary_retains_full_roi_and_green_average(self):
        raw=np.zeros((2016,2016),dtype='float32');raw[0::2,0::2]=100;raw[0::2,1::2]=200;raw[1::2,0::2]=300;raw[1::2,1::2]=400
        rgb,meta=summary_rgb(raw)
        self.assertEqual(rgb.shape,(126,126,3));np.testing.assert_array_equal(rgb[0,0],[100,250,400])
    def test_wire_zero_offsets_are_recorded_separately_and_must_stay_fixed(self):
        p=dict(params(),offset_x=216,offset_y=16);a=FrameAssembler(p)
        def lead(x,y):return struct.pack('!HHQIIIIIHH',0,1,100,FORMAT,8,6,x,y,0,0)
        a.push(datagram(1,0,lead(0,0)),1)
        a.push(datagram(3,1,b'\0'*96),2)
        raw,m=a.push(datagram(2,2,b''),3)
        self.assertEqual(m['sensor_offset_xy'],[216,16]);self.assertEqual(m['offset_x'],0)
        with self.assertRaises(ValueError):a.push(datagram(1,0,lead(216,16)),4)
        with self.assertRaises(ValueError):FrameAssembler(p).push(datagram(1,0,lead(2,0)),4)
        with tempfile.TemporaryDirectory() as tmp:
            t=time.monotonic();s=CaptureStore(tmp,dict(job_id='c'*32,boot_id='ok',frames=1,requested_monotonic=t),'ok',p,{})
            m['leader_monotonic']=t+.1;s.accept(raw,m)
            self.assertTrue(s.finish(p)['parameters_stable'])
if __name__=='__main__':unittest.main()
