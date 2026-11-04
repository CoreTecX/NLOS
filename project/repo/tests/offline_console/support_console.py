from pathlib import Path
import sys,json,time,copy,tempfile,dataclasses
REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'tests/offline_dynamic'))
from support import *
for name in ['src/software/meas_norm/temporal','src/software/ui/offline_console','src/software/sd_frame/offline_save','src/software/telemetry/offline_log']:
    sys.path.insert(0,str(REPO/name))
CONSOLE=read_json(REPO/'configs/offline_console.json')
from quality import QualityEstimator,reconstruction_quality
from burst import BurstAccumulator,QualityPipeline
from contracts import InputError
GOOD=dict(valid=True,quality='LOW_SIGNAL',snr_db=2.)
EVENT=dict(valid=True,activity=0.)

def registered(index,x=None,**kw):
    return dataclasses.replace(measurement(index,x,**kw),capture_signature='fixed-synthetic-capture',temporal_scope='SYNTHETIC_DECLARED')

def accumulator(k):return BurstAccumulator(dict(CONSOLE['burst'],k=k))

