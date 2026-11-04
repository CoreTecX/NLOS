from pathlib import Path
import sys,os
REPO=Path(__file__).resolve().parents[4]
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ.setdefault(key,'1')
for folder in ('src/software/camera_service/production_input','src/software/meas_norm','src/software/meas_norm/camera_motion',
               'src/software/meas_norm/temporal','src/software/full_model/fast_path','src/software/full_model/dual_mode',
               'src/software/model_loader/localization','src/software/sd_frame/offline_save','src/software/telemetry/offline_log'):
    sys.path.insert(0,str(REPO/folder))
