"""Run local MATLAB without modifying the user's persistent environment."""
import os,subprocess
from pathlib import Path

repo=Path(__file__).resolve().parents[2]
out=repo.parent.parent/'03_任务产出/S8-26_Board_Verified_20261006'
matlab=Path(r'D:\Program Files\MATLAB\R2024b\bin\matlab.exe')
licenses=list((matlab.parent.parent/'licenses').glob('*.lic'))
if len(licenses)!=1:raise RuntimeError('Specify the installed MATLAB license explicitly')
env=os.environ.copy()
for key in ['MATLAB_PREFDIR','USERPROFILE','APPDATA','LOCALAPPDATA','TMP','TEMP']:env[key]=r'D:\Temp'
script_dir=(repo/'board/linux_runtime').as_posix().replace("'","''")
output_dir=out.as_posix().replace("'","''")
command=f"addpath('{script_dir}'); matched_matlab_validation('{output_dir}')"
subprocess.run([str(matlab),'-c',str(licenses[0]),'-nojvm','-batch',command,'-sd',r'D:\Temp','-logfile',str(out/'matlab_validation_20261007.log')],cwd=r'D:\Temp',env=env,check=True,timeout=300)
