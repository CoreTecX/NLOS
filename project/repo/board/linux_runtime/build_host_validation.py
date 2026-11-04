import subprocess
from pathlib import Path
r=Path(r'D:\学习\26秋嵌入式\PenumbraSight工作区\02_代码仓库\penumbrasight')
cc=r'D:\CCC\mingw64\bin\gcc.exe'; cxx=r'D:\CCC\mingw64\bin\g++.exe'
incs=['board','src/software/include','src/software/fpga_control','src/software/board_app','src/software/sd_frame','src/software/meas_norm','src/fpga/hls']
flags=['-D__USE_MINGW_ANSI_STDIO=1','-O2','-Wall','-Wextra','-Werror']+['-I'+str(r/i) for i in incs]
src=['src/software/board_app/'+s+'.c' for s in ['frame_pipeline','board_top','board_regseq','board_fb','ps_cache']]+['src/software/fpga_control/'+s+'.c' for s in ['fix_core_driver','uio_backend']]+['src/software/meas_norm/meas_norm.c','src/software/sd_frame/sd_frame.c','src/software/model_loader/psm_crc32.c','board/fix_core.c']
objects=[]
for i,s in enumerate(src):
 o=r/'build'/('validation_frame_'+str(i)+'.o'); subprocess.run([cc,'-std=c11',*flags,'-c',str(r/s),'-o',str(o)],check=True);objects.append(str(o))
hls=['ps_fixed_top.cpp','projection_engine/projection_engine_fixed.cpp','lr_operator/lr_operator_fixed.cpp','pdhg_core/pdhg_core_fixed.cpp']
subprocess.run([cxx,'-std=c++17',*flags,'-Wno-unknown-pragmas',str(r/'board/linux_runtime/host_validation.cpp'),*objects,*[str(r/'src/fpga/hls'/s) for s in hls],'-o',str(r/'build/host_validation.exe')],check=True)
