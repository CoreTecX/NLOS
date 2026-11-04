$ErrorActionPreference='Stop'
$psRoot=Split-Path $PSScriptRoot -Parent
Set-Location $psRoot
& 'D:\chang\python.exe' scripts/make_rtl_lite.py
if($LASTEXITCODE -ne 0){throw 'RTL harness generation failed'}
& 'D:\CCC\mingw64\bin\g++.exe' -std=c++17 -O3 -ffp-contract=off -fno-fast-math -Wno-unknown-pragmas tests/rtl_vectors.cpp src/reference_gemv.cpp -o bin/rtl_vectors.exe
if($LASTEXITCODE -ne 0){throw 'RTL golden generator compilation failed'}
& '.\bin\rtl_vectors.exe' data/D11_rgb.pfm data/D11_rgb.pfy build/rtl_lite
if($LASTEXITCODE -ne 0){throw 'RTL golden vector generation failed'}
& 'D:\chang\python.exe' scripts/check_rtl_sources.py
if($LASTEXITCODE -ne 0){throw 'RTL source pairing failed'}
Set-Location (Join-Path $psRoot 'build/rtl_lite')
& 'D:\vivado\2025.2\Vivado\bin\xelab.bat' xil_defaultlib.rtl_lite_tb xil_defaultlib.glbl -prj rtl_lite.prj -L xil_defaultlib -L unisims_ver -L xpm -L floating_point_v7_1_21 -L floating_point_v7_0_26 -relax -s rtl_lite -log ../../reports/rtl_lite_elab.log
if($LASTEXITCODE -ne 0){throw 'RTL elaboration failed'}
& 'D:\vivado\2025.2\Vivado\bin\xsim.bat' rtl_lite -tclbatch run.tcl --noieeewarnings -log ../../reports/rtl_lite_sim.log
if($LASTEXITCODE -ne 0){throw 'RTL simulation process failed'}
if(-not (Select-String -Path ../../reports/rtl_lite_sim.log -SimpleMatch 'RTL_LITE_ALL_PASS cases=14')){throw 'RTL simulation did not reach all acceptance checks'}
Set-Location $psRoot
