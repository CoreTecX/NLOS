$ErrorActionPreference='Stop'
Set-Location (Split-Path $PSScriptRoot -Parent)
& "$PSScriptRoot/build_software.ps1"
& "$PSScriptRoot/run_eda.ps1" hls
& 'D:/chang/python.exe' "$PSScriptRoot/audit_contract.py"
if($LASTEXITCODE -ne 0){throw 'Contract audit failed'}
& 'D:/chang/python.exe' "$PSScriptRoot/run_rtl_parallel.py"
if($LASTEXITCODE -ne 0){throw 'RTL acceptance failed'}
& 'D:/chang/python.exe' "$PSScriptRoot/check_shard_sources.py"
if($LASTEXITCODE -ne 0){throw 'RTL shard sources mismatch'}
& 'D:/chang/python.exe' "$PSScriptRoot/run_p56_supply.py" --tag pressure_v4 --hold 12000
if($LASTEXITCODE -ne 0){throw 'Bounded FIFO/backpressure reset validation failed'}
& "$PSScriptRoot/run_eda.ps1" build
& 'D:/chang/python.exe' "$PSScriptRoot/check_physical_sources.py"
if($LASTEXITCODE -ne 0){throw 'Implemented IP differs from verified HLS RTL'}
& 'D:/chang/python.exe' "$PSScriptRoot/check_reset.py"
if($LASTEXITCODE -ne 0){throw 'Reset integration validation failed'}
& "$PSScriptRoot/run_eda.ps1" audit
& 'D:/chang/python.exe' "$PSScriptRoot/check_clock_contract.py"
if($LASTEXITCODE -ne 0){throw 'Hardware clock contract mismatch'}
Write-Output 'BUILD_AND_RTL_PASS; rerun full regression and regenerate release manifest before publishing'
