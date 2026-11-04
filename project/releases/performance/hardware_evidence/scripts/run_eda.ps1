param([ValidateSet('hls','rtl','build','audit')][string]$Stage='hls')
$ErrorActionPreference='Stop'
# One public clock selection controls HLS, BD and the independent routed audit.
if($env:PS_HW_PROFILE){
    $psRates=@{'p56_100'=100; 'p56_125'=125; 'p56_150'=150}
    if(-not $psRates.ContainsKey($env:PS_HW_PROFILE)){throw 'Unknown hardware profile'}
    $psRate=$psRates[$env:PS_HW_PROFILE]
    if($env:PS_CLOCK_MHZ -and [double]$env:PS_CLOCK_MHZ -ne $psRate){throw 'Conflicting hardware clock'}
    $psPeriod=(1000.0/$psRate).ToString('R',[System.Globalization.CultureInfo]::InvariantCulture)
    if($env:PS_HLS_PERIOD -and [Math]::Abs([double]$env:PS_HLS_PERIOD-[double]$psPeriod) -gt 0.00001){throw 'Conflicting HLS period'}
    $env:PS_CLOCK_MHZ=[string]$psRate
    $env:PS_HLS_PERIOD=$psPeriod
}

Set-Location (Split-Path $PSScriptRoot -Parent)
$psStore='D:/vivado/2025.2/Vivado/data/XilinxTclStore'
$psTclDirs=@(Get-ChildItem "$psStore/support" -Directory)
foreach($psProvider in Get-ChildItem "$psStore/tclapp" -Directory){$psTclDirs+=Get-ChildItem $psProvider.FullName -Directory}
$env:TCLLIBPATH=($psTclDirs | ForEach-Object {$_.FullName.Replace('\','/')}) -join ' '
if($Stage -eq 'hls'){
    & 'D:/vivado/2025.2/Vitis/bin/vitis-run.bat' --mode hls --tcl --input_file scripts/hls.tcl
}elseif($Stage -eq 'rtl'){
    & "$PSScriptRoot/run_rtl_lite.ps1"
}elseif($Stage -eq 'build'){
    & 'D:/vivado/2025.2/Vivado/bin/vivado.bat' -mode batch -nojournal -log reports/direct_build.log -source scripts/build_bd.tcl
}else{
    & 'D:/vivado/2025.2/Vivado/bin/vivado.bat' -mode batch -nojournal -log reports/routed_audit.log -source scripts/check_routed.tcl
}
if($LASTEXITCODE -ne 0){throw "EDA stage $Stage failed"}
