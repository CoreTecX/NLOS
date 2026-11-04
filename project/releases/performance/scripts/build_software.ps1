$ErrorActionPreference='Stop'
Set-Location (Split-Path $PSScriptRoot -Parent)
$psFlags=@('-std=c++17','-O3','-ffp-contract=off','-fno-fast-math','-Wall','-Wextra','-Werror','-Wno-unknown-pragmas','-Wno-unused-label')
& 'D:/CCC/mingw64/bin/g++.exe' @psFlags src/main.cpp src/reference_gemv.cpp -o bin/ps_full_host.exe
if($LASTEXITCODE -ne 0){throw 'Host build failed'}
& 'D:/CCC/mingw64/bin/g++.exe' @psFlags -DPS_TEST_PACKED_HOST src/main.cpp src/full_gemv.cpp src/reference_gemv.cpp -o bin/ps_full_packed_host.exe
if($LASTEXITCODE -ne 0){throw 'Packed host build failed'}
& 'D:/CCC/mingw64/bin/g++.exe' @psFlags tests/gemv_tb.cpp src/full_gemv.cpp src/reference_gemv.cpp -o bin/gemv_tb.exe
if($LASTEXITCODE -ne 0){throw 'Native test build failed'}
& './bin/gemv_tb.exe' | Out-File -Encoding utf8 reports/native_test.log
if($LASTEXITCODE -ne 0){throw 'Native checks failed'}
& 'D:/CCC/mingw64/bin/g++.exe' @psFlags tests/packing_boundaries.cpp -o bin/packing_boundaries.exe
if($LASTEXITCODE -ne 0){throw 'Packing test build failed'}
& './bin/packing_boundaries.exe' | Out-File -Encoding utf8 reports/packing_boundaries.log
if($LASTEXITCODE -ne 0){throw 'Packing boundaries failed'}
& 'D:/vivado/2025.2/gnu/aarch64/nt/aarch64-linux/bin/aarch64-linux-gnu-g++.exe' @psFlags -static -mcpu=cortex-a53 src/main.cpp src/reference_gemv.cpp -o bin/ps_full_aarch64
if($LASTEXITCODE -ne 0){throw 'AArch64 build failed'}
& 'D:/vivado/2025.2/gnu/aarch64/nt/aarch64-linux/bin/aarch64-linux-gnu-g++.exe' @psFlags -static -mcpu=cortex-a53 tests/dct_test.cpp -o bin/dct_test_aarch64
if($LASTEXITCODE -ne 0){throw 'ARM DCT self-test build failed'}
& 'D:/vivado/2025.2/gnu/aarch64/nt/aarch64-linux/bin/aarch64-linux-gnu-g++.exe' @psFlags -static -mcpu=cortex-a53 -DPS_DCT_LEGACY src/main.cpp src/reference_gemv.cpp -o bin/ps_full_legacy_aarch64
if($LASTEXITCODE -ne 0){throw 'ARM legacy fallback build failed'}
Write-Output SOFTWARE_BUILD_PASS
