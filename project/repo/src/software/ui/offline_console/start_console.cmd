@echo off
setlocal
if exist "D:\psight\preboard_env\Scripts\pythonw.exe" (
  start "" "D:\psight\preboard_env\Scripts\pythonw.exe" -B "%~dp0app.py"
) else (
  py -3 -B "%~dp0app.py"
)
endlocal
