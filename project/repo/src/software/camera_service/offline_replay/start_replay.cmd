@echo off
setlocal
if not defined PSIGHT_REPLAY_PYTHON set "PSIGHT_REPLAY_PYTHON=D:\psight\preboard_env\Scripts\python.exe"
if not exist "%PSIGHT_REPLAY_PYTHON%" (
    echo Python not found. Set PSIGHT_REPLAY_PYTHON to an existing python.exe.
    exit /b 2
)
"%PSIGHT_REPLAY_PYTHON%" -B "%~dp0replay.py" %*
exit /b %errorlevel%
