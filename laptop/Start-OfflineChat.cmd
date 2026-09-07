@echo off
set "DUCK2_OFFLINE_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\pythonw.exe"
if exist "%DUCK2_OFFLINE_PYTHON%" (
  "%DUCK2_OFFLINE_PYTHON%" "%~dp0offline_chat.py"
) else (
  py -3 "%~dp0offline_chat.py"
)
