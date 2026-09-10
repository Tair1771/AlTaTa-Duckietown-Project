@echo off
set "DUCK2_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%DUCK2_PYTHON%" (
  "%DUCK2_PYTHON%" "%~dp0connect_companion.py"
) else (
  py -3 "%~dp0connect_companion.py"
)
pause
