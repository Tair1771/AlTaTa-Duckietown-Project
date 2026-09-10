@echo off
set "DUCK2_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\pythonw.exe"
if exist "%DUCK2_PYTHON%" (
  "%DUCK2_PYTHON%" "%~dp0duck2_companion.py" --straight-crossing-check
) else (
  py -3 "%~dp0duck2_companion.py" --straight-crossing-check
)
