@echo off
set "DUCK2_CHECK_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%DUCK2_CHECK_PYTHON%" (
  "%DUCK2_CHECK_PYTHON%" "%~dp0inspect_robot_setup.py" --connect --save-summary
) else (
  py -3 "%~dp0inspect_robot_setup.py" --connect --save-summary
)
pause
