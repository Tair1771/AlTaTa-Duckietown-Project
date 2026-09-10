@echo off
set "DUCK2_BENCH_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\pythonw.exe"
if exist "%DUCK2_BENCH_PYTHON%" (
  "%DUCK2_BENCH_PYTHON%" "%~dp0duck2_companion.py" --bench
) else (
  py -3 "%~dp0duck2_companion.py" --bench
)
