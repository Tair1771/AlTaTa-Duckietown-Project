@echo off
set "DUCK2_PREVIEW_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\pythonw.exe"
if exist "%DUCK2_PREVIEW_PYTHON%" (
  "%DUCK2_PREVIEW_PYTHON%" "%~dp0command_preview_chat.py"
) else (
  py -3 "%~dp0command_preview_chat.py"
)
