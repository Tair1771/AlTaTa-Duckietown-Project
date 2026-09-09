@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-Duck2-GroundTest.ps1" %*
exit /b %ERRORLEVEL%
