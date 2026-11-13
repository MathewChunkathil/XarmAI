@echo off
rem One-click installer. Extra switches pass through: install.bat -All / -Ollama / -Whisper / -CpuOnly / -Test / -InstallPython
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" %*
set RC=%ERRORLEVEL%
echo.
pause
exit /b %RC%
