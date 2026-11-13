@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo .venv not found. Run install.bat first.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -I scripts\sim_app.py %*
if errorlevel 1 pause
