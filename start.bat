@echo off
cd /d "%~dp0"
if exist "dist\Robocon2027.exe" (
    start "" "dist\Robocon2027.exe"
) else if exist ".venv\Scripts\pythonw.exe" (
    start "" ".venv\Scripts\pythonw.exe" app.py
) else (
    python app.py
    if errorlevel 1 pause
)
