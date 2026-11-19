@echo off
rem Starts Robot Tuner on this computer and opens it in Chrome (Windows). Close this window to stop.
rem Needs Python 3: https://www.python.org/downloads/ (tick "Add python.exe to PATH" when installing).
cd /d "%~dp0web"
set PORT=8000
set URL=http://localhost:%PORT%/
where py >nul 2>nul && (set PY=py -3) || (set PY=python)
%PY% --version >nul 2>nul && goto run
echo Robot Tuner needs Python 3, which is not installed.
choice /C YN /M "Install Python 3 now with winget (Microsoft package manager)"
if errorlevel 2 (echo Get it from https://www.python.org/downloads/ and tick "Add python.exe to PATH". & pause & exit /b 1)
winget install -e --id Python.Python.3.12 --accept-package-agreements --accept-source-agreements || (echo Install failed. Get it from https://www.python.org/downloads/ & pause & exit /b 1)
echo Python installed. Double-click tune.bat again to start Robot Tuner.
pause
exit /b 0
:run
echo Robot Tuner: %URL%   (close this window to stop)
start "" /min cmd /c "timeout /t 2 >nul & (start chrome %URL% || start msedge %URL% || start %URL%)"
%PY% -m http.server %PORT% --bind 127.0.0.1
