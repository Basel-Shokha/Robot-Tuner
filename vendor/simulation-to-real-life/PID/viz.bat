@echo off
rem Visualize tracks one after another in a single window.
rem   viz all              every tracks\*.track, in name order
rem   viz NAME [NAME ...]  just these tracks, in this order
rem   viz                  the normal visualizer (view_track from sim_config.txt, loops forever)
rem   viz rep [NAME]       record one run (view_track, or NAME) as an interactive replay in videos\ and open it
rem Each track runs once (finish, crash or max_episode_steps) and the next one loads; N skips ahead.
setlocal
cd /d "%~dp0"
if /i "%~1"=="rep" (
    mingw32-make --no-print-directory record.exe || exit /b 1
    if "%~2"=="" ( "%~dp0record.exe" --open ) else ( "%~dp0record.exe" --track "%~2" --open )
    exit /b
)
mingw32-make --no-print-directory visualizer.exe || exit /b 1

if "%~1"=="" (
    "%~dp0visualizer.exe"
    exit /b
)
if /i "%~1"=="all" (
    "%~dp0visualizer.exe" --all
    exit /b
)

set "LIST=%~1"
:collect
shift
if "%~1"=="" goto run
set "LIST=%LIST%,%~1"
goto collect
:run
"%~dp0visualizer.exe" --tracks "%LIST%"
