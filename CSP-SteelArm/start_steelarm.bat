@echo off
rem ============================================================
rem  SteelArm launcher
rem   1. Preflight (this window): WSL, robot arm USB (registers it with usbipd if needed), headset (adb)
rem   2. Opens ONE window: the SteelArm dashboard (WSL). It starts the ROS endpoint, YOLO, alignment (LOCK)
rem      and the arm, attaches the arm USB, sets adb reverse and keeps the headset awake.
rem   Every program logs to local\logs\session_<date>\ ; the window shows a status table.
rem
rem   start_steelarm.bat           choose the mode on screen
rem   start_steelarm.bat 2         start mode 2 directly (1 head, 2 object, 3/4 dry run)
rem   start_steelarm.bat logcat    also open a Unity logcat tab
rem   start_steelarm.bat nocheck   skip the preflight
rem ============================================================
setlocal
set "REPO=%~dp0.."
set "ARG="
set "LOGCAT="
set "NOCHECK="
for %%A in (%*) do (
    if /i "%%~A"=="logcat" (set "LOGCAT=1") else if /i "%%~A"=="nocheck" (set "NOCHECK=1") else (set "ARG=%%~A")
)

if not defined NOCHECK (
    powershell -NoProfile -ExecutionPolicy Bypass -File "%REPO%\tools\steelarm\preflight.ps1"
    if errorlevel 1 goto :eof
)

where wt >nul 2>nul
if errorlevel 1 goto fallback

if defined LOGCAT (
    wt -w new new-tab --title "SteelArm" wsl.exe --cd "%REPO%" -e bash tools/steelarm/steelarm.sh %ARG% ; new-tab --title "Unity logcat" cmd /k "adb logcat -s Unity" ; focus-tab -t 0
) else (
    wt -w new new-tab --title "SteelArm" wsl.exe --cd "%REPO%" -e bash tools/steelarm/steelarm.sh %ARG%
)
goto :eof

:fallback
echo Windows Terminal not found - opening a normal window instead.
start "SteelArm" wsl.exe --cd "%REPO%" -e bash tools/steelarm/steelarm.sh %ARG%
if defined LOGCAT start "Unity logcat" cmd /k "adb logcat -s Unity"
