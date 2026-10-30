@echo off
rem ============================================================
rem  SteelArm launcher
rem   Tab 1: ROS TCP Endpoint (WSL)
rem   Tab 2: YOLO             (WSL, starts 2s after tab 1)
rem   Tab 3: adb reverse      (Windows adb)
rem   Tab 4: Unity logcat     (Windows adb, headset-side logs)
rem  Uses Windows Terminal tabs; falls back to separate windows if wt is missing.
rem ============================================================

where wt >nul 2>nul
if errorlevel 1 goto fallback

wt -w new new-tab --title "1 ROS Endpoint" --suppressApplicationTitle wsl.exe -e bash -ic "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0\; ec=$?\; echo\; echo '[STOPPED] exit code:' $ec\; exec bash" ^
 ; new-tab --title "2 YOLO" --suppressApplicationTitle wsl.exe -e bash -ic "sleep 2 && ros2 launch yolo_bringup yolo.launch.py model:=yolov8n-seg.pt\; ec=$?\; echo\; echo '[STOPPED] exit code:' $ec\; exec bash" ^
 ; new-tab --title "3 adb reverse" --suppressApplicationTitle cmd /k "adb reverse tcp:10000 tcp:10000 && echo [OK] reverse active: && adb reverse --list || echo [ERROR] adb reverse failed - check the headset is connected (adb devices)" ^
 ; new-tab --title "4 Unity logcat" --suppressApplicationTitle cmd /k "echo Waiting for headset... && adb logcat -s Unity" ^
 ; focus-tab -t 0
goto :eof

:fallback
echo Windows Terminal not found - opening separate windows instead.
start "1 ROS Endpoint" wsl.exe -e bash -ic "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0; ec=$?; echo; echo '[STOPPED] exit code:' $ec; exec bash"
start "2 YOLO" wsl.exe -e bash -ic "sleep 2 && ros2 launch yolo_bringup yolo.launch.py model:=yolov8n-seg.pt; ec=$?; echo; echo '[STOPPED] exit code:' $ec; exec bash"
start "3 adb reverse" cmd /k "adb reverse tcp:10000 tcp:10000 && echo [OK] reverse active: && adb reverse --list || echo [ERROR] adb reverse failed - check the headset is connected (adb devices)"
start "4 Unity logcat" cmd /k "echo Waiting for headset... && adb logcat -s Unity"
