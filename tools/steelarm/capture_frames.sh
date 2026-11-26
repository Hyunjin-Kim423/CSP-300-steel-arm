#!/usr/bin/env bash
# Camera check for the AprilTag step: saves real Quest frames and tests them for the tag (OpenCV detector).
# Needs the ROS endpoint + app running. Do NOT run the dashboard's LOCK at the same time (or quit the dashboard first).
# Run, from the repo folder:  bash tools/steelarm/capture_frames.sh [seconds, default 20]
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/ros_env.sh" || exit 1
LOG="$LOCAL/logs/capture_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee "$LOG") 2>&1
# Our own rectify node, same remaps as calibration.launch.py.
setsid ros2 run image_proc rectify_node --ros-args -r __node:=check_rectify \
    -r image:=/quest/camera/image_raw -r camera_info:=/quest/camera/camera_info -r image_rect:=/quest/camera/image_rect \
    >/tmp/check_rectify.log 2>&1 &
RP=$!
trap 'kill -INT -- -$RP 2>/dev/null' EXIT
sleep 3
echo "-- pre-check: is the Quest streaming? (8 s)"
timeout 8 ros2 topic list 2>&1 | grep -E "^/quest" || echo "   (no /quest topics: the endpoint or the app is not running)"
for t in /quest/head/pose /quest/camera/image_raw /quest/camera/camera_info; do
    r=$(timeout 6 ros2 topic hz "$t" 2>&1 | grep "average rate" | tail -1); echo "   $t: ${r:-(nothing)}"
done
python3 "$HERE/capture_frames.py" "${1:-20}"
echo "-- rectify node output (last 5 lines, sync warnings hidden):"
grep -vE "synchronized|messages received|Synchronized pairs" /tmp/check_rectify.log | tail -5
echo "full log: $LOG"
