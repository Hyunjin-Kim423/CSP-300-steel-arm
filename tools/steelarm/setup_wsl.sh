#!/usr/bin/env bash
# SteelArm WSL setup (Ubuntu 24.04): installs ROS 2 Jazzy and everything the dashboard needs, and builds the workspaces.
# Safe to run again: finished steps are skipped. Asks for your sudo password (apt).
# Run from the repo folder, in a NEW WSL terminal (no conda env):
#   bash tools/steelarm/setup_wsl.sh
# The output is also saved to local/logs/setup_wsl_<date>.log. Details: README_allSetup.md
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
mkdir -p "$REPO/local/logs"
LOG="$REPO/local/logs/setup_wsl_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee "$LOG") 2>&1
FAILED=()
step() { echo; echo "================ $1 ================"; }
fail() { echo "!!! FAILED: $1"; FAILED+=("$1"); }
leave_conda() { command -v conda >/dev/null 2>&1 && for _ in 1 2 3 4; do conda deactivate 2>/dev/null || break; done; }
PIP="python3 -m pip install --user --break-system-packages"

step "0. Info"
date; . /etc/os-release; echo "OS: $PRETTY_NAME   user: $USER   repo: $REPO"
[ "$VERSION_CODENAME" = "noble" ] || echo "!!! ROS 2 Jazzy needs Ubuntu 24.04 (noble). This is $VERSION_CODENAME."
leave_conda

step "1. ROS 2 Jazzy apt source"
if [ -f /opt/ros/jazzy/setup.bash ] || apt-cache policy ros-jazzy-ros-base 2>/dev/null | grep -q Candidate:.*[0-9]; then
    echo "ROS apt source already set"
else
    sudo apt update && sudo apt install -y software-properties-common curl || fail "apt prerequisites"
    sudo add-apt-repository -y universe
    V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.${VERSION_CODENAME}_all.deb" \
        && sudo dpkg -i /tmp/ros2-apt-source.deb || fail "ROS apt source"
fi

step "2. apt packages"
sudo apt update || fail "apt update"
sudo apt install -y ros-jazzy-ros-base ros-dev-tools python3-rosdep \
    ros-jazzy-image-proc ros-jazzy-apriltag-ros ros-jazzy-cv-bridge ros-jazzy-image-geometry ros-jazzy-tf2-ros \
    python3-pip python3-serial python3-opencv git || fail "apt install"

step "3. ~/.bashrc"
cp ~/.bashrc ~/.bashrc.steelarm_backup 2>/dev/null
# An older source-built ROS must not be loaded on top of the apt one.
sed -i -E 's|^([^#]*source[^#]*ros2_jazzy/install/(local_)?setup\.bash.*)$|# (steelarm_setup) \1|' ~/.bashrc
grep -q "^source /opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -q "ros2_ws/install/setup.bash" ~/.bashrc || echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
grep -nE "ros" ~/.bashrc

step "4. ~/ros2_ws: ROS TCP Endpoint + YOLO (yolo_ros)"
source /opt/ros/jazzy/setup.bash || fail "source ROS"
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
[ -d ROS-TCP-Endpoint ] || git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git || fail "clone ROS-TCP-Endpoint"
if [ ! -d yolo_ros ]; then
    # The version we use is 4.7.1; fall back to the default branch if that tag does not exist.
    git clone -b 4.7.1 https://github.com/mgonzs13/yolo_ros.git || git clone https://github.com/mgonzs13/yolo_ros.git || fail "clone yolo_ros"
fi
[ -f yolo_ros/requirements.txt ] && { $PIP -r yolo_ros/requirements.txt || fail "pip yolo_ros requirements"; }
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update >/dev/null && rosdep install --from-paths ~/ros2_ws/src --ignore-src -r -y || echo "(rosdep reported problems; the build below shows if they matter)"
cd ~/ros2_ws && colcon build --symlink-install || fail "colcon build ~/ros2_ws"
source ~/ros2_ws/install/setup.bash

step "5. Python packages for the system Python (the one ROS uses)"
$PIP feetech-servo-sdk filelock || fail "pip feetech-servo-sdk filelock"
# Install any requirement of ultralytics / torch that is still missing, without touching existing packages.
MISSING=$(python3 - <<'PY'
import re
from importlib import metadata
out = []
for dist in ["ultralytics", "torch", "torchvision"]:
    try:
        reqs = metadata.requires(dist) or []
    except metadata.PackageNotFoundError:
        continue
    for r in reqs:
        if ";" in r and "extra" in r.split(";", 1)[1]:
            continue
        name = re.split(r"[ <>=!~\[;(]", r.strip(), 1)[0]
        if name.lower().startswith(("nvidia-", "triton")):
            continue
        try:
            metadata.version(name)
        except metadata.PackageNotFoundError:
            out.append(r.split(";", 1)[0].replace(" ", ""))
print(" ".join(sorted(set(out))))
PY
)
[ -n "$MISSING" ] && { echo "installing missing: $MISSING"; $PIP --no-deps $MISSING || fail "pip missing requirements"; }

step "6. This repo's packages (steelarm_calibration, arm_pointer)"
cd "$REPO/ros2_ws" && colcon build --packages-select steelarm_calibration arm_pointer || fail "colcon build repo"

step "7. Serial port permission"
id -nG | grep -qw dialout && echo "OK   dialout group" || { sudo usermod -aG dialout "$USER" && echo "added to dialout: CLOSE and reopen WSL"; }

step "8. Final check"
source "$HERE/ros_env.sh" || fail "ros_env.sh"
for p in ros_tcp_endpoint yolo_ros yolo_bringup image_proc apriltag_ros cv_bridge tf2_ros steelarm_calibration arm_pointer; do
    ros2 pkg prefix $p >/dev/null 2>&1 && echo "OK   $p" || { echo "MISSING $p"; fail "package $p"; }
done
for m in scservo_sdk serial cv2 ultralytics ultralytics.data.converter; do
    python3 -c "import $m" 2>/dev/null && echo "OK   python $m" || { echo "MISSING python $m"; fail "python $m"; }
done
for w in adb.exe usbipd.exe; do
    command -v $w >/dev/null && echo "OK   $w reachable from WSL" || echo "WARN $w not reachable from WSL (install it on Windows and add it to PATH)"
done
[ -s "$REPO/CSP-SteelArm/yolov8n-seg.pt" ] && echo "OK   yolov8n-seg.pt" || echo "WARN CSP-SteelArm/yolov8n-seg.pt missing (git lfs pull)"

step "Done"
if [ ${#FAILED[@]} -eq 0 ]; then
    echo "All steps finished. Open a NEW terminal, then double-click CSP-SteelArm/start_steelarm.bat"
else
    echo "Failed steps:"; printf '  - %s\n' "${FAILED[@]}"
fi
echo "Log: $LOG"
