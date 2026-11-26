#!/usr/bin/env bash
# Shared environment for the SteelArm tools: `source tools/steelarm/ros_env.sh` (sets REPO, LOCAL and sources ROS).
# Returns non-zero with a message when something basic is missing.
STEELARM_TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$STEELARM_TOOLS/../.." && pwd)"
LOCAL="$REPO/local"                      # machine-specific files and logs (git-ignored)
mkdir -p "$LOCAL/logs"
if [ -n "$CONDA_DEFAULT_ENV" ] && [ "$CONDA_DEFAULT_ENV" != "base" ]; then
    echo "!!! conda env '$CONDA_DEFAULT_ENV' is active: run 'conda deactivate' first"; return 1
fi
# ROS 2 Jazzy: apt install (recommended) or an older source build.
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f ~/ros2_jazzy/install/setup.bash ]; then
    source ~/ros2_jazzy/install/setup.bash
else
    echo "!!! ROS 2 Jazzy not found. Run: bash tools/steelarm/setup_wsl.sh   (see README_allSetup.md)"; return 1
fi
# Third-party workspace (YOLO, ROS TCP Endpoint), then this repo's packages.
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
if [ -f "$REPO/ros2_ws/install/setup.bash" ]; then
    source "$REPO/ros2_ws/install/setup.bash"
else
    echo "!!! this repo's ros2_ws is not built. Run: bash tools/steelarm/setup_wsl.sh"; return 1
fi
# Same ROS_* settings as interactive shells (ROS_DOMAIN_ID etc.).
eval "$(grep -E '^\s*export\s+ROS_' ~/.bashrc 2>/dev/null)"
export PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0
return 0
