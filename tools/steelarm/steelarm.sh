#!/usr/bin/env bash
# SteelArm launcher (WSL side). Usually started by CSP-SteelArm/start_steelarm.bat.
# Direct use, from the repo folder:  bash tools/steelarm/steelarm.sh [1-4]   (no number = choose on screen)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if source "$HERE/ros_env.sh"; then
    # Rebuild this repo's ROS packages when their source changed (after a pull, or a fix), so the run uses it.
    STAMP="$REPO/ros2_ws/install/.steelarm_build_stamp"
    if [ ! -f "$STAMP" ] || [ -n "$(find "$REPO/ros2_ws/src" -newer "$STAMP" -type f \( -name '*.py' -o -name '*.yaml' -o -name '*.xml' -o -name 'setup.cfg' \) -print -quit)" ]; then
        echo "Source changed: rebuilding steelarm_calibration and arm_pointer ..."
        if (cd "$REPO/ros2_ws" && colcon build --packages-select steelarm_calibration arm_pointer > "$LOCAL/logs/colcon_build.log" 2>&1); then
            touch "$STAMP"
            source "$REPO/ros2_ws/install/setup.bash"
        else
            echo "!!! build failed, see local/logs/colcon_build.log (starting with the old build)"
            sleep 3
        fi
    fi
    python3 "$HERE/dashboard.py" "$@"
fi
echo; read -r -p "Press Enter to close this window." _
