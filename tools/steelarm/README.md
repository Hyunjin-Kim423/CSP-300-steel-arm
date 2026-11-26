# tools/steelarm: launcher, dashboard and arm tools

Start everything with **`CSP-SteelArm/start_steelarm.bat`** (double-click). First-time setup: [`README_allSetup.md`](../../README_allSetup.md).

## What runs

```
start_steelarm.bat (Windows)
  ├─ preflight.ps1 (same window): WSL, arm USB (registers it with usbipd if needed), headset (adb)
  └─ one Windows Terminal tab: tools/steelarm/steelarm.sh (WSL)
       └─ ros_env.sh (sources ROS)  →  dashboard.py
            ├─ endpoint      ros_tcp_endpoint            (Quest app <-> ROS, port 10000)
            ├─ yolo          yolo_bringup yolo.launch.py (segmentation for the app)
            ├─ head_target   steelarm_calibration head_target.launch.py alignment:=tag_once  (AprilTag LOCK)
            ├─ object_target object_target.py            (modes 2/4: selected object -> /object_target_point)
            └─ arm           arm_pointer pointer_node    (after [Y], or at once in dry-run modes)
       background: usbipd attach (arm USB), adb reverse + keep the headset awake,
                   first-run arm calibration (record_ranges.py --from-limits, read-only)
       publishes:  /steelarm/status  (std_msgs/String, JSON, 2 Hz) -> the app's Debug HUD
```

Every program logs to `local/logs/session_<date>/<name>.log`. When one fails, its row shows `file : line`.

## Files

| File | What |
|---|---|
| `dashboard.py` | Launcher + status table + status topic for the app |
| `steelarm.sh`, `ros_env.sh` | WSL entry point; ROS environment shared by all tools |
| `object_target.py` | Selected object (app) -> robot frame -> `/object_target_point` |
| `record_ranges.py` | Arm calibration from the motors, read-only (`--from-limits`; the dashboard runs it on first use) |
| `record_pose.py` | Records a ready pose by hand, read-only (`--shared` writes the team file) |
| `capture_frames.py/.sh` | Camera check for the AprilTag: saves frames, tests them for the tag |
| `setup_wsl.sh` | Installs/builds everything in WSL |
| `preflight.ps1` | Windows checks before the dashboard (run by `start_steelarm.bat`; `nocheck` skips it) |

Settings files and their order: [`config/steelarm/README.md`](../../config/steelarm/README.md).

## Adding things

**A status row** (also shown in the app HUD automatically): add a tuple in `Dashboard.rows()`:
`R.append(("My part", "OK" | "FAIL" | "TODO" | "WAIT" | "--", "short note"))`. The HUD shows whatever the rows say.

**A ROS program started with the rest:** in `Dashboard.start_programs()`:
`self.add("my_node", ["ros2", "run", "my_pkg", "my_node"], parse=self.parse_my_node)`.
It gets its own log and an automatic FAIL row with the error line if you add `R.append(self.proc_row("My node", "my_node", "running"))`.
`parse` receives every new log line, for progress text.

**A mode:** add an entry to `MODES` (key, target kind, dry run, title). Today the target kind decides
which programs start (`start_programs`), the arm options (`arm_cmd`) and the target row (`target_state`).
A planned "free mode" (the app's object menu chooses an action: point, grab, wave, ...) will replace this
`if kind == ...` logic with one action dispatcher; the current modes stay as debug modes.

**Something the app should show:** put it in a row (above), or add a field to the JSON in `Dashboard.send_status()`
and read it in `CSP-SteelArm/Assets/Scripts/Settings/DebugHud.cs` (class `Status`).

## Rules we learned the hard way

- Never run `lerobot-calibrate` on the shared arm: it rewrites the motors' offsets and the hand-measured values in
  `arm_pointer/config/params.yaml` stop matching. Use `record_ranges.py --from-limits` (read-only).
- Keep cables away from the AprilTag's white border: a partly covered tag is not detected at all.
- If the app restarts or the headset recenters after LOCK, the lock is wrong: quit and start again.
