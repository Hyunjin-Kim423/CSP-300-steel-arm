# SteelArm: full setup on a new PC

Everything needed to build and run this project on a Windows PC that has nothing installed.
After setup, daily use is one double-click: `CSP-SteelArm/start_steelarm.bat`.

```
Meta Quest app (Unity)  ──USB, adb reverse :10000──►  ROS 2 Jazzy in WSL  ──USB (usbipd)──►  SO-101 arm
  passthrough camera, head pose,                       endpoint, YOLO, AprilTag LOCK,
  object selection, Debug HUD                          object_target, arm_pointer, dashboard
```

---

## 1. Hardware

| Item | Notes |
|---|---|
| Meta Quest 3 / 3S | Passthrough camera access, USB-C data cable |
| SO-101 follower arm | Motor power adapter + USB (CH343 serial, `1a86:55d3`) |
| AprilTag | `tag36h11`, **ID 0**, **8 cm** black square edge, printed flat (matte paper is best) |
| Tag mount | Fixed to the robot base as measured in `ros2_ws/src/steelarm_calibration/config/tag_mount.yaml` |
| Windows 11 PC | NVIDIA GPU recommended (YOLO). Keep the charger plugged in |

Keep cables away from the tag's white border: a partly covered tag is not detected at all.

---

## 2. Windows

1. **Git + Git LFS** (the YOLO model files are in LFS)
   ```powershell
   winget install Git.Git GitHub.GitLFS
   git lfs install
   git clone <repo url> "C:\Git Repositories\CSP-300-steel-arm"
   ```
2. **Unity Hub** → install **Unity 6000.5.3f1** with **Android Build Support** (including OpenJDK and Android SDK & NDK).
3. **Quest developer mode**: in the Meta Horizon phone app, Devices → Developer Mode → on. Connect the headset with USB and allow USB debugging in the headset.
4. **adb** (Android platform-tools): install [platform-tools](https://developer.android.com/tools/releases/platform-tools) (or Meta Quest Developer Hub) and add its folder to the Windows **PATH**. Check: `adb devices` shows the headset as `device`.
5. **Windows Terminal** (preinstalled on Windows 11; otherwise `winget install Microsoft.WindowsTerminal`).
6. **WSL 2 + Ubuntu 24.04**
   ```powershell
   wsl --install -d Ubuntu-24.04
   ```
   Restart if asked, open Ubuntu once and create your user.
7. **usbipd-win** (passes the arm's USB into WSL)
   ```powershell
   winget install usbipd
   ```
   That is all: the first time you start `start_steelarm.bat`, its preflight finds the arm and asks to
   register it with usbipd (one admin prompt, once per arm). After that the dashboard attaches it by itself.
   Manual way, in an admin PowerShell: `usbipd list`, then `usbipd bind --busid <BUSID of the CH343 line>`.

---

## 3. WSL (Ubuntu 24.04)

Open a new Ubuntu terminal (no conda env active), go to the repo and run the setup script:

```bash
cd "/mnt/c/Git Repositories/CSP-300-steel-arm"
bash tools/steelarm/setup_wsl.sh
```

It asks for your sudo password and takes a while the first time. It is safe to run again. It does:

| Step | What |
|---|---|
| ROS 2 Jazzy | apt source + `ros-jazzy-ros-base`, `ros-dev-tools`, `image-proc`, `apriltag-ros`, `cv-bridge`, `image-geometry`, `tf2-ros` |
| `~/.bashrc` | sources `/opt/ros/jazzy` and `~/ros2_ws` (an old source-built ROS line is commented out) |
| `~/ros2_ws` | clones and builds [ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) (`main-ros2`) and [yolo_ros](https://github.com/mgonzs13/yolo_ros) (4.7.1), installs YOLO's Python requirements |
| Python (system) | `feetech-servo-sdk` (`scservo_sdk`, arm bus), `filelock`, and any other missing ultralytics/torch requirement |
| This repo | builds `ros2_ws`: `steelarm_calibration`, `arm_pointer` |
| Serial | adds you to the `dialout` group (close and reopen WSL once) |
| Check | packages, Python modules, `adb.exe` / `usbipd.exe` reachable from WSL, YOLO model file |

Its log is saved to `local/logs/setup_wsl_<date>.log`. Everything at the end should be `OK`.

Notes:
- Do **not** use the conda Python for ROS. LeRobot is **not** needed.
- Never run `lerobot-calibrate` on the shared arm (it rewrites the motors; see `tools/steelarm/README.md`).

---

## 4. Unity app

1. Unity Hub → Add → `CSP-SteelArm/`. The first import takes a while (Meta XR SDK, packages restore by themselves).
2. File → Build Profiles → **Android** → Switch Platform.
3. ROS connection: already set in `Assets/Resources/ROSConnectionPrefab` to **127.0.0.1 : 10000** (USB with `adb reverse`). Nothing to change; check it only if the app never connects (Robotics → ROS Settings, protocol ROS2).
4. Open `Assets/Scenes/SampleScene`, connect the headset, **Build And Run**.

In the app: left controller menu button → Settings → **Debug** tab → turn on *View · Status HUD* to see the PC dashboard in the headset.

---

## 5. Physical setup

- Robot on a flat desk, arm **folded**, motor power on, USB to the PC.
- AprilTag on the base, facing where you stand (the robot's +x side), nothing covering its border.
- Stand in front of the tag, more than 0.5 m from the robot.

---

## 6. First run

1. Double-click **`CSP-SteelArm/start_steelarm.bat`**. A short **preflight** runs first in that window:
   WSL starts, the arm USB is plugged in and registered with usbipd (it asks and registers it if needed),
   and adb sees the headset. Problems are shown with what to do, and you can re-check, skip or quit.
   If two devices with the arm's USB chip are plugged in (e.g. a leader arm), unplug the other one.
2. Choose a mode: `1` follow the headset, `2` point at the selected object, `3` / `4` the same as dry run (the arm never moves). **Start with 3.**
3. Follow the dashboard. Rows that need you show `>>` and what to do:
   - Arm USB: attached automatically. On the first run the arm calibration is read from the motors (read-only) into `local/calibration/`.
   - Headset (adb): `adb reverse` and stay-awake are set automatically.
   - App stream: start the app on the headset.
   - LOCK: look at the AprilTag from the front, 0.4–0.5 m, until `LOCKED`.
4. Modes 1/2: when everything is OK, press **[Y]** to enable the arm. Quit with **[Q]** or **Ctrl+C**: the arm folds first, then torque off.
5. Optional: `start_steelarm.bat logcat` also opens a Unity logcat tab.

---

## 7. Settings: what is shared and what is per PC

| What | Where | Shared (git) |
|---|---|---|
| Package defaults (hand-measured joint values) | `ros2_ws/src/arm_pointer/config/params.yaml` | yes |
| Team arm settings, every mode | `config/steelarm/arm_common.yaml` | yes |
| Team desk-side ready pose (object modes) | `config/steelarm/arm_object.yaml` | yes |
| Tag mount, bridge settings | `ros2_ws/src/steelarm_calibration/config/*.yaml` | yes |
| App settings (menus, Debug switches) | saved on the headset (Settings panel) | no |
| Arm calibration | `local/calibration/my_follower_arm.json` (made automatically) | no |
| PC overrides | `local/arm_local.yaml`, `local/arm_object.yaml` (optional) | no |
| Logs | `local/logs/` | no |

Later files override earlier ones; details in `config/steelarm/README.md`.

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `$'\r': command not found` | A script was checked out with CRLF. `git add --renormalize .` or re-clone (the repo's `.gitattributes` prevents this) |
| Arm USB row stays `>>` | Arm unplugged, or not registered with usbipd (the preflight registers it; `start_steelarm.bat nocheck` skips the preflight) |
| `Could not connect on port` / no motors | Motor power adapter not plugged in (USB alone powers only the USB chip) |
| App stream: nothing received | App not running, or the endpoint restarted: restart the app; check the endpoint log for a connection |
| LOCK never progresses | Tag covered (cables!), too far, too dark. `bash tools/steelarm/capture_frames.sh` saves frames and tests them |
| Arm points to a wrong side after a while | The headset slept/recentered after LOCK: quit and start again |
| YOLO row FAIL with `ModuleNotFoundError` | Run `bash tools/steelarm/setup_wsl.sh` again (installs missing Python packages) |
| PC restarts with a blue screen `0x116` | NVIDIA driver timeout under load: close other GPU apps, update the driver |
