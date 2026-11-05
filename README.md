# CSP-300 SteelArm

This project uses a Meta Quest headset to control an SO-101 robot arm.

The Quest sends camera images and position data to ROS 2. We use an AprilTag on the robot base to find where the robot is. After calibration, the robot arm can point at the headset or a target.

## How It Works

```text
Meta Quest (Unity)
    |
    |-- /quest/camera/image_raw
    |-- /quest/camera/camera_info
    |-- /quest/camera/pose
    |
    v
ROS 2 (Ubuntu / WSL)
    |
    |-- apriltag_ros
    |      Finds the AprilTag on the robot
    |
    |-- steelarm_calibration
    |      Connects the Quest and robot coordinates
    |
    |-- arm_pointer
    |      Calculates joint angles and moves the arm
    |
    v
SO-101 Robot Arm
```

1. The Quest sends camera images and position data to ROS 2.
2. `apriltag_ros` finds the AprilTag on the robot base.
3. `tag_align_once` finds the position and rotation between the Quest and the robot.
4. When the terminal shows `LOCKED`, calibration is finished. The Quest does not need to see the tag anymore.
5. `arm_pointer` calculates the joint angles and moves the robot arm.

## Project Files

| Path | Description |
|---|---|
| `CSP-SteelArm/` | Unity project for Meta Quest |
| `CSP-SteelArm/Assets/CameraImagePublisher.cs` | Sends camera images and pose to ROS |
| `CSP-SteelArm/Assets/Calibration/` | Camera code for Quest and Unity Editor |
| `CSP-SteelArm/Assets/RosMessages/` | ROS message files |
| `ros2_ws/src/steelarm_calibration/` | ROS 2 package for AprilTag calibration |

The `arm_pointer` package is not in this repository yet.

### Branch Description
- DY: integrate calibration of coordinates system with Meta Quest camera.
- Jin: AR test. (most of UI)
- gwen-tf2: calibrate coordinates system. test with USB webcam first
- gwen-DOFSim: 2-DOF, 3-DOF simulation

## Requirements

**Hardware**
- Meta Quest with passthrough camera access
- SO-101 robot arm
- USB cable for the robot
- AprilTag (`tag36h11`, ID 0, 8 cm black square edge)

**Software**
- Unity 6000.5.3f1 with Android Build Support
- Meta XR SDK
- Ubuntu 24.04 (WSL is also fine)
- ROS 2 Jazzy
- `ros_tcp_endpoint`
- `apriltag_ros`

This project uses Git LFS. Install Git LFS before cloning the repository.

## Build

### Unity

1. Open `CSP-SteelArm/` in Unity.
2. Set the ROS IP in the Robotics settings.
3. For USB with `adb reverse`, use `127.0.0.1:10000`.
4. Build the app for Android and run it on the Quest.

### ROS 2

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select steelarm_calibration
source install/setup.bash
```

To run the tests:

```bash
python3 -m unittest discover src/steelarm_calibration/test
```

## How to Run

### 1. Connect the Quest

Connect the Quest to your PC with USB.

```bash
adb shell setprop debug.oculus.guardian_pause 1
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close
adb reverse tcp:10000 tcp:10000
```

### 2. Start the ROS server

Open the first terminal.

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Start the Unity app on the Quest and put on the headset.

### 3. Calibrate the robot

Open the second terminal.

```bash
ros2 launch steelarm_calibration head_target.launch.py alignment:=tag_once
```

Look at the AprilTag from the front, about 0.4–0.5 m away.

Keep looking at the tag for about 15 seconds until the terminal shows `LOCKED`.

### 4. Run the robot arm

Open the third terminal.

```bash
ros2 run arm_pointer pointer_node --ros-args \
  --params-file <path to>/params.yaml \
  -p port:=/dev/ttyACM0 \
  -p dry_run:=false \
  -p vertical_ik:=true \
  -p max_speed_deg_s:=15.0
```

Set `dry_run:=true` first to test without moving the robot.

To stop, press Ctrl+C once and wait until the terminal shows `Torque disabled.`

## Safety

- Start with the robot arm folded.
- Stand in front of the tag (+x side).
- Keep the headset more than 0.5 m away from the robot.
- Put the robot on a flat surface.
- If you move the robot and tag together, calibrate again.
- If you move only the tag, update `config/tag_mount.yaml`.

## Current Status

We tested the system with the real SO-101 robot arm.

The AprilTag calibration, coordinate transforms, and robot pointing are working.

Things we still need to do:

- Merge the `Jin` branch because the camera topic names are different.
- Fix the Unity rotation conversion in `CameraImagePublisher.cs`.
- Add `arm_pointer` to this repository.

The calibration package has unit tests, but we do not have automatic tests for the Unity app yet.

## License

MIT License. See [LICENSE](LICENSE).
