# SteelArm AprilTag calibration (Phase 0–1)

This isolated ROS 2 Jazzy `ament_python` package creates the safe TF plumbing for
`quest_world -> quest_camera_optical -> robot_tag_0 -> robot_base`. It sends no
motor command and does not alter the existing YOLO or Leader–Follower code.

## What is implemented

- `camera_pose_tf`: validates and broadcasts the bridge camera pose as the first TF edge.
- `tag_base_tf`: broadcasts the last TF edge **only** after physical values are supplied.
- `calibration_validator`: detects missing/stale/inconsistent TF and optionally writes JSON.
- transform math tests for composition/inversion, optical-axis convention and invalid quaternions.
- Unity `CameraFrameProvider` abstraction: live Quest PCA provider reads documented physical RGB intrinsics, pose and timestamp; Editor mock emits a visibly synthetic checkerboard and is rejected in player builds.
- The existing `CameraImagePublisher` remains the sole sender and now publishes synchronized Image, CameraInfo and PoseStamped messages. It records a capture sequence internally, bounds remembered frames, rejects failed GPU readbacks, and uses the source UTC timestamp rather than receipt time.
- A test-only OpenCV AprilTag-36h11/ID-0 generator and analytic pinhole fixture are provided. `synthetic_apriltag.yaml` is deliberately separate from production values.
- parameters, launch file, and explicit placeholders for tag size/mount measurements.

The Unity bridge uses `/quest/camera/image_raw`, `/quest/camera/camera_info`,
and `/quest/camera/pose`. It sends `rgb8`, flips texture rows to ROS top-left
image order, and uniformly decimates pixels. Its K/P values are scaled from
the Meta PCA sensor intrinsics for that transmitted resolution. It does not
crop or rectify; distortion remains `unknown` until a target-Quest capture
confirms the sensor distortion model. Apply `image_proc` only once that model
is known.

## Coordinate convention

All ROS translations are metres. `T_A_B` maps coordinates from B to A. ROS
camera optical axes are +X right, +Y down, +Z forward. Unity camera local axes
are +X right, +Y up, +Z forward; therefore the optical local vector mapping is
`(x_ros, y_ros, z_ros) = (x_unity, -y_unity, z_unity)`. Unity world is
left-handed and must be basis-converted to the selected right-handed ROS world
frame before publishing; do not reuse headset-center pose as this camera pose.

## Required physical TODOs

1. Print tag36h11 ID 0 and measure its black detection-square edge in metres;
   set `config/apriltag.yaml: tag_size_m`.
2. Rigidly mount it on the SO-101 base and measure `robot_tag_0 -> robot_base`
   translation and orientation; fill `config/tag_mount.yaml`.
3. On the target Ubuntu computer, determine whether Meta's raw image is
   rectified and obtain the intrinsics/distortion for that exact image stream.
   Never use paper dimensions, display-image intrinsics, or zero distortion by assumption.

Current hardware preparation: tag36h11 ID 0 is attached to a rigid box; its
black-code detector corner-to-corner edge is confirmed as 8 cm, so production
`tag_size_m` is set to `0.08 m`. The box-to-robot-base relation is unmeasured, so production
`tag_mount.yaml` intentionally remains empty; neither the tag nor box origin is
assumed to be `robot_base`.

## Ubuntu 24.04 / ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
cd ~/CSP-300-steel-arm/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select steelarm_calibration
source install/setup.bash
python3 -m unittest src/steelarm_calibration/test/test_transforms.py
ros2 launch steelarm_calibration calibration.launch.py
```

Expected before measurements: launch stays running; `tag_base_tf` logs that the
mount is incomplete and deliberately publishes no robot-base TF. After the
Unity bridge publishes its final three topics and `apriltag_ros` is configured
for its installed Jazzy schema, inspect with:

```bash
ros2 topic hz /quest/camera/image_raw
ros2 topic echo --once /quest/camera/camera_info
ros2 topic echo --once /quest/camera/pose
ros2 run tf2_ros tf2_echo quest_world robot_base
ros2 run tf2_tools view_frames
rviz2
```

At 10–15 FPS images, camera info and pose must have the same source timestamp
and frame. A missing tag must be reported as absent/stale, never republished as
a current observation. View the tag from multiple static-robot viewpoints and
compare validator standard deviation; that is repeatability, not ground-truth
accuracy. Use `image_proc` before `apriltag_ros` if the raw image is distorted.

## Phase 2 checklist

- In Unity set the existing publisher's topics and select the real left/right
  PCA component. Editor mock mode is suitable only for transport/UI tests.
- Confirm raw-image orientation, distortion model and the exact deployed Quest
  SDK output. Do not treat `distortion_model: unknown` as a calibrated stream.
- Install/test a Jazzy-compatible `apriltag_ros`; verify its actual parameter,
  detection, and TF names before adding it to the launch file.
- Add the Unity axis visualization only after an independently checked TF is
  available. No robot or Quest hardware validation has been performed here.

## Synthetic / replay validation

Run `python3 scripts/generate_synthetic_tag.py --output tag36h11_id0.png` on
Ubuntu with `python3-opencv`; it uses OpenCV's AprilTag 36h11 dictionary, not a
hand-drawn marker. Warp/project that image with `synthetic_apriltag.yaml` and
feed it to the installed Jazzy `apriltag_ros` input. A valid result must report
ID 0, `robot_tag_0`, and a plausible 1 m pose for the supplied analytic fixture.
This test config is never referenced by the live launch file.

With Jazzy `apriltag_ros` installed, run the complete hardware-free path:

```bash
python3 src/steelarm_calibration/scripts/generate_synthetic_tag.py --output /tmp/synthetic_tag36h11_id0.pgm
ros2 launch steelarm_calibration synthetic_replay.launch.py image_path:=/tmp/synthetic_tag36h11_id0.pgm
ros2 topic echo /detections --once
```

The default generated tag is 160 pixels wide with a 600-pixel focal length and
a 0.10 m synthetic edge, so its fronto-parallel reference pose is approximately
0.375 m. Change `--pixels` to vary this known synthetic depth.
This is an integration test only; it does not publish production mount TF or
validate physical accuracy.

`enable_apriltag:=true` is deliberately disabled by default. On the target
Jazzy computer install `apriltag_ros`, inspect its actual executable and
parameters, then replace the candidate YAML/remaps with that verified contract
and a measured non-zero tag size. This prevents an unverified package variant
or synthetic size from entering a live TF tree.
# USB webcam test (not production calibration)

Attach the webcam to WSL, find its V4L2 path with `ls -l /dev/video*`, then run:

```bash
ros2 launch steelarm_calibration webcam_test.launch.py device:=/dev/video0 fourcc:=MJPG
```

This publishes `/test/webcam/image_raw`, `/test/webcam/camera_info`, and
`/test/webcam/detections`, plus the detection TF
`webcam_optical_test -> webcam_tag_0_test`.  It uses an explicitly
**provisional 70-degree horizontal FOV** only so the detection pipeline can be
exercised.  The detected metric pose is not valid until the Office G1080 is
calibrated with a checkerboard and its real intrinsics replace that estimate.

The webcam test never fills `tag_mount.yaml` and never publishes
`robot_base`; only a measured, rigid tag-to-base transformation can do that.

