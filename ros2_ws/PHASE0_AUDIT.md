# Phase 0 repository audit — 2026-10-18

Repository inspected at `main` commit `1fcc189` (`UI tweaks added`). The new
workspace is deliberately isolated at `ros2_ws/`; existing Unity assets and
CAD files are unchanged.

| Area | Finding | Reuse decision |
|---|---|---|
| Unity/Quest | `CSP-SteelArm`, Unity 6000.5.3f1, Meta XR Core/MRUK 205.0.0 and Passthrough Camera access enabled. | Reuse the existing camera access prefab/permission path. |
| Camera networking | `Assets/CameraImagePublisher.cs` uses `ROSConnection`, 10 Hz bounded `AsyncGPUReadback`, RGB8, and the camera timestamp. | Extend this sender in Phase 2; do not add a competing network protocol. |
| YOLO | `YoloBoxVisualizer.cs` consumes `/yolo/detections` and calls the camera publisher's frame cache. | Preserve unchanged. Keep image-topic migration compatible/configurable. |
| Robot/LeRobot | No ROS 2 package, launch file, Python LeRobot program, or SO-101 control source was found in the checked-out repository. | No control code modified. |
| ROS/TF | No `package.xml`, `CMakeLists.txt`, launch file, TF broadcaster, calibration, or camera calibration data was found. | Create independent `ament_python` package. |
| Camera metadata | Current publisher emits only `camera/rgb/image_raw`, frame `quest_camera`; no `CameraInfo` or `PoseStamped`. | Phase 2 must provide physically paired intrinsics/pose. |
| AprilTag | No `apriltag_ros` integration or tag measurement/mount metadata was found. | Keep detector launch opt-in until target Jazzy installation is verified. |

## Environment result

This implementation workstation is Windows and exposes neither `ros2` nor
`colcon`; therefore it cannot truthfully perform an Ubuntu 24.04/Jazzy build
or prove whether `apriltag_ros` is installed. Python syntax compilation and
the dependency-free transform tests were run locally. The required Ubuntu
commands and acceptance checks are in `src/steelarm_calibration/README.md`.
