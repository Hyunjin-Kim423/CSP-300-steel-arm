# config/steelarm: team settings for the robot arm

The dashboard (`tools/steelarm/dashboard.py`) starts `arm_pointer` with these files, in this order.
**A later file wins**, and each file only needs the values it changes.

| # | File | Who | Committed | What |
|---|---|---|---|---|
| 1 | `ros2_ws/src/arm_pointer/config/params.yaml` | arm_pointer package | yes | Package defaults, hand-measured joint values (front, ready pose, joint zeros) |
| 2 | `config/steelarm/arm_common.yaml` | team | yes | Every mode (speed limit, ...) |
| 3 | `config/steelarm/arm_object.yaml` | team | yes | Object modes: desk-side ready pose |
| 4 | `local/arm_object.yaml` | this PC | no | Object modes, this PC only (`record_pose.py` without `--shared`) |
| 5 | `local/arm_local.yaml` | this PC | no | Every mode, this PC only (try a value before sharing it) |
| - | command line | dashboard | - | `port`, `dry_run`, `vertical_ik`, `calibration_file`, topic remap |

The arm calibration (`local/calibration/my_follower_arm.json`) is **not** shared: the dashboard reads it from the
motors (read-only) the first time the arm is connected to a PC.

Format of every file:

```yaml
pointer_node:
  ros__parameters:
    name: value
```

To share a value you tested in `local/arm_local.yaml`, move it into a team file and commit.
If your arm or desk is different (a fork, another lab), keep the team files and put your values in `local/`.
