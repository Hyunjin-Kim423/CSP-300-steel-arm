"""Make the SO-ARM101 follower arm point at a 3D target from /target_point.

Pan: shoulder_pan follows the target (pan = atan2(y, x)).
Vertical (vertical_ik, off by default): after the arm goes to the ready pose,
the arm follows the height of the target. See mapping.py.

Safety: dry_run is on by default. Speed is limited every tick.
Joints stay inside the calibrated limits. On exit, the arm goes back to the start pose before torque off.
"""
import json
import math
import signal
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_pointer.feetech_bus import BusError, FeetechBus
from arm_pointer.mapping import (
    deg_s_to_ticks_per_s,
    in_limits,
    pan_to_raw,
    phi_to_raw,
    elevation_to_arm_angle,
    pointing_pitch_deg,
    raw_to_phi,
    raw_to_rad,
    solve_vertical_ik,
    wrist_aim_phi3,
    wrist_from_shoulder,
    step_toward,
    synced_steps,
    watchdog_state,
)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
# These 3 joints go to the ready pose. The order is same as IK angles (phi1, phi2, phi3).
READY_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]
DEFAULT_CALIB = "~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json"
# If read fails this many times in a row, log the error again.
MAX_READ_FAILS = 10


class PointerNode(Node):
    def __init__(self):
        super().__init__("pointer_node")
        calib_file = self._declare_params()
        self.dry_tag = " [dry-run]" if self.dry_run else ""

        # Max move in one control tick (in servo ticks).
        self.max_step = deg_s_to_ticks_per_s(self.max_speed_deg_s) / self.rate
        # Step size of each joint. Lift, elbow and wrist are scaled to arrive at the same time.
        self.step = {j: self.max_step for j in JOINTS}

        calib = json.loads(Path(calib_file).expanduser().read_text())
        self.ids = [calib[j]["id"] for j in JOINTS]
        self.bus = FeetechBus(self.port, self.ids)
        self.bus.open()
        self.torque_on = False

        start = self.bus.read_positions()
        self.start = {j: float(start[calib[j]["id"]]) for j in JOINTS}
        # goal = the value we send to the servo (it moves step by step).
        self.goal = dict(self.start)
        # target = the place we want the arm to go.
        self.target = dict(self.start)

        self.limits = self._make_limits(calib)
        self.ready = self._make_ready_pose()
        if self.vertical_ik:
            self._check_vertical_setup()

        self.fails = 0
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(PointStamped, "target_point", self.on_target, 10)

        if self.dry_run:
            self.get_logger().warn("DRY RUN: read-only. Torque stays as-is, no goal is ever written.")
        else:
            self.bus.set_acceleration(self.accel)
            # First, set goal to the current position. Then the arm does not jump.
            self.bus.write_goals({calib[j]["id"]: self.start[j] for j in JOINTS})
            self.bus.set_torque(True)
            self.torque_on = True
            self.get_logger().warn("Torque ENABLED, holding startup pose.")
        self.get_logger().info("start pose (raw): " + ", ".join(f"{j}={self.start[j]:.0f}" for j in JOINTS))
        self.create_timer(1.0 / self.rate, self.tick)

    def _declare_params(self):
        """Read all ROS parameters. Returns the calibration file path."""
        p = self.declare_parameter
        self.port = p("port", "/dev/so101_follower").value
        self.dry_run = p("dry_run", True).value
        self.rate = float(p("control_rate_hz", 50.0).value)
        self.max_speed_deg_s = float(p("max_speed_deg_s", 20.0).value)
        self.accel = int(p("servo_acceleration", 20).value)
        self.limit_margin = int(p("limit_margin_ticks", 60).value)
        self.roll_half_range = float(p("wrist_roll_half_range_deg", 45.0).value)
        self.pan_front = float(p("pan_front_raw", 2048).value)
        self.pan_sign = int(p("pan_sign", 1).value)
        self.min_radius = float(p("min_target_radius_m", 0.03).value)
        self.return_on_exit = bool(p("return_to_start_on_exit", True).value)
        self.target_timeout = float(p("target_timeout_s", 1.0).value)
        self.return_after = float(p("return_after_s", 10.0).value)
        self.ready_raw = [int(p(f"ready_{j}_raw", -1).value) for j in READY_JOINTS]

        # Vertical aiming is optional. It is off by default.
        self.vertical_ik = bool(p("vertical_ik", False).value)
        # "wrist_aim": shoulder and elbow stay at the ready pose. Only the wrist aims.
        # "ik": shoulder, elbow and wrist all move. It can not reach some targets behind the arm.
        self.vertical_mode = str(p("vertical_mode", "wrist_aim").value)
        # zero_raw = the raw value when the joint angle is 0 in the URDF.
        # sign = +1 means: higher raw value tilts the far side of the joint DOWN.
        # The lists are in this order: lift, elbow, wrist.
        self.zero_raw = [float(p(f"{n}_zero_raw", 2048).value) for n in ("lift", "elbow", "wrist")]
        self.signs = [int(p(f"{n}_sign", 1).value) for n in ("lift", "elbow", "wrist")]
        # The shoulder axis is this far in front of the pan axis (from the URDF).
        self.shoulder_x = float(p("shoulder_x_m", 0.0304).value)
        # Height of the shoulder axis in the target frame. We need to tune this.
        self.shoulder_z = float(p("shoulder_z_m", 0.0).value)
        # +1: the target is in front of the arm. -1: the target is behind the arm.
        # On this robot, the front of the arm is -x of /target_point. The user stands at +x.
        self.target_side = int(p("target_side", -1).value)
        # Distance from the shoulder to the wrist along the aiming line (ik mode).
        self.reach_wrist = float(p("reach_wrist_m", 0.14).value)
        self.ik_pitch_min = float(p("ik_pitch_min_deg", -40.0).value)
        self.ik_pitch_max = float(p("ik_pitch_max_deg", 50.0).value)

        self.ready_reached = False
        # Time (time.monotonic()) of the last target we accepted.
        self.last_target_time = None
        self.wd_state = "idle"
        return p("calibration_file", "").value or DEFAULT_CALIB

    def _make_limits(self, calib):
        """Joint limits in raw ticks. It is the calibrated range minus a safety margin."""
        limits = {}
        for j in JOINTS:
            lo = calib[j]["range_min"] + self.limit_margin
            hi = calib[j]["range_max"] - self.limit_margin
            if j == "wrist_roll":
                # The wrist roll moves only a little around the start value.
                half = self.roll_half_range / 360.0 * 4096
                lo, hi = max(0.0, self.start[j] - half), min(4095.0, self.start[j] + half)
            if not in_limits(self.start[j], lo, hi):
                self.get_logger().warn(
                    f"{j}: start pose {self.start[j]:.0f} is outside limits [{lo:.0f}, {hi:.0f}], widening")
                lo, hi = min(lo, self.start[j]), max(hi, self.start[j])
            limits[j] = (lo, hi)
        return limits

    def _make_ready_pose(self):
        """The arm goes to the ready pose when the first target comes. Returns {} if it is not set or not valid."""
        if not all(v >= 0 for v in self.ready_raw):
            return {}
        ready = dict(zip(READY_JOINTS, map(float, self.ready_raw)))
        bad = [j for j, v in ready.items() if not in_limits(v, *self.limits[j])]
        if bad:
            names = ", ".join(f"{j}={ready[j]:.0f}" for j in bad)
            self.get_logger().error(f"ready pose ignored, outside limits: {names}")
            return {}
        self.get_logger().info(
            "ready pose (raw): " + ", ".join(f"{j}={v:.0f}" for j, v in ready.items())
            + " -- reached when the first target arrives")
        return ready

    def _check_vertical_setup(self):
        """Turn off vertical aiming if the settings are not valid. Also log the settings."""
        if not self.ready:
            self.get_logger().error("vertical_ik needs a valid ready pose; vertical aiming disabled")
            self.vertical_ik = False
            return
        if self.vertical_mode not in ("wrist_aim", "ik"):
            self.get_logger().error(f"unknown vertical_mode '{self.vertical_mode}'; vertical aiming disabled")
            self.vertical_ik = False
            return
        self.get_logger().warn(
            f"VERTICAL aiming on: mode {self.vertical_mode}, target side {self.target_side:+d}, "
            f"shoulder x {self.shoulder_x:.3f} m z {self.shoulder_z:+.3f} m, "
            f"elevation limits [{self.ik_pitch_min:.0f}, {self.ik_pitch_max:.0f}] deg, "
            f"zero raw {self.zero_raw}, signs {self.signs}")

    def _ready_angles(self):
        """Shoulder and elbow angles (phi1, phi2, in degrees) at the ready pose."""
        phi1 = raw_to_phi(self.ready["shoulder_lift"], self.zero_raw[0], self.signs[0])
        phi2 = raw_to_phi(self.ready["elbow_flex"], self.zero_raw[1], self.signs[1])
        return phi1, phi2

    def on_target(self, msg: PointStamped):
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        # We accept only the base_link frame.
        if msg.header.frame_id not in ("base_link", ""):
            self.get_logger().warn(
                f"frame_id '{msg.header.frame_id}' is not base_link; target ignored", throttle_duration_sec=2.0)
            return
        if not all(math.isfinite(v) for v in (x, y, z)):
            self.get_logger().warn("non-finite target ignored")
            return
        # If the target is on the pan axis, the direction is not clear. So skip it.
        if math.hypot(x, y) < self.min_radius:
            self.get_logger().warn(f"target ({x:.2f},{y:.2f}) too close to base axis; ignored")
            return
        raw = pan_to_raw(x, y, self.pan_front, self.pan_sign)
        lo, hi = self.limits["shoulder_pan"]
        if not in_limits(raw, lo, hi):
            self.get_logger().warn(f"pan target raw {raw:.0f} outside [{lo:.0f}, {hi:.0f}]; ignored")
            return
        self.target["shoulder_pan"] = raw
        self.last_target_time = time.monotonic()
        if self.vertical_ik and self.ready_reached:
            if self.vertical_mode == "wrist_aim":
                self._update_wrist_aim(x, y, z, raw)
            else:
                self._update_ik(x, y, z, raw)
        else:
            self.get_logger().info(
                f"target ({x:.2f},{y:.2f},{z:.2f}) -> pan raw {raw:.0f}{self.dry_tag}",
                throttle_duration_sec=0.4 if self.vertical_ik else 0.0)

    def _update_wrist_aim(self, x, y, z, pan_raw):
        """Shoulder and elbow stay at the ready pose. Only wrist_flex turns the hand to the target.

        The hand points from the WRIST axis to the target.
        The wrist position at the ready pose comes from the URDF model.
        This only changes self.target. If the new wrist angle is out of the limits, we keep the old wrist target.
        """
        phi1, phi2 = self._ready_angles()
        res = wrist_aim_phi3(x, y, z, phi1, phi2, self.shoulder_x, self.shoulder_z,
                             self.ik_pitch_min, self.ik_pitch_max, self.target_side)
        if res is None:
            self.get_logger().warn("wrist aim failed; keeping the last wrist target", throttle_duration_sec=2.0)
            return
        phi3, elev = res
        wrist_raw = phi_to_raw(phi3, self.zero_raw[2], self.signs[2])
        lo, hi = self.limits["wrist_flex"]
        if not (math.isfinite(wrist_raw) and in_limits(wrist_raw, lo, hi)):
            self.get_logger().warn(
                f"wrist raw {wrist_raw:.0f} outside [{lo:.0f}, {hi:.0f}]; keeping the last wrist target",
                throttle_duration_sec=2.0)
            return
        self.target["wrist_flex"] = wrist_raw
        # The step from the ready pose sync can be very small. So reset it, and the hand follows at full speed.
        self.step["wrist_flex"] = self.max_step
        self.get_logger().info(
            f"target x={x:+.2f} y={y:+.2f} z={z:+.2f} | elevation {elev:+.0f} deg | wrist phi {phi3:+.0f} deg"
            f" | raw pan={pan_raw:.0f} wrist={wrist_raw:.0f}{self.dry_tag}",
            throttle_duration_sec=0.4)

    def _update_ik(self, x, y, z, pan_raw):
        """Set lift, elbow and wrist targets from the height angle of the target (planar IK).

        This only changes self.target. The speed limit in tick() moves the arm.
        If IK fails or a joint goes out of the limits, we keep the old vertical target.
        """
        elev = pointing_pitch_deg(x, y, z, self.shoulder_x, self.shoulder_z,
                                  self.ik_pitch_min, self.ik_pitch_max, self.target_side)
        psi = elevation_to_arm_angle(elev, self.target_side)
        phis = solve_vertical_ik(psi, self.reach_wrist)
        if phis is None:
            self.get_logger().warn("vertical IK failed; keeping the last vertical target", throttle_duration_sec=2.0)
            return
        raws = {j: phi_to_raw(phi, zero, sign)
                for j, phi, zero, sign in zip(READY_JOINTS, phis, self.zero_raw, self.signs)}
        bad = [j for j, v in raws.items() if not (math.isfinite(v) and in_limits(v, *self.limits[j]))]
        if bad:
            names = ", ".join(f"{j}={raws[j]:.0f}" for j in bad)
            self.get_logger().warn(
                f"vertical IK result outside joint limits ({names}); keeping the last vertical target",
                throttle_duration_sec=2.0)
            return
        self.target.update(raws)
        self._sync_steps()
        self.get_logger().info(
            f"target x={x:+.2f} y={y:+.2f} z={z:+.2f} | elevation {elev:+.0f} deg"
            f" | phi lift={phis[0]:+.0f} elbow={phis[1]:+.0f} wrist={phis[2]:+.0f} deg"
            f" | raw pan={pan_raw:.0f} lift={raws['shoulder_lift']:.0f}"
            f" elbow={raws['elbow_flex']:.0f} wrist={raws['wrist_flex']:.0f}{self.dry_tag}",
            throttle_duration_sec=0.4)

    def tick(self):
        try:
            pos = self.bus.read_positions()
            self.fails = 0
        except BusError as e:
            self.fails += 1
            if self.fails in (1, MAX_READ_FAILS):
                self.get_logger().error(f"read failed ({self.fails}x): {e}")
            # We can not read the arm now. So we do not write a new goal in this cycle.
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [raw_to_rad(pos[i]) for i in self.ids]
        self.pub.publish(msg)

        self.update_watchdog()
        for j in JOINTS:
            self.goal[j] = step_toward(self.goal[j], self.target[j], self.step[j])
        # When the arm reaches the ready pose, start to follow the height of the target.
        if (self.vertical_ik and not self.ready_reached and self.wd_state == "track"
                and all(self.goal[j] == self.ready[j] for j in READY_JOINTS)):
            self.ready_reached = True
            self.get_logger().info("ready pose reached: now following the target's height")
        if self.dry_run:
            self._log_dry_run(pos[self.ids[0]])
        else:
            try:
                self.bus.write_goals({i: self.goal[j] for i, j in zip(self.ids, JOINTS)})
            except BusError as e:
                self.get_logger().error(f"write failed: {e}")

    def _log_dry_run(self, pan_now_raw):
        pan = "shoulder_pan"
        self.get_logger().info(
            f"[dry-run] {self.wd_state}: pan now {self.raw_to_pan_deg(pan_now_raw):+.1f} deg, "
            f"goal {self.raw_to_pan_deg(self.goal[pan]):+.1f} deg, "
            f"target {self.raw_to_pan_deg(self.target[pan]):+.1f} deg",
            throttle_duration_sec=1.0)

    def raw_to_pan_deg(self, raw):
        """Pan angle in the target frame from a raw servo value (counter-clockwise from above, 0 = front)."""
        return (raw - self.pan_front) / self.pan_sign * 360.0 / 4096.0

    def update_watchdog(self):
        """If targets stop, hold the pan. If it is long, go back to the start pose (with speed limit)."""
        elapsed = None if self.last_target_time is None else time.monotonic() - self.last_target_time
        state = watchdog_state(elapsed, self.target_timeout, self.return_after)
        if state == self.wd_state:
            return
        self.wd_state = state
        if state == "hold":
            self.target["shoulder_pan"] = self.goal["shoulder_pan"]
            if self.vertical_ik and self.ready_reached:
                # Hold the vertical pose also.
                for j in READY_JOINTS:
                    self.target[j] = self.goal[j]
            self.get_logger().warn(f"no valid target for {self.target_timeout:.1f}s: holding current pan")
        elif state == "return":
            self.target = dict(self.start)
            self.ready_reached = False
            self._sync_steps()
            self.get_logger().warn(f"no valid target for {self.return_after:.1f}s: returning to startup pose")
        elif state == "track":
            # After "hold", vertical aiming sets the new targets by itself. So skip the ready pose.
            if self.ready and not (self.vertical_ik and self.ready_reached):
                self.target.update(self.ready)
                self._sync_steps()
            self.get_logger().info("target received: tracking" + (" (moving to ready pose)" if self.ready else ""))

    def _sync_steps(self):
        """Change the speed of lift, elbow and wrist. They reach their targets at the same time."""
        self.step.update(synced_steps(
            {j: self.goal[j] for j in READY_JOINTS}, {j: self.target[j] for j in READY_JOINTS}, self.max_step))

    def safe_shutdown(self):
        try:
            if self.torque_on:
                self._ramp_to_start()
                self.bus.set_torque(False)
                self.get_logger().info("Torque disabled.")
        except KeyboardInterrupt:
            self.get_logger().warn("Second Ctrl+C: torque off NOW (arm may drop).")
            try:
                self.bus.set_torque(False)
            except BusError as e:
                self.get_logger().error(f"torque off failed: {e}")
        except BusError as e:
            self.get_logger().error(f"shutdown error: {e}")
        finally:
            self.bus.close()

    def _ramp_to_start(self):
        if not self.return_on_exit:
            return
        self.get_logger().info("Returning to startup pose before torque off (Ctrl+C again to cut torque now)...")
        # Stop after 30 seconds.
        deadline = time.monotonic() + 30.0
        self.target = dict(self.start)
        self._sync_steps()
        while time.monotonic() < deadline:
            done = True
            for j in JOINTS:
                self.goal[j] = step_toward(self.goal[j], self.start[j], self.step[j])
                done = done and self.goal[j] == self.start[j]
            self.bus.write_goals({i: self.goal[j] for i, j in zip(self.ids, JOINTS)})
            if done:
                # Wait a little. The servos need time to stop.
                time.sleep(0.5)
                return
            time.sleep(1.0 / self.rate)


def main(args=None):
    # Keep the default SIGINT of Python (KeyboardInterrupt). Then we can stop the shutdown ramp with Ctrl+C.
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    # SIGTERM (for example from a launch file) does the same shutdown as Ctrl+C.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = PointerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
