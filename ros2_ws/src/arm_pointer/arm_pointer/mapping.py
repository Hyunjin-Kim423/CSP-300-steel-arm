"""Helper functions only (no ROS, no hardware). So it is easy to test."""
import math
from dataclasses import dataclass

TICKS_PER_REV = 4096
RAW_CENTER = 2048


def raw_to_rad(raw: float) -> float:
    return (raw - RAW_CENTER) * 2.0 * math.pi / TICKS_PER_REV


def deg_s_to_ticks_per_s(deg_s: float) -> float:
    return deg_s / 360.0 * TICKS_PER_REV


def pan_to_raw(x: float, y: float, front_raw: float, sign: int) -> float:
    """Raw servo value of shoulder_pan, so the arm points at (x, y) in the base frame."""
    pan = math.atan2(y, x)
    return front_raw + sign * pan * TICKS_PER_REV / (2.0 * math.pi)


@dataclass(frozen=True)
class PlanarArm:
    """The SO-101 seen from the side, in the vertical plane of the pan (pan = 0).

    The numbers are from so101_new_calib_abs.urdf.
    There are 3 parts at the URDF zero pose (all 3 raw values are at their zero):
    shoulder -> elbow, elbow -> wrist, and the hand (wrist -> gripper tip).
    a1, a2, a3 are their directions in degrees above the horizontal front.
    The joint angle phi is in degrees and DOWN is positive.
    Higher raw value tilts everything after that joint down (checked on the real arm, 2026-11-02).
    So phi turns all the later parts clockwise.
    We checked it with the URDF forward kinematics (error < 1e-6 m).
    """
    l1: float = 0.1160
    a1: float = 76.03
    l2: float = 0.1350
    a2: float = 2.21
    hand: float = 0.1594
    a3: float = -2.84


ARM = PlanarArm()


def raw_to_phi(raw: float, zero_raw: float, sign: int = 1) -> float:
    """Joint angle in degrees (down is positive) from a raw servo value."""
    return sign * (raw - zero_raw) * 360.0 / TICKS_PER_REV


def phi_to_raw(phi_deg: float, zero_raw: float, sign: int = 1) -> float:
    return zero_raw + sign * phi_deg * TICKS_PER_REV / 360.0


def segment_pitches(phi1: float, phi2: float, phi3: float, arm: PlanarArm = ARM):
    """Direction (degrees above the horizontal front) of the upper arm, the forearm and the hand."""
    return arm.a1 - phi1, arm.a2 - phi1 - phi2, arm.a3 - phi1 - phi2 - phi3


def wrist_from_shoulder(phi1: float, phi2: float, arm: PlanarArm = ARM):
    """Wrist position from the shoulder axis, as (horizontal, vertical)."""
    tu, tf, _ = segment_pitches(phi1, phi2, 0.0, arm)
    tu, tf = math.radians(tu), math.radians(tf)
    return arm.l1 * math.cos(tu) + arm.l2 * math.cos(tf), arm.l1 * math.sin(tu) + arm.l2 * math.sin(tf)


def pointing_pitch_deg(x: float, y: float, z: float, shoulder_x: float, shoulder_z: float,
                       min_deg: float, max_deg: float, side: int = 1) -> float:
    """Height angle (degrees above horizontal, cut to min/max) of (x, y, z), seen from the shoulder axis.

    x, y, z are in the base frame (/target_point).
    The shoulder axis is shoulder_x in front of the pan axis (in the front direction of the arm),
    and shoulder_z above the frame origin.
    side = +1: the target is in front of the arm.
    side = -1: the target is behind the arm. The arm leans back to the target.
    This robot is like this, because the user stands behind it.
    """
    horizontal = math.hypot(x, y) - side * shoulder_x
    return max(min_deg, min(max_deg, math.degrees(math.atan2(z - shoulder_z, horizontal))))


def elevation_to_arm_angle(elev_deg: float, side: int = 1) -> float:
    """Direction in the arm's own frame (0 = arm front, 90 = up), from the height angle seen from the target side."""
    return elev_deg if side > 0 else 180.0 - elev_deg


def arm_angle_to_elevation(psi_deg: float, side: int = 1) -> float:
    """The opposite of elevation_to_arm_angle. The result is in (-180, 180]."""
    e = psi_deg if side > 0 else 180.0 - psi_deg
    return (e + 180.0) % 360.0 - 180.0


def wrist_aim_phi3(x: float, y: float, z: float, phi1: float, phi2: float, shoulder_x: float, shoulder_z: float,
                   min_elev_deg: float, max_elev_deg: float, side: int = 1, arm: PlanarArm = ARM):
    """Wrist angle phi3 (degrees, down is positive). With this angle, the hand points from the WRIST axis to (x, y, z).

    Shoulder and elbow stay at phi1 and phi2 (the ready pose).
    So the wrist axis is at wrist_from_shoulder(phi1, phi2) from the shoulder.
    The hand points from that place to the target.
    The aiming height angle is cut to [min_elev_deg, max_elev_deg], seen from the target side.
    Returns (phi3, elevation_used_deg). Returns None if the input is bad.
    """
    vals = (x, y, z, phi1, phi2, shoulder_x, shoulder_z)
    if not all(math.isfinite(v) for v in vals):
        return None
    wx, wz = wrist_from_shoulder(phi1, phi2, arm)
    vx = side * math.hypot(x, y) - shoulder_x - wx
    vz = (z - shoulder_z) - wz
    if math.hypot(vx, vz) < 1e-6:
        return None
    elev = arm_angle_to_elevation(math.degrees(math.atan2(vz, vx)), side)
    elev = max(min_elev_deg, min(max_elev_deg, elev))
    psi = elevation_to_arm_angle(elev, side)
    return arm.a3 - phi1 - phi2 - psi, elev


def solve_vertical_ik(psi_deg: float, reach_m: float, arm: PlanarArm = ARM, margin_m: float = 0.01):
    """Joint angles (phi1, phi2, phi3 in degrees, down is positive) for the vertical IK.

    The wrist is on a line from the shoulder. The line has direction psi_deg, and the wrist is reach_m away.
    The hand points along the same line.
    So the tip is on this line at about reach_m + hand length. It is far from the head.
    We use only the elbow-up solution (the forearm turns clockwise from the upper arm).
    The calibrated elbow range is there, so the solution never flips.
    reach_m is cut to the range the arm can reach. Returns None if the input is bad.
    """
    if not (math.isfinite(psi_deg) and math.isfinite(reach_m)):
        return None
    lo, hi = abs(arm.l1 - arm.l2) + margin_m, arm.l1 + arm.l2 - margin_m
    if lo > hi:
        return None
    d = min(max(reach_m, lo), hi)
    cos_g = max(-1.0, min(1.0, (arm.l1 ** 2 + arm.l2 ** 2 - d ** 2) / (2.0 * arm.l1 * arm.l2)))
    # How much the forearm turns from the upper arm.
    delta = -(math.pi - math.acos(cos_g))
    theta_u = math.radians(psi_deg) - math.atan2(arm.l2 * math.sin(delta), arm.l1 + arm.l2 * math.cos(delta))
    theta_u_deg = math.degrees(theta_u)
    theta_f_deg = theta_u_deg + math.degrees(delta)
    phi1 = arm.a1 - theta_u_deg
    phi2 = arm.a2 - phi1 - theta_f_deg
    # The direction of the hand must be psi.
    phi3 = arm.a3 - phi1 - phi2 - psi_deg
    return phi1, phi2, phi3


def in_limits(raw: float, lo: float, hi: float) -> bool:
    return lo <= raw <= hi


def step_toward(current: float, target: float, max_step: float) -> float:
    """Move current toward target, at most max_step. It never goes over the target."""
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


def synced_steps(current: dict, target: dict, max_step: float) -> dict:
    """Step size of each joint, so all joints arrive at the target at the same time.

    The joint with the longest distance moves at max_step. The other joints move slower, by the same ratio.
    """
    deltas = {j: abs(target[j] - current[j]) for j in target}
    biggest = max(deltas.values(), default=0.0)
    if biggest == 0.0:
        return {j: max_step for j in target}
    return {j: (max_step * d / biggest if d > 0.0 else max_step) for j, d in deltas.items()}


def watchdog_state(elapsed_s, target_timeout_s, return_after_s):
    """Watchdog state, from the seconds after the last valid target (None = no target yet).

    "idle": no target yet.
    "track": the target is new.
    "hold": the target is lost. Keep the current pan.
    "return": the target is lost for a long time. Go back to the startup pose.
    """
    if elapsed_s is None:
        return "idle"
    if elapsed_s >= return_after_s:
        return "return"
    if elapsed_s >= target_timeout_s:
        return "hold"
    return "track"
