"""Raise the SO-101 follower slowly from the folded rest pose to a standing pose, hold, and lower it back.

Sequence: read the current (rest) pose -> goal = current, torque on -> ramp up -> hold -> ramp down to the
rest pose -> wait 0.5 s -> torque off. Only shoulder_lift, elbow_flex and wrist_flex move; shoulder_pan,
wrist_roll and gripper keep their starting value.

DRY RUN BY DEFAULT: nothing is written to the servos unless --execute is given.

  python3 stand_pose.py                      # dry run: read-only, prints the whole plan
  python3 stand_pose.py --offline            # dry run without touching the serial port (assumes the measured rest pose)
  python3 stand_pose.py --execute            # moves the arm (torque on!)

Ctrl+C once: lower to the rest pose now, then torque off.  Ctrl+C twice: torque off immediately (arm may fall).
Uses arm_pointer.feetech_bus / arm_pointer.mapping from ~/dev_ws/src/arm_pointer (system python, no lerobot/torch).
"""
import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

try:
    from arm_pointer.feetech_bus import BusError, FeetechBus
    from arm_pointer.mapping import deg_s_to_ticks_per_s, step_toward
except ImportError:
    sys.path.insert(0, str(Path.home() / "dev_ws/src/arm_pointer"))
    from arm_pointer.feetech_bus import BusError, FeetechBus
    from arm_pointer.mapping import deg_s_to_ticks_per_s, step_toward
from scservo_sdk import COMM_SUCCESS  # importable once feetech_bus has put scs_sdk on sys.path

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
MOVING = ["shoulder_lift", "elbow_flex", "wrist_flex"]
CALIB_FILE = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/my_follower_arm.json"

# Hand-read after the recalibration (raw ticks)
STAND_DEFAULT = {"shoulder_pan": 2007, "shoulder_lift": 2091, "elbow_flex": 1025,
                 "wrist_flex": 2019, "wrist_roll": 2039, "gripper": 2039}
REST_REF = {"shoulder_pan": 2002, "shoulder_lift": 956, "elbow_flex": 3072,
            "wrist_flex": 1151, "wrist_roll": 2038, "gripper": 2039}
# Expected waypoints from the operator, for the dry-run cross-check (progress -> raw)
EXPECTED_MID = {0.25: {"shoulder_lift": 1240, "elbow_flex": 2560, "wrist_flex": 1368},
                0.50: {"shoulder_lift": 1524, "elbow_flex": 2049, "wrist_flex": 1585},
                0.75: {"shoulder_lift": 1807, "elbow_flex": 1537, "wrist_flex": 1802}}

RATE_HZ = 50.0
REST_TOLERANCE = 150      # ticks
SPEED_MAX_DEG_S = 50.0
SPEED_WARN_DEG_S = 15.0   # above this: warn (inertia, servo load)
MAX_LAG_DEFAULT = 150     # ticks goal may run ahead of the measured position before lowering starts
SETTLE_TOL_TICKS = 20     # after the goal reaches rest, wait until the measured pose is this close ...
SETTLE_TIMEOUT_S = 5.0    # ... (at most this long) before the 0.5 s wait and torque off
SHORT = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow",
         "wrist_flex": "wflex", "wrist_roll": "roll", "gripper": "grip"}
HOLD_MAX_S = 60.0
TEMP_WARN_C, TEMP_STOP_C = 60, 70
MAX_READ_FAILS = 3
ADDR_PRESENT_TEMPERATURE = 63  # lerobot STS_SMS_SERIES_CONTROL_TABLE: "Present_Temperature": (63, 1)


# ---- pure helpers (no hardware; unit-tested) -----------------------------------------------------------
def ticks_to_deg(ticks):
    return ticks * 360.0 / 4096.0


def interpolate(start, target, progress):
    """Raw values at the given progress (0..1) of a straight-line move."""
    return {j: round(start[j] + (target[j] - start[j]) * progress) for j in start}


def make_steps(frm, to, step_per_tick):
    """Per-joint max step so every joint finishes on the same tick: the largest mover gets step_per_tick."""
    max_delta = max(abs(to[j] - frm[j]) for j in frm)
    if max_delta == 0:
        return {j: 0.0 for j in frm}
    frac = min(1.0, step_per_tick / max_delta)
    return {j: abs(to[j] - frm[j]) * frac for j in frm}


def advance(goal, to, steps):
    """One control tick of the move (mapping.step_toward per joint)."""
    return {j: step_toward(goal[j], to[j], steps[j]) for j in goal}


def simulate(frm, to, step_per_tick, max_ticks=1_000_000):
    """Run advance() without sleeping. Returns (ticks until all joints arrived, {joint: tick it arrived})."""
    steps, goal, arrived = make_steps(frm, to, step_per_tick), dict(frm), {}
    for tick in range(1, max_ticks + 1):
        goal = advance(goal, to, steps)
        for j in goal:
            if j not in arrived and goal[j] == to[j]:
                arrived[j] = tick
        if len(arrived) == len(goal):
            return tick, arrived
    raise RuntimeError("move did not finish")


def lagging_joints(goal, current, max_lag):
    """Joints whose commanded goal is more than max_lag ticks away from the measured position
    (the servo is not keeping up). Returns [(joint, goal - current)]."""
    return [(j, goal[j] - current[j]) for j in JOINTS if abs(goal[j] - current[j]) > max_lag]


def check_margin(targets, calib, margin, joints):
    """Stand targets: must be at least `margin` ticks inside [range_min, range_max]."""
    problems = []
    for j in joints:
        lo, hi = calib[j]["range_min"] + margin, calib[j]["range_max"] - margin
        if not lo <= targets[j] <= hi:
            problems.append(f"{j}: {targets[j]} outside [{lo}, {hi}] (range {calib[j]['range_min']}..{calib[j]['range_max']}, margin {margin})")
    return problems


def check_range(targets, calib, joints):
    """Rest-return targets: only required to be inside [range_min, range_max], NO margin.

    Exception on purpose: the folded rest pose sits ~13 ticks from the shoulder_lift range_min (956 vs 943),
    i.e. inside the margin used for the stand pose. Applying the margin there would make the way home illegal.
    """
    problems = []
    for j in joints:
        lo, hi = calib[j]["range_min"], calib[j]["range_max"]
        if not lo <= targets[j] <= hi:
            problems.append(f"{j}: {targets[j]} outside range [{lo}, {hi}]")
    return problems


def check_rest_deviation(current, tolerance=REST_TOLERANCE, reference=REST_REF):
    """Joints whose current raw differs from the measured rest pose by more than `tolerance`."""
    return [(j, current[j], reference[j], current[j] - reference[j])
            for j in JOINTS if abs(current[j] - reference[j]) > tolerance]


def build_plan(start, stand, speed_deg_s, hold_s, calib, margin):
    """Everything the dry run reports, computed without hardware."""
    speed_ticks = deg_s_to_ticks_per_s(speed_deg_s)
    deltas = {j: stand[j] - start[j] for j in JOINTS}
    max_delta = max(abs(d) for d in deltas.values())
    move_s = max_delta / speed_ticks
    return {
        "deltas": deltas,
        "move_s": move_s,
        "up_s": move_s, "hold_s": hold_s, "down_s": move_s,
        "total_s": 2 * move_s + hold_s,
        "stand_margin_problems": check_margin(stand, calib, margin, MOVING),
        "stand_range_problems": check_range(stand, calib, JOINTS),
        "rest_range_problems": check_range(start, calib, JOINTS),
    }


# ---- hardware ------------------------------------------------------------------------------------------
class StandBus(FeetechBus):
    """FeetechBus plus a temperature read (arm_pointer itself is not modified)."""

    def read_temperatures(self):
        out = {}
        for i in self.ids:
            value, result, _ = self._ph.read1ByteTxRx(self._port, i, ADDR_PRESENT_TEMPERATURE)
            out[i] = value if result == COMM_SUCCESS else None
        return out


class Actuator:
    """Every servo WRITE of this script goes through here; with dry_run=True nothing is sent."""

    def __init__(self, bus, ids, dry_run):
        self.bus, self.ids, self.dry_run, self.torque_on = bus, ids, dry_run, False

    def set_acceleration(self, value):
        if not self.dry_run:
            self.bus.set_acceleration(value)           # WRITE: Acceleration register (41), all motors

    def write_goals(self, goals):
        if not self.dry_run:
            self.bus.write_goals({self.ids[j]: goals[j] for j in JOINTS})  # WRITE: Goal_Position (42)

    def torque(self, on):
        if not self.dry_run:
            self.bus.set_torque(on)                    # WRITE: Torque_Enable (40), all motors
        self.torque_on = on and not self.dry_run


class Interrupts:
    """First Ctrl+C/SIGTERM = lower now; second = torque off immediately."""

    def __init__(self):
        self.count = 0
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self.handler)

    def handler(self, signum, frame):
        self.count += 1

    @property
    def lower(self):
        return self.count >= 1

    @property
    def emergency(self):
        return self.count >= 2


class ReadFailure(RuntimeError):
    pass


def positions_by_joint(raw_by_id, ids):
    return {j: float(raw_by_id[ids[j]]) for j in JOINTS}


def temperatures_by_joint(bus, ids):
    temps = bus.read_temperatures()
    return {j: temps[ids[j]] for j in JOINTS}


# ---- reporting -----------------------------------------------------------------------------------------
def fmt_row(label, values, width=14):
    return f"{label:<{width}}" + " ".join(f"{values[j]:>13}" for j in JOINTS)


def print_plan(start, stand, rest, plan, args, calib, temps):
    print("\n=== PLAN " + ("(DRY RUN: nothing is written)" if not args.execute else "(EXECUTE)") + " ===")
    print(f"speed {args.speed:.1f} deg/s ({deg_s_to_ticks_per_s(args.speed):.1f} ticks/s), control {RATE_HZ:.0f} Hz, "
          f"acceleration {args.accel}, margin {args.margin} ticks, hold {args.hold_s:.1f} s")
    print(fmt_row("joint", {j: j for j in JOINTS}))
    print(fmt_row("rest (start)", {j: f"{start[j]:.0f}" for j in JOINTS}))
    print(fmt_row("stand target", {j: f"{stand[j]:.0f}" for j in JOINTS}))
    print(fmt_row("delta ticks", {j: f"{plan['deltas'][j]:+.0f}" for j in JOINTS}))
    print(fmt_row("delta deg", {j: f"{ticks_to_deg(plan['deltas'][j]):+.1f}" for j in JOINTS}))
    print(fmt_row("range min", {j: calib[j]["range_min"] for j in JOINTS}))
    print(fmt_row("range max", {j: calib[j]["range_max"] for j in JOINTS}))
    if temps:
        print(fmt_row("temp C", {j: ("?" if temps[j] is None else temps[j]) for j in JOINTS}))
    per_tick = make_steps(start, stand, deg_s_to_ticks_per_s(args.speed) / RATE_HZ)
    print(fmt_row("ticks/ctrl tick", {j: f"{per_tick[j]:.2f}" for j in JOINTS}))
    print(f"max step per control tick: {max(per_tick.values()):.2f} ticks "
          f"({ticks_to_deg(max(per_tick.values())):.3f} deg) at {RATE_HZ:.0f} Hz; lowering starts if |goal - current| > {args.max_lag} ticks")
    print(f"\nrise : {plan['up_s']:.1f} s (~{math.ceil(plan['up_s'])} s)   hold: {plan['hold_s']:.1f} s   "
          f"lower: {plan['down_s']:.1f} s (~{math.ceil(plan['down_s'])} s)   total ~{math.ceil(plan['total_s'])} s")
    print("\nlimit checks:")
    print(f"  stand targets with margin {args.margin} (moving joints): "
          + ("OK" if not plan["stand_margin_problems"] else "FAIL"))
    for p in plan["stand_margin_problems"]:
        print("    " + p)
    print("  stand targets inside range (all joints, no margin): " + ("OK" if not plan["stand_range_problems"] else "FAIL"))
    for p in plan["stand_range_problems"]:
        print("    " + p)
    print("  rest return inside range (no margin, by design):    " + ("OK" if not plan["rest_range_problems"] else "FAIL"))
    for p in plan["rest_range_problems"]:
        print("    " + p)
    for j in MOVING:
        print(f"    rest {j}: {start[j]:.0f} is {start[j] - calib[j]['range_min']:.0f} above range_min, "
              f"{calib[j]['range_max'] - start[j]:.0f} below range_max")

    print("\nmid path (straight line, all moving joints share one progress):")
    print(f"{'progress':<10}" + " ".join(f"{j:>14}" for j in MOVING))
    for p in (0.25, 0.50, 0.75):
        mid = interpolate(start, stand, p)
        print(f"{p:<10.0%}" + " ".join(f"{mid[j]:>14}" for j in MOVING))
    ref = {j: REST_REF[j] for j in JOINTS}, {j: STAND_DEFAULT[j] for j in JOINTS}
    print("\ncross-check against your expected waypoints (measured rest -> default stand):")
    all_ok = True
    for p, exp in EXPECTED_MID.items():
        calc = interpolate(ref[0], ref[1], p)
        same = all(abs(calc[j] - exp[j]) <= 1 for j in MOVING)  # +-1: exact 2048.5 ties round either way
        all_ok &= same
        print(f"  {p:>4.0%}: calc " + ", ".join(f"{j.split('_')[0]} {calc[j]}" for j in MOVING)
              + f" | expected " + ", ".join(f"{exp[j]}" for j in MOVING) + ("  OK" if same else "  MISMATCH")
              + ("  (rounding tie, differs by 1)" if same and any(calc[j] != exp[j] for j in MOVING) else ""))
    print("  -> " + ("all waypoints match" if all_ok else "SOME WAYPOINTS DIFFER"))
    ticks, arrived = simulate(start, stand, deg_s_to_ticks_per_s(args.speed) / RATE_HZ)
    print(f"\nsimulated ramp: {ticks} ticks = {ticks / RATE_HZ:.1f} s; joints arrive on ticks "
          + ", ".join(f"{j.split('_')[0]}={arrived[j]}" for j in MOVING))


# ---- the run -------------------------------------------------------------------------------------------
def run(args):
    calib = json.loads(Path(args.calibration).expanduser().read_text())
    ids = {j: calib[j]["id"] for j in JOINTS}
    stand_override = dict(STAND_DEFAULT)
    for item in args.stand or []:
        name, _, value = item.partition("=")
        if name not in MOVING or not value.lstrip("-").isdigit():
            sys.exit(f"--stand takes JOINT=RAW for {MOVING}, got '{item}'")
        stand_override[name] = int(value)

    bus = temps = None
    if args.offline:
        start = {j: float(REST_REF[j]) for j in JOINTS}
        print("OFFLINE: assuming the measured rest pose as the start pose; serial port not opened.")
    else:
        bus = StandBus(args.port, [ids[j] for j in JOINTS])
        bus.open()
        start = positions_by_joint(bus.read_positions(), ids)
        temps = temperatures_by_joint(bus, ids)
        print(f"connected on {args.port}; current raw: " + ", ".join(f"{j}={start[j]:.0f}" for j in JOINTS))

    stand = {j: float(stand_override[j]) if j in MOVING else start[j] for j in JOINTS}
    rest = dict(start)  # the pose we return to is whatever we found at startup
    plan = build_plan(start, stand, args.speed, args.hold_s, calib, args.margin)
    print_plan(start, stand, rest, plan, args, calib, temps)

    # refusals ------------------------------------------------------------------------------------------
    refuse = []
    deviating = check_rest_deviation(start)
    if deviating:
        print(f"\nWARNING: start pose is not the measured rest pose (> {REST_TOLERANCE} ticks off): "
              "the arm may already be raised or in an unexpected pose.")
        for j, cur, ref_raw, diff in deviating:
            print(f"    {j}: current {cur:.0f}, rest {ref_raw}, diff {diff:+.0f}")
        if not args.force:
            refuse.append("start pose differs from the rest pose (use --force to override)")
    refuse += plan["stand_margin_problems"] + plan["stand_range_problems"] + plan["rest_range_problems"]
    if temps and any(t is not None and t > TEMP_STOP_C for t in temps.values()):
        refuse.append(f"a motor is above {TEMP_STOP_C} C")
    if refuse:
        print("\nREFUSED:" if args.execute else "\nWOULD REFUSE to execute:")
        for r in refuse:
            print("  - " + r)
    if not args.execute:
        print("\nDry run finished. Nothing was written.")
        if bus:
            bus.close()
        return 0 if not refuse else 2
    if refuse:
        bus.close()
        return 2
    if args.offline:
        sys.exit("--execute needs the real arm; do not combine with --offline")

    return execute(args, bus, ids, start, stand, rest, calib)


def execute(args, bus, ids, start, stand, rest, calib):
    arm = Actuator(bus, ids, dry_run=False)
    irq = Interrupts()
    step_per_tick = deg_s_to_ticks_per_s(args.speed) / RATE_HZ
    dt = 1.0 / RATE_HZ
    print("\nStarting in 3 s. Ctrl+C now aborts without touching the torque.")
    for _ in range(30):
        if irq.count:
            bus.close()
            return 1
        time.sleep(0.1)

    goal = dict(rest)
    arm.set_acceleration(args.accel)
    arm.write_goals(goal)   # goal = current pose first ...
    arm.torque(True)        # ... then torque on
    print(f"torque ON (acceleration {args.accel}), holding the rest pose.")

    phase, target = "up", stand
    steps = make_steps(goal, target, step_per_tick)
    hold_t0 = None
    consecutive = total_fails = 0
    warned_at = {}
    next_report = time.monotonic()
    next_tick = time.monotonic()

    def start_lowering(reason):
        nonlocal phase, target, steps
        if phase == "down":
            return
        phase, target = "down", rest
        steps = make_steps(goal, target, step_per_tick)
        print(f"-> lowering to the rest pose: {reason}")

    try:
        while True:
            if irq.emergency:
                arm.torque(False)   # WRITE: Torque_Enable off, immediately
                print("Second Ctrl+C: torque OFF now (arm may fall).")
                bus.close()
                return 130
            next_tick += dt
            now = time.monotonic()

            try:
                cur = positions_by_joint(bus.read_positions(), ids)
                consecutive = 0
            except BusError as e:
                consecutive += 1
                total_fails += 1
                print(f"read failed ({consecutive} in a row): {e}")
                if consecutive >= MAX_READ_FAILS:
                    raise ReadFailure from e
                time.sleep(max(0.0, next_tick - time.monotonic()))
                continue  # blind cycle: write no new goal

            if irq.lower and phase != "down":
                start_lowering("Ctrl+C")
            lagging = lagging_joints(goal, cur, args.max_lag)
            if lagging and phase != "down":
                start_lowering("servo not following (goal - current > " + str(args.max_lag) + " ticks): "
                               + ", ".join(f"{j} {d:+.0f}" for j, d in lagging))

            if now >= next_report:
                next_report = now + 1.0
                temp_now = temperatures_by_joint(bus, ids)
                for j, t in temp_now.items():
                    if t is None:
                        continue
                    if t > TEMP_WARN_C and now - warned_at.get(j, -1e9) > 10.0:
                        warned_at[j] = now
                        print(f"WARNING: {j} is {t} C (> {TEMP_WARN_C} C)")
                    if t > TEMP_STOP_C:
                        start_lowering(f"{j} is {t} C (> {TEMP_STOP_C} C)")
                hottest = max((t for t in temp_now.values() if t is not None), default=None)
                span = max(abs(stand[j] - rest[j]) for j in JOINTS) or 1.0
                prog = max(abs(goal[j] - rest[j]) for j in JOINTS) / span  # 0 = rest pose, 1 = stand pose
                held = "" if hold_t0 is None or phase != "hold" else f" {now - hold_t0:.0f}/{args.hold_s:.0f}s"
                print(f"[{phase}{held} {prog:4.0%}] goal-cur: "
                      + " ".join(f"{SHORT[j]}={goal[j] - cur[j]:+.0f}" for j in JOINTS)
                      + f" | read fails {total_fails} | max temp {hottest} C"
                      + ("  LAG!" if lagging else ""))

            goal = advance(goal, target, steps)
            arrived = all(goal[j] == target[j] for j in JOINTS)
            if phase == "up" and arrived:
                phase, hold_t0 = "hold", now
                print(f"stand pose reached; holding for {args.hold_s:.0f} s")
            elif phase == "hold" and now - hold_t0 >= args.hold_s:
                start_lowering("hold finished")
            elif phase == "down" and arrived:
                arm.write_goals(goal)
                break
            try:
                arm.write_goals(goal)
            except BusError as e:
                consecutive += 1
                total_fails += 1
                print(f"write failed: {e}")
                if consecutive >= MAX_READ_FAILS:
                    raise ReadFailure from e
            time.sleep(max(0.0, next_tick - time.monotonic()))

        # The goal has reached rest; a lagging servo may still be behind it, so wait for the measured pose.
        settle_end = time.monotonic() + SETTLE_TIMEOUT_S
        settled = False
        while time.monotonic() < settle_end and not irq.emergency:
            try:
                settled = not lagging_joints(rest, positions_by_joint(bus.read_positions(), ids), SETTLE_TOL_TICKS)
            except BusError:
                break
            if settled:
                break
            time.sleep(dt)
        if not settled:
            print(f"WARNING: arm not within {SETTLE_TOL_TICKS} ticks of the rest pose after {SETTLE_TIMEOUT_S:.0f} s; "
                  "torque off anyway (it may drop a little)")
        print("rest pose reached; waiting 0.5 s")
        end = time.monotonic() + 0.5
        while time.monotonic() < end and not irq.emergency:
            time.sleep(0.02)
        arm.torque(False)   # WRITE: Torque_Enable off
        print("torque OFF. Done.")
        bus.close()
        return 0
    except ReadFailure:
        print("Bus read failed 3 times in a row: stopping the script. "
              "자동 토크 해제는 하지 않는다. 전원을 확인하라.")
        bus.close()
        return 3


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--execute", action="store_true", help="really move the arm (default: dry run)")
    p.add_argument("--offline", action="store_true", help="dry run without opening the serial port")
    p.add_argument("--force", action="store_true", help="start even if the arm is not near the rest pose")
    p.add_argument("--port", default="/dev/so101_follower")
    p.add_argument("--calibration", default=str(CALIB_FILE))
    p.add_argument("--stand", nargs="+", metavar="JOINT=RAW", help="override stand targets, e.g. elbow_flex=1100")
    p.add_argument("--speed", type=float, default=5.0, help=f"deg/s, max {SPEED_MAX_DEG_S:.0f}")
    p.add_argument("--hold_s", type=float, default=10.0, help=f"seconds, max {HOLD_MAX_S:.0f}")
    p.add_argument("--margin", type=int, default=80, help="ticks inside range_min/max for the stand pose")
    p.add_argument("--accel", type=int, default=20, help="servo acceleration register (<= 20 recommended)")
    p.add_argument("--max_lag", type=int, default=MAX_LAG_DEFAULT,
                   help="ticks; start lowering if goal - current exceeds this for any joint")
    args = p.parse_args(argv)
    if not 0 < args.speed <= SPEED_MAX_DEG_S:
        p.error(f"--speed must be in (0, {SPEED_MAX_DEG_S}]")
    if not 0 <= args.hold_s <= HOLD_MAX_S:
        p.error(f"--hold_s must be in [0, {HOLD_MAX_S}]")
    if not 1 <= args.accel <= 254:
        p.error("--accel must be 1..254")
    if args.max_lag < 1:
        p.error("--max_lag must be >= 1")
    if args.speed > SPEED_WARN_DEG_S:
        print(f"WARNING: speed > {SPEED_WARN_DEG_S:.0f} deg/s: higher inertia and servo load; keep hand on power")
    if args.accel > 20:
        print(f"WARNING: --accel {args.accel} is above the recommended 20")
    return args


if __name__ == "__main__":
    sys.exit(run(parse_args()))
