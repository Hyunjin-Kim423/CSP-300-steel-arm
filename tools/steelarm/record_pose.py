#!/usr/bin/env python3
"""READ-ONLY: records the arm's DESK-SIDE ready pose for object pointing -> local/params_object.yaml.

Nothing is written to the motors. The original params.yaml is not touched.
1. Motor power on, torque OFF (power-cycle the arm if a joint is stiff).
2. Pose the arm by hand: the hand leaning over the DESK side, wrist roughly in the middle of its range,
   nothing touching the desk. (You can pose it before or while this script runs.)
3. Press Enter to save. Run again any time to re-record (the old file is backed up).

Run (no conda env, from the repo folder):  python3 tools/steelarm/record_pose.py [--shared]
"""
import glob
import json
import os
import select
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[1]
LOCAL = PROJECT / "local"
sys.path.insert(0, str(PROJECT / "ros2_ws/src/arm_pointer"))
from arm_pointer.feetech_bus import COMM_SUCCESS, PacketHandler, PortHandler  # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
READY = ["shoulder_lift", "elbow_flex", "wrist_flex"]
ADDR_TORQUE_ENABLE, ADDR_PRESENT_POSITION = 40, 56
MARGIN = 60  # same as limit_margin_ticks in params.yaml
CALIB = LOCAL / "calibration/my_follower_arm.json"
# Default: a machine-only override (local/arm_object.yaml, beats the team file).
# --shared: write the TEAM file config/steelarm/arm_object.yaml (commit it so everyone gets the new pose).
SHARED = "--shared" in sys.argv
OUT = PROJECT / "config/steelarm/arm_object.yaml" if SHARED else LOCAL / "arm_object.yaml"
LOG = LOCAL / "logs" / f"record_pose_{datetime.now():%Y%m%d_%H%M%S}.log"
LOG.parent.mkdir(parents=True, exist_ok=True)
_log = open(LOG, "w")


def log(msg=""):
    print(msg)
    _log.write(msg + "\n")
    _log.flush()


def find_port():
    for pattern in ("/dev/serial/by-id/*1a86*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


def main():
    log(f"date: {datetime.now()}   READ-ONLY (never writes to the motors)")
    calib = json.loads(CALIB.read_text())
    ids = {j: calib[j]["id"] for j in JOINTS}
    port = os.environ.get("PORT") or find_port()
    if not port:
        log("!!! no serial port (usbipd attach --wsl --hardware-id 1a86:55d3)")
        return 1
    ph, pt = PacketHandler(0), PortHandler(port)
    if not pt.openPort() or not pt.setBaudRate(1_000_000):
        log(f"!!! cannot open {port}")
        return 1

    def read(j, addr, size=2):
        f = ph.read2ByteTxRx if size == 2 else ph.read1ByteTxRx
        v, res, _ = f(pt, ids[j], addr)
        return v if res == COMM_SUCCESS else None

    stiff = [j for j in JOINTS if read(j, ADDR_TORQUE_ENABLE, 1)]
    if stiff:
        log(f"!!! torque is ON for {', '.join(stiff)}: unplug the motor power for 5 s and run again.")
        return 1

    print("\nPose the arm (hand over the DESK side). Live values below. Press Enter to save.\n")
    pos = {}
    while True:
        for j in JOINTS:
            v = read(j, ADDR_PRESENT_POSITION)
            if v is not None:
                pos[j] = v
        print("\r" + "  ".join(f"{j[:9]}={pos.get(j, '?')}" for j in JOINTS) + "   ", end="", flush=True)
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            break
        time.sleep(0.1)
    print()
    pt.closePort()

    log("\n== recorded pose (raw) ==")
    problems = []
    for j in JOINTS:
        lo, hi = calib[j]["range_min"] + MARGIN, calib[j]["range_max"] - MARGIN
        v = pos[j]
        note = ""
        if j in READY:
            if not lo <= v <= hi:
                note = f"  <-- OUTSIDE usable range {lo}..{hi}"
                problems.append(j)
            elif j == "wrist_flex":
                note = f"  (wrist room: {v - lo} ticks one way, {hi - v} the other)"
        log(f"  {j:14s} {v:5d}{note}")
    if problems:
        log(f"!!! {', '.join(problems)} outside the usable range: pose again a little away from the end. NOT saved.")
        return 1

    if OUT.exists() and not SHARED:   # the team file has git history instead
        backup = OUT.with_name(f"arm_object_{datetime.now():%Y%m%d_%H%M%S}.bak.yaml")
        shutil.copy2(OUT, backup)
        log(f"old file backed up: {backup}")
    OUT.write_text(
        f"# Object mode: desk-side ready pose. Made by tools/steelarm/record_pose.py{' --shared' if SHARED else ''} on "
        f"{datetime.now():%Y-%m-%d %H:%M}.\n"
        + ("# TEAM file (committed). A machine can override it with local/arm_object.yaml.\n" if SHARED else
           "# Machine-only override (git-ignored). Overrides config/steelarm/arm_object.yaml.\n")
        + "# Loaded by the dashboard in object modes, after params.yaml and config/steelarm/arm_common.yaml.\n"
        "# Desk-side ready pose: the hand leans over the desk, so the mirrored target + target_side 1 points at the object.\n"
        "pointer_node:\n  ros__parameters:\n"
        + "".join(f"    ready_{j}_raw: {pos[j]}\n" for j in READY)
        + "    target_side: 1\n"
        f"    # recorded at shoulder_pan={pos['shoulder_pan']} (the pan is not used; it follows the object)\n")
    log(f"saved: {OUT}\nlog: {LOG}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        log("\ncancelled, nothing saved")
        sys.exit(130)
