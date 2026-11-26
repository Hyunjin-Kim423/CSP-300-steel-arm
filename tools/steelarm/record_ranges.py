#!/usr/bin/env python3
"""READ-ONLY range recorder for the SO-101 follower arm.

It makes a calibration json for arm_pointer WITHOUT writing anything to the motors
(unlike lerobot-calibrate, which writes new homing offsets into every motor).
So the raw values in params.yaml (pan_front_raw, ready pose, *_zero_raw) stay valid.

How to use (system python, no conda env):
    python3 tools/steelarm/record_ranges.py --from-limits   (the dashboard does this by itself)
    options: --port /dev/ttyACM0   --out <json path>

1. The arm's motor power adapter must be plugged in (USB alone powers only the USB chip).
2. Torque must be OFF (power-cycle the arm if a joint feels stiff).
3. Move every joint slowly by hand from one end to the other, then press Enter.
"""
import argparse
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
LOCAL = PROJECT / "local"            # machine-specific files (git-ignored)
sys.path.insert(0, str(PROJECT / "ros2_ws/src/arm_pointer"))
from arm_pointer.feetech_bus import COMM_SUCCESS, PacketHandler, PortHandler  # noqa: E402  (also sets up scservo_sdk path)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
DEFAULT_IDS = {j: i + 1 for i, j in enumerate(JOINTS)}  # SO-101 standard: 1..6
# STS3215 control table (read only here)
ADDR_MIN_LIMIT, ADDR_MAX_LIMIT = 9, 11
ADDR_HOMING_OFFSET = 31
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56
MIN_SPAN = 400           # ticks; a smaller range probably means the joint was not moved
FULL_TURN = {"wrist_roll"}  # LeRobot also stores 0..4095 for the wrist roll

LOG_DIR = LOCAL / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
LOG_PATH = LOG_DIR / f"record_ranges_{datetime.now():%Y%m%d_%H%M%S}.log"
_log = open(LOG_PATH, "w")


def log(msg=""):
    print(msg)
    _log.write(msg + "\n")
    _log.flush()


def find_port():
    for pattern in ("/dev/serial/by-id/*1a86*", "/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyCH343USB*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


def sign_magnitude(value, sign_bit=11):
    return -(value & ((1 << sign_bit) - 1)) if value & (1 << sign_bit) else value


class Reader:
    def __init__(self, port):
        self.port = PortHandler(port)
        self.ph = PacketHandler(0)
        if not self.port.openPort():
            raise SystemExit(f"!!! cannot open {port}")
        if not self.port.setBaudRate(1_000_000):
            raise SystemExit("!!! cannot set baudrate 1000000")

    def ping(self, i):
        _, result, _ = self.ph.ping(self.port, i)
        return result == COMM_SUCCESS

    def read1(self, i, addr):
        v, result, _ = self.ph.read1ByteTxRx(self.port, i, addr)
        return v if result == COMM_SUCCESS else None

    def read2(self, i, addr):
        v, result, _ = self.ph.read2ByteTxRx(self.port, i, addr)
        return v if result == COMM_SUCCESS else None

    def close(self):
        self.port.closePort()


def enter_pressed():
    return bool(select.select([sys.stdin], [], [], 0)[0]) and sys.stdin.readline() is not None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--out", default=str(LOCAL / "calibration/my_follower_arm.json"))
    ap.add_argument("--from-limits", action="store_true",
                    help="skip recording: use the angle-limit registers (lerobot-calibrate stored its range there)")
    args = ap.parse_args()

    log(f"date: {datetime.now()}")
    log("READ-ONLY: this script never writes to the motors.")
    port = args.port or find_port()
    if not port:
        log("!!! No serial port in WSL. In PowerShell: usbipd attach --wsl --hardware-id 1a86:55d3")
        return 1
    log(f"port: {port} -> {os.path.realpath(port)}")
    bus = Reader(port)

    # 1. Find the motors
    log("\n== 1. Motors ==")
    ids, info = {}, {}
    for j in JOINTS:
        i = DEFAULT_IDS[j]
        if not bus.ping(i):
            log(f"  {j:14s} id={i}: NO RESPONSE")
            continue
        ids[j] = i
        info[j] = {
            "torque": bus.read1(i, ADDR_TORQUE_ENABLE),
            "homing": bus.read2(i, ADDR_HOMING_OFFSET),
            "lim": (bus.read2(i, ADDR_MIN_LIMIT), bus.read2(i, ADDR_MAX_LIMIT)),
            "pos": bus.read2(i, ADDR_PRESENT_POSITION),
        }
        d = info[j]
        homing = sign_magnitude(d["homing"]) if d["homing"] is not None else None
        log(f"  {j:14s} id={i}: pos={d['pos']} torque={d['torque']} homing_offset={homing} angle_limit={d['lim']}")
    if len(ids) < len(JOINTS):
        log("!!! Some motors did not answer. Is the motor power adapter plugged in? Are the cables connected?")
        bus.close()
        return 1
    stiff = [j for j in JOINTS if info[j]["torque"]]
    if stiff:
        log(f"!!! Torque is ON for: {', '.join(stiff)}. Unplug the motor power for 5 s, plug it in, and run again.")
        log("    (This script does not turn torque off by itself, to stay read-only.)")
        bus.close()
        return 1

    if args.from_limits:
        log("\n== 2. Using the angle-limit registers (no recording) ==")
        lo = {j: info[j]["lim"][0] for j in JOINTS}
        hi = {j: info[j]["lim"][1] for j in JOINTS}
        bus.close()
        return save(args, ids, info, lo, hi)

    # 2. Record min/max while the user moves the joints
    log("\n== 2. Recording ==")
    print("Move EVERY joint slowly by hand from one end to the other (gently, do not force the end).")
    print("wrist_roll does not need to move (it is stored as 0..4095). Press Enter when done.\n")
    lo = {j: info[j]["pos"] for j in JOINTS}
    hi = dict(lo)
    fails = 0
    last_draw = 0.0
    while True:
        for j in JOINTS:
            v = bus.read2(ids[j], ADDR_PRESENT_POSITION)
            if v is None:
                fails += 1
                continue
            lo[j], hi[j] = min(lo[j], v), max(hi[j], v)
            info[j]["pos"] = v
        now = time.time()
        if now - last_draw > 0.2:
            last_draw = now
            row = "  ".join(
                f"{j[:8]}:{info[j]['pos']:4d} [{lo[j]:4d}-{hi[j]:4d}]{'*' if hi[j] - lo[j] >= MIN_SPAN or j in FULL_TURN else ' '}"
                for j in JOINTS)
            print("\r" + row, end="", flush=True)
        if enter_pressed():
            break
        time.sleep(0.01)
    print()
    bus.close()
    log(f"(read failures while recording: {fails})")
    return save(args, ids, info, lo, hi)


def save(args, ids, info, lo, hi):
    # 3. Build and save the json
    log("\n== 3. Result ==")
    calib, warn = {}, []
    for j in JOINTS:
        rmin, rmax = (0, 4095) if j in FULL_TURN else (lo[j], hi[j])
        homing = sign_magnitude(info[j]["homing"]) if info[j]["homing"] is not None else 0
        calib[j] = {"id": ids[j], "drive_mode": 0, "homing_offset": homing, "range_min": rmin, "range_max": rmax}
        span = rmax - rmin
        mark = "" if span >= MIN_SPAN else "   <-- small range, did you move it fully?"
        if mark:
            warn.append(j)
        log(f"  {j:14s} id={ids[j]} range={rmin}..{rmax} (span {span}){mark}")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        backup = out.with_name(f"{out.stem}_{datetime.now():%Y%m%d_%H%M%S}.bak.json")
        shutil.copy2(out, backup)
        log(f"old file backed up: {backup}")
    out.write_text(json.dumps(calib, indent=4) + "\n")
    log(f"saved: {out}")
    if warn:
        log(f"WARNING: small range for {', '.join(warn)}. Run again and move those joints fully.")
    log(f"log: {LOG_PATH}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        log("\ncancelled, nothing saved")
        sys.exit(130)
