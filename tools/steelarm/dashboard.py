#!/usr/bin/env python3
"""SteelArm launcher + dashboard: starts everything in ONE terminal and shows the state of each part.

Every program writes to its own log file (local/logs/session_<date>/<name>.log). The screen shows only a status table.
When something fails, its row shows the log file and the line number to look at.

Started by start_steelarm.bat (or tools/steelarm/steelarm.sh). Keys: [Y] start the arm (real modes), [Q] / Ctrl+C quit.
On quit the arm goes back to its start pose and turns torque off FIRST, then the rest stops.
"""
import json
import math
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from collections import deque
from datetime import datetime
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parents[1]
UNITY_DIR = REPO / "CSP-SteelArm"          # yolov8n-seg.pt lives here (the YOLO launch uses a relative path)
OBJECT_NODE = TOOLS / "object_target.py"
RECORD_RANGES = TOOLS / "record_ranges.py"
# Settings, loaded in this order (a later file wins). See config/steelarm/README.md.
#   1. arm_pointer/config/params.yaml       the arm_pointer package defaults (hand-measured joint values)
#   2. config/steelarm/arm_common.yaml       TEAM: every mode
#   3. config/steelarm/arm_object.yaml       TEAM: object modes (desk-side ready pose)
#   4. local/arm_object.yaml                 THIS PC only: object modes (record_pose.py without --shared)
#   5. local/arm_local.yaml                  THIS PC only: every mode (e.g. wrist_zero_raw while tuning)
CONFIG = REPO / "config" / "steelarm"
ARM_COMMON = CONFIG / "arm_common.yaml"
ARM_OBJECT = CONFIG / "arm_object.yaml"
LOCAL = REPO / "local"                     # machine-specific files and logs (git-ignored)
LOCAL_OBJECT = LOCAL / "arm_object.yaml"
LOCAL_ARM = LOCAL / "arm_local.yaml"
CALIB = LOCAL / "calibration" / "my_follower_arm.json"   # made from the motors by record_ranges.py --from-limits
USB_ID = "1a86:55d3"

MODES = {
    "1": ("head", False, "Follow the headset"),
    "2": ("object", False, "Point at the selected object"),
    "3": ("head", True, "Check only: follow the headset, DRY RUN (arm never moves)"),
    "4": ("object", True, "Check only: point at objects, DRY RUN (arm never moves)"),
}

# ---------------------------------------------------------------- terminal helpers
GREEN, RED, YELLOW, GRAY, BOLD, RESET = "\033[32m", "\033[31m", "\033[33m", "\033[90m", "\033[1m", "\033[0m"
TAG = {"OK": GREEN + " OK " + RESET, "FAIL": RED + "FAIL" + RESET, "WAIT": YELLOW + "WAIT" + RESET,
       "TODO": YELLOW + " >> " + RESET, "--": GRAY + " -- " + RESET}
ERROR_RE = re.compile(r"Traceback|\[ERROR\]|\bFATAL\b|Error:|Exception:|process has died")


def now():
    return time.monotonic()


# ---------------------------------------------------------------- one managed program
class Proc:
    """A program started in its own process group, writing to its own log file.

    The log is read as it grows, so the dashboard can parse progress lines and remember where errors are.
    """

    def __init__(self, name, cmd, logdir, cwd=None, parse=None):
        self.name, self.cmd, self.cwd, self.parse = name, cmd, cwd, parse
        self.log = logdir / f"{name}.log"
        self.p = None
        self.offset = 0
        self.lineno = 0
        self.errors = []          # (line number, text): first line of each error
        self.started = None

    def start(self):
        env = dict(os.environ, PYTHONUNBUFFERED="1", RCUTILS_COLORIZED_OUTPUT="0", RCUTILS_LOGGING_BUFFERED_STREAM="0")
        f = open(self.log, "ab")
        f.write(f"$ {' '.join(map(str, self.cmd))}\n".encode())
        f.flush()
        self.p = subprocess.Popen(list(map(str, self.cmd)), stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                  cwd=self.cwd, env=env, start_new_session=True)
        self.started = now()

    @property
    def running(self):
        return self.p is not None and self.p.poll() is None

    @property
    def exit_code(self):
        return None if self.p is None else self.p.poll()

    def read_new(self):
        if not self.log.exists():
            return
        try:
            with open(self.log, "rb") as f:
                f.seek(self.offset)
                data = f.read()
        except OSError:
            # The Windows drive (/mnt/c) sometimes fails a read for a moment (Errno 61): try again next time.
            return
        if not data:
            return
        # Only complete lines; keep a partial last line for next time.
        cut = data.rfind(b"\n") + 1
        self.offset += cut
        for raw in data[:cut].splitlines():
            self.lineno += 1
            line = raw.decode("utf-8", "replace")
            if ERROR_RE.search(line) and "do not appear to be synchronized" not in line:
                # A traceback is one error: keep its first line, but show the final "XxxError: ..." text.
                last = self.errors[-1] if self.errors else None
                in_tb = last and self.lineno - last[0] <= 40 and "Traceback" in last[1] and "process has died" not in line
                if in_tb and re.match(r"^\S*(Error|Exception)\b", line.split("] ")[-1]):
                    self.errors[-1] = (last[0], "Traceback ... " + line.strip()[-120:])
                elif not (in_tb and "Traceback" not in line):
                    self.errors.append((self.lineno, line.strip()[-140:]))
            if self.parse:
                self.parse(line)

    def where(self, err=None):
        err = err or (self.errors[0] if self.errors else None)
        rel = self.log.relative_to(REPO)
        return f"{rel} : line {err[0]}" if err else f"{rel}"

    def stop(self, sig=signal.SIGINT):
        if self.running:
            try:
                os.killpg(self.p.pid, sig)
            except ProcessLookupError:
                pass


# ---------------------------------------------------------------- ROS monitor (rates and targets)
class RosMonitor:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PointStamped, PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("steelarm_dashboard")
        self.times = {k: deque(maxlen=200) for k in ("head", "camera", "yolo")}
        self.target = None          # (x, y, z, time)
        self.object = None
        n = self.node
        # Status for the app HUD (std_msgs/String with JSON). The app only displays it.
        from std_msgs.msg import String
        self.String = String
        self.status_pub = n.create_publisher(String, "/steelarm/status", 10)
        # TF, to send where the robot base is after LOCK (robot gizmo in the app).
        from tf2_ros import Buffer, TransformListener
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, n)
        n.create_subscription(PoseStamped, "/quest/head/pose", lambda m: self.times["head"].append(now()), qos_profile_sensor_data)
        n.create_subscription(CameraInfo, "/quest/camera/camera_info", lambda m: self.times["camera"].append(now()), qos_profile_sensor_data)
        n.create_subscription(PointStamped, "/target_point", lambda m: setattr(self, "target", (m.point.x, m.point.y, m.point.z, now())), 10)
        n.create_subscription(PointStamped, "/object_target_point", lambda m: setattr(self, "object", (m.point.x, m.point.y, m.point.z, now())), 10)
        try:
            from yolo_msgs.msg import DetectionArray
            n.create_subscription(DetectionArray, "/yolo/detections", lambda m: self.times["yolo"].append(now()), qos_profile_sensor_data)
        except Exception:
            pass
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            self.rclpy.spin(self.node)
        except Exception:
            pass

    def robot_pose(self):
        """base_link in quest_world as (position, x axis, z axis), or None before LOCK."""
        from rclpy.time import Time
        try:
            t = self.tf.lookup_transform("quest_world", "base_link", Time())
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation

        def rot(v):
            x, y, z, w = q.x, q.y, q.z, q.w
            vx, vy, vz = v
            tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
            return [vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)]
        return [tr.x, tr.y, tr.z], rot((1.0, 0.0, 0.0)), rot((0.0, 0.0, 1.0))

    def publish_status(self, payload):
        self.status_pub.publish(self.String(data=json.dumps(payload)))

    def rate(self, key, window=3.0):
        t = now()
        recent = [x for x in self.times[key] if t - x < window]
        return len(recent) / window if len(recent) > 1 else 0.0

    def close(self):
        try:
            self.node.destroy_node()
            self.rclpy.try_shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------- the dashboard
class Dashboard:
    def __init__(self, mode_key):
        self.kind, self.dry, self.mode_text = MODES[mode_key]
        self.t0 = now()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logdir = LOCAL / "logs" / f"session_{stamp}"
        self.logdir.mkdir(parents=True, exist_ok=True)
        self.events = deque(maxlen=4)
        self.port = None
        self.usb_note = "looking for the arm"
        self.calib_note = ""
        self.calib_tried_port = None
        self.adb = {"state": "--", "note": "checking"}
        self.adb_done_serials = set()
        self.lock = {"state": "WAIT", "note": "waiting for camera frames", "last_progress": None}
        self.arm = {"note": "", "torque": False, "last": ""}
        self.object_note = ""
        self.quit = False
        self.arm_requested = False
        self.stopping = False
        self.procs = {}
        self.ros = None
        self.aim_up = os.environ.get("AIM_UP")

    # ---------- events
    def event(self, text):
        self.events.appendleft(f"{datetime.now():%H:%M:%S}  {text}")

    # ---------- program parsers
    def parse_lock(self, line):
        m = re.search(r"collecting samples (\d+)/(\d+)", line)
        if "LOCKED" in line:
            spread = re.search(r"spread ([\d.]+ cm / [\d.]+ deg)", line)
            self.lock.update(state="OK", note="LOCKED" + (f" (spread {spread.group(1)})" if spread else ""))
            self.event("LOCKED: headset and robot coordinates aligned")
        elif m:
            self.lock.update(state="TODO", note=f"collecting {m.group(1)}/{m.group(2)}: keep looking at the tag, hold still",
                             last_progress=now())
        elif "samples disagree" in line:
            self.lock.update(state="TODO", note="samples disagree: hold the headset still", last_progress=now())

    def parse_arm(self, line):
        if "Torque ENABLED" in line:
            self.arm["torque"] = True
            self.event("arm torque ON")
        elif "Torque disabled" in line:
            self.arm["torque"] = False
            self.event("arm torque OFF")
        elif "ready pose reached" in line:
            self.event("arm reached the ready pose")
        elif "outside" in line and "ignored" in line:
            self.arm["last"] = "target is outside the pan range (ignored)"
        m = re.search(r"\] (\[dry-run\] )?(idle|track|hold|return): (.*)", line)
        if m:
            self.arm["last"] = f"{m.group(2)}: {m.group(3)}"[:90]
        m = re.search(r"target x=.*", line)
        if m:
            self.arm["last"] = m.group(0)[:90]

    def parse_object(self, line):
        m = re.search(r"object at (.*)", line)
        if m:
            self.object_note = m.group(1)[:95]
        elif "stopped" in line:
            self.object_note = "no object selected (deselected or lost)"
        elif "no base_link" in line:
            self.object_note = "waiting for LOCK"

    # ---------- Windows-side helpers (run in background threads)
    def usb_loop(self):
        tried = 0.0
        while not self.quit:
            port = self.find_port()
            if port:
                if port != self.port:
                    self.event(f"arm USB port: {port}")
                self.port, self.usb_note = port, os.path.realpath(port)
                if not CALIB.exists() and self.calib_tried_port != port:
                    self.make_calibration(port)
            else:
                self.port = None
                if not shutil.which("usbipd.exe"):
                    self.usb_note = "no port, and usbipd.exe is not reachable: in PowerShell run  usbipd attach --wsl --hardware-id " + USB_ID
                elif now() - tried > 8:
                    tried = now()
                    self.usb_note = "attaching with usbipd ..."
                    try:
                        r = subprocess.run(["usbipd.exe", "attach", "--wsl", "--hardware-id", USB_ID],
                                           capture_output=True, text=True, timeout=30)
                        out = (r.stdout + r.stderr).strip().splitlines()
                        self.usb_note = ("usbipd: " + out[-1][:80]) if out else "usbipd attach done, waiting for the port"
                        if r.returncode != 0:
                            self.usb_note += "  (arm USB plugged in? first time: 'usbipd bind' as admin)"
                    except Exception as e:
                        self.usb_note = f"usbipd error: {e}"[:90]
            time.sleep(1.0)

    def make_calibration(self, port):
        """First run on a PC: read the arm's calibration from the motors (read-only) into local/calibration/."""
        self.calib_tried_port = port
        self.calib_note = "reading the arm calibration from the motors ..."
        self.event("no arm calibration on this PC: reading it from the motors (read-only)")
        try:
            r = subprocess.run(["python3", RECORD_RANGES, "--from-limits", "--port", port],
                               capture_output=True, text=True, timeout=30)
            if r.returncode == 0 and CALIB.exists():
                self.calib_note = ""
                self.event("arm calibration saved: local/calibration/my_follower_arm.json")
            else:
                last = [l for l in (r.stdout + r.stderr).splitlines() if l.startswith("!!!")]
                self.calib_note = (last[-1][4:] if last else "failed") + " -> fix it, then unplug/replug the arm USB"
        except Exception as e:
            self.calib_note = f"calibration read failed: {e}"[:90]

    @staticmethod
    def find_port():
        import glob
        for pattern in ("/dev/serial/by-id/*1a86*", "/dev/ttyACM*", "/dev/ttyUSB*"):
            hits = sorted(glob.glob(pattern))
            if hits:
                return hits[0]
        return None

    def adb_loop(self):
        while not self.quit:
            if not shutil.which("adb.exe"):
                self.adb = {"state": "FAIL", "note": "adb.exe not reachable from WSL (add platform-tools to the Windows PATH)"}
                time.sleep(5)
                continue
            try:
                r = subprocess.run(["adb.exe", "devices"], capture_output=True, text=True, timeout=10)
                devs = [l.split()[0] for l in r.stdout.splitlines()[1:] if l.strip().endswith("device")]
                bad = [l for l in r.stdout.splitlines()[1:] if "unauthorized" in l]
                if not devs:
                    self.adb = {"state": "TODO", "note": "headset not found: connect the USB cable" +
                                (" and allow USB debugging in the headset" if bad else "")}
                else:
                    serial = devs[0]
                    rev = subprocess.run(["adb.exe", "reverse", "--list"], capture_output=True, text=True, timeout=10).stdout
                    if "tcp:10000" not in rev:
                        subprocess.run(["adb.exe", "reverse", "tcp:10000", "tcp:10000"], capture_output=True, timeout=10)
                        self.event("adb reverse tcp:10000 set")
                    if serial not in self.adb_done_serials:
                        # Keep the headset awake when it is not worn: if it sleeps, Quest can recenter and the LOCK breaks.
                        subprocess.run(["adb.exe", "shell", "setprop", "debug.oculus.guardian_pause", "1"], capture_output=True, timeout=10)
                        subprocess.run(["adb.exe", "shell", "am", "broadcast", "-a", "com.oculus.vrpowermanager.prox_close"],
                                       capture_output=True, timeout=10)
                        self.adb_done_serials.add(serial)
                        self.event("headset: reverse port + stay-awake set")
                    self.adb = {"state": "OK", "note": f"{serial}: port 10000 reversed, stays awake"}
            except Exception as e:
                self.adb = {"state": "FAIL", "note": f"adb error: {e}"[:90]}
            time.sleep(5)

    # ---------- starting programs
    def add(self, name, cmd, cwd=None, parse=None):
        p = Proc(name, cmd, self.logdir, cwd=cwd, parse=parse)
        p.start()
        self.procs[name] = p
        self.event(f"started {name}")
        return p

    def start_programs(self):
        self.add("endpoint", ["ros2", "run", "ros_tcp_endpoint", "default_server_endpoint", "--ros-args", "-p", "ROS_IP:=0.0.0.0"])
        time.sleep(2)
        self.add("yolo", ["ros2", "launch", "yolo_bringup", "yolo.launch.py", "model:=yolov8n-seg.pt", "retina_masks:=True",
                          "threshold:=0.3", "input_image_topic:=/quest/camera/image_raw"], cwd=UNITY_DIR)
        self.add("head_target", ["ros2", "launch", "steelarm_calibration", "head_target.launch.py", "alignment:=tag_once"],
                 parse=self.parse_lock)
        if self.kind == "object":
            self.add("object_target", ["python3", OBJECT_NODE], parse=self.parse_object)

    def arm_cmd(self):
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("arm_pointer")) / "config" / "params.yaml"
        cmd = ["ros2", "run", "arm_pointer", "pointer_node", "--ros-args", "--params-file", share,
               "--params-file", ARM_COMMON]
        if self.kind == "object":
            cmd += ["--params-file", ARM_OBJECT]
            if LOCAL_OBJECT.exists():
                cmd += ["--params-file", LOCAL_OBJECT]
            cmd += ["-r", "target_point:=/object_target_point", "-p", "target_side:=1"]
        if LOCAL_ARM.exists():
            cmd += ["--params-file", LOCAL_ARM]
        if self.kind == "object" and self.aim_up:
            cmd += ["-p", f"wrist_zero_raw:={round(2070 - float(self.aim_up) * 4096 / 360)}"]
        cmd += ["-p", f"port:={self.port}", "-p", f"dry_run:={'true' if self.dry else 'false'}", "-p", "vertical_ik:=true",
                "-p", f"calibration_file:={CALIB}"]
        return cmd

    # ---------- checks used by the rows and by [Y]
    def files_state(self):
        missing = []
        if not CALIB.exists():
            missing.append("arm calibration (" + (self.calib_note or "made automatically when the arm USB is found") + ")")
        if not ARM_COMMON.exists():
            missing.append("config/steelarm/arm_common.yaml")
        if self.kind == "object" and not ARM_OBJECT.exists():
            missing.append("config/steelarm/arm_object.yaml")
        return missing

    def target_state(self):
        """(state, note, ok_for_arm)"""
        if self.lock["state"] != "OK":
            return "--", "waiting for LOCK", False
        if self.kind == "head":
            t = self.ros.target if self.ros else None
            if not t or now() - t[3] > 1.0:
                return "WAIT", "no /target_point yet (is the head pose streaming?)", False
            dist, bearing = math.hypot(t[0], t[1]), math.degrees(math.atan2(t[1], t[0]))
            note = f"headset {dist:.2f} m away, {bearing:+.0f} deg from straight ahead, height {t[2]:+.2f} m"
            if abs(bearing) > 40 or dist > 1.5:
                return "FAIL", note + "  -> not 'in front of the tag'? stale LOCK: restart", False
            return "OK", note, True
        o = self.ros.object if self.ros else None
        if not o or now() - o[3] > 1.0:
            return "TODO", "select an object with the trigger" + (f"  (last: {self.object_note})" if self.object_note else ""), True
        return "OK", self.object_note or "object selected", True

    # ---------- arm start / stop
    def try_start_arm(self, auto=False):
        if "arm" in self.procs and self.procs["arm"].running:
            return
        reasons = []
        if not self.port:
            reasons.append("no arm USB port")
        if self.files_state():
            reasons.append("missing files")
        if not self.dry:
            if self.lock["state"] != "OK":
                reasons.append("not LOCKED yet")
            ts, tnote, ok = self.target_state()
            if not ok and self.lock["state"] == "OK":
                reasons.append(tnote)
        if reasons:
            if not auto:
                self.event("cannot start the arm: " + "; ".join(reasons))
            return
        self.add("arm", self.arm_cmd(), parse=self.parse_arm)

    def shutdown(self):
        if self.stopping:
            return
        self.stopping = True
        arm = self.procs.get("arm")
        if arm and arm.running:
            self.event("stopping the arm first: it returns to the start pose, then torque off ...")
            arm.stop(signal.SIGINT)
            end = now() + 25
            while arm.running and now() < end:
                arm.read_new()
                self.draw()
                time.sleep(0.2)
            if arm.running:
                arm.stop(signal.SIGTERM)
        for p in self.procs.values():
            p.stop(signal.SIGINT)
        end = now() + 6
        while any(p.running for p in self.procs.values()) and now() < end:
            time.sleep(0.2)
        for p in self.procs.values():
            p.stop(signal.SIGKILL)
        self.quit = True

    # ---------- screen
    def rows(self):
        R = []
        missing = self.files_state()
        R.append(("Arm USB", "OK" if self.port else "TODO", self.port and f"{self.port.split('/')[-1]} -> {self.usb_note}" or self.usb_note))
        R.append(("Headset (adb)", self.adb["state"], self.adb["note"]))
        extras = [n for f, n in ((LOCAL_OBJECT, "local/arm_object.yaml"), (LOCAL_ARM, "local/arm_local.yaml"))
                  if f.exists() and (self.kind == "object" or f is LOCAL_ARM)]
        R.append(("Arm settings", "FAIL" if missing else "OK", "missing: " + ", ".join(missing) if missing else
                  "team config/steelarm" + (" + " + " + ".join(extras) if extras else "") +
                  (f" + AIM_UP {self.aim_up} deg" if self.aim_up else "")))
        R.append(self.proc_row("ROS endpoint", "endpoint", "running"))
        head, cam = (self.ros.rate("head"), self.ros.rate("camera")) if self.ros else (0, 0)
        if head > 2:
            R.append(("App stream", "OK", f"head pose {head:.0f} Hz, camera {cam:.1f} Hz"))
        else:
            R.append(("App stream", "TODO", "start the app on the headset (nothing received yet)" if now() - self.t0 > 5 else "waiting"))
        yolo = self.ros.rate("yolo") if self.ros else 0
        R.append(self.proc_row("YOLO", "yolo", f"running, results {yolo:.1f} Hz" if yolo else "running (no results yet: needs camera frames)"))
        # LOCK
        hp = self.procs.get("head_target")
        if hp and not hp.running:
            R.append(self.proc_row("LOCK", "head_target", ""))
        else:
            st, note = self.lock["state"], self.lock["note"]
            if st != "OK" and head > 2 and cam > 0:
                st = "TODO"
                if self.lock["last_progress"] is None:
                    note = "look at the AprilTag from the front, 0.4-0.5 m"
                    if now() - self.t0 > 40:
                        note += "  (no detection yet: is the tag covered, e.g. by a cable? enough light?)"
            R.append(("LOCK", st, note))
        ts, tnote, _ = self.target_state()
        R.append(("Target" if self.kind == "head" else "Object", ts, tnote))
        # arm
        arm = self.procs.get("arm")
        if arm and arm.running:
            what = "DRY RUN (never moves)" if self.dry else ("torque ON" if self.arm["torque"] else "starting")
            R.append(("Arm", "OK", f"{what}  {self.arm['last']}"))
        elif arm and not arm.running and arm.exit_code not in (0, None) and not self.stopping:
            R.append(self.proc_row("Arm", "arm", ""))
        elif self.dry:
            R.append(("Arm", "WAIT", "dry run starts when the arm USB port is found"))
        else:
            ready = self.port and not missing and self.lock["state"] == "OK"
            R.append(("Arm", "TODO" if ready else "--", "press [Y] to enable torque and start" if ready else "waits for USB + files + LOCK"))
        return R

    def proc_row(self, label, name, ok_note):
        p = self.procs.get(name)
        if p is None:
            return (label, "--", "not started")
        if p.running:
            # Running, but a node inside a launch may have died: ros2 launch reports it in the log.
            died = [e for e in p.errors if "process has died" in e[1]]
            if died:
                first = p.errors[0]
                return (label, "FAIL", f"a node died -> {p.where(first)}")
            return (label, "OK", ok_note)
        first = p.errors[0] if p.errors else None
        return (label, "FAIL", f"exited (code {p.exit_code}) -> {p.where(first)}")

    def draw(self):
        for p in self.procs.values():
            p.read_new()
        if self.dry and not self.stopping and "arm" not in self.procs and self.port and not self.files_state():
            self.try_start_arm(auto=True)
        mode = f"{self.mode_text}"
        out = [f"{BOLD}SteelArm{RESET}  mode {self.kind}{' (dry run)' if self.dry else ''}: {mode}"
               f"{' ' * 4}{datetime.now():%H:%M:%S}", "-" * 100]
        rows = self.rows()
        for label, st, note in rows:
            out.append(f" {label:<14} [{TAG.get(st, st)}]  {note}")
        out.append("-" * 100)
        errs = [(p, e) for p in self.procs.values() for e in p.errors[-2:]]
        self.send_status(rows, errs)
        if errs:
            out.append(" Recent errors (file : line):")
            for p, e in errs[-4:]:
                out.append(f"   {p.where(e)}   {GRAY}{e[1][:70]}{RESET}")
        out.append(" Events:")
        out += [f"   {e}" for e in self.events]
        out.append("-" * 100)
        keys = "[Q] / Ctrl+C quit" if self.dry else "[Y] start the arm   [Q] / Ctrl+C quit (the arm folds first)"
        out.append(f" {keys}      logs: {self.logdir.relative_to(REPO)}/")
        sys.stdout.write("\033[H\033[2J" + "\n".join(out) + "\n")
        sys.stdout.flush()

    def send_status(self, rows, errs):
        """The same table, for the app HUD (2 Hz, from draw)."""
        if not self.ros or now() - getattr(self, "_last_status", 0) < 0.5:
            return
        self._last_status = now()
        payload = {"mode": f"{self.kind}{' (dry run)' if self.dry else ''}",
                   "rows": [{"label": l, "state": st, "note": str(n)} for l, st, n in rows],
                   "errors": [f"{p.where(e)}  {e[1][:60]}" for p, e in errs[-3:]],
                   "has_robot": False}
        pose = self.ros.robot_pose() if self.lock["state"] == "OK" else None
        if pose:
            payload.update(has_robot=True, robot_p=pose[0], robot_ax=pose[1], robot_az=pose[2])
        try:
            self.ros.publish_status(payload)
        except Exception:
            pass

    # ---------- main loop
    def run(self):
        threading.Thread(target=self.usb_loop, daemon=True).start()
        threading.Thread(target=self.adb_loop, daemon=True).start()
        self.ros = RosMonitor()
        self.start_programs()
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while not self.quit:
                self.draw()
                if select.select([sys.stdin], [], [], 0.5)[0]:
                    k = sys.stdin.read(1).lower()
                    if k == "y" and not self.dry:
                        self.try_start_arm()
                    elif k == "q":
                        self.shutdown()
        except KeyboardInterrupt:
            self.shutdown()
        finally:
            if not self.stopping:
                self.shutdown()
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.ros.close()
            self.summary()

    def summary(self):
        print("\nStopped. Logs:", self.logdir)
        for p in self.procs.values():
            p.read_new()
            if p.errors:
                print(f"  {p.name}: first error at {p.where(p.errors[0])}")


def choose_mode():
    if len(sys.argv) > 1 and sys.argv[1] in MODES:
        return sys.argv[1]
    print(f"{BOLD}SteelArm launcher{RESET}\n")
    for k, (_, _, text) in MODES.items():
        print(f"  {k}  {text}")
    while True:
        k = input("\nChoose a mode [1-4]: ").strip()
        if k in MODES:
            return k


def main():
    if os.environ.get("CONDA_DEFAULT_ENV") not in (None, "", "base"):
        print("!!! a conda env is active. Run 'conda deactivate' and start again.")
        return 1
    key = choose_mode()
    dash = Dashboard(key)
    signal.signal(signal.SIGTERM, lambda *a: dash.shutdown())
    try:
        dash.run()
    except Exception:
        import traceback
        traceback.print_exc()
        crash = dash.logdir / "dashboard_crash.log"
        crash.write_text(traceback.format_exc())
        dash.shutdown()
        print(f"\nThe dashboard crashed: {crash.relative_to(REPO)} (see the last lines)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
