#!/usr/bin/env python3
"""Camera check for the AprilTag step: saves real Quest frames and tests them for the tag OFFLINE.

It answers: do frames arrive, how fast, are they sharp/bright enough, is the tag in them,
how big is the tag in pixels, and is the rectified image OK. Do NOT run head_target at the same time: the .sh starts its own rectify node to check image_rect.

Run (start_steelarm.bat + app running, look at the tag the whole time):
    bash tools/steelarm/capture_frames.sh
Output: local/logs/frames_<date>/ (PNG files + report.txt)
"""
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
OUT = Path(__file__).resolve().parents[2] / "local" / "logs" / f"frames_{datetime.now():%Y%m%d_%H%M%S}"
OUT.mkdir(parents=True, exist_ok=True)
REPORT = open(OUT / "report.txt", "w")


def log(msg=""):
    print(msg, flush=True)
    REPORT.write(msg + "\n")
    REPORT.flush()


def make_detector():
    """OpenCV's AprilTag 36h11 detector (independent of apriltag_ros)."""
    if not hasattr(cv2, "aruco"):
        return None
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        return lambda g: det.detectMarkers(g)[:2]
    params = cv2.aruco.DetectorParameters_create()
    return lambda g: cv2.aruco.detectMarkers(g, d, parameters=params)[:2]


DETECT = make_detector()


def to_gray(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        code = cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
        return img, cv2.cvtColor(img, code)
    if msg.encoding in ("mono8", "8UC1"):
        g = buf.reshape(msg.height, msg.step)[:, : msg.width]
        return g, g
    raise ValueError(f"unsupported encoding {msg.encoding}")


class Capture(Node):
    def __init__(self):
        super().__init__("steelarm_frame_check")
        self.info = None
        self.stats = {"raw": [], "rect": []}
        self.saved = {"raw": 0, "rect": 0}
        self.last_save = {"raw": 0.0, "rect": 0.0}
        self.create_subscription(CameraInfo, "/quest/camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/quest/camera/image_raw", lambda m: self.on_image("raw", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/quest/camera/image_rect", lambda m: self.on_image("rect", m), qos_profile_sensor_data)

    def on_info(self, m):
        self.info = m

    def on_image(self, kind, msg):
        now = time.time()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            img, gray = to_gray(msg)
        except Exception as e:
            log(f"[{kind}] cannot read image: {e}")
            return
        bright = float(gray.mean())
        sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        tag = "n/a (no cv2.aruco)"
        if DETECT is not None:
            corners, ids = DETECT(gray)
            if ids is not None and len(ids):
                parts = []
                for c, i in zip(corners, ids.flatten()):
                    side = float(np.mean([np.linalg.norm(c[0][k] - c[0][(k + 1) % 4]) for k in range(4)]))
                    parts.append(f"id {i} side {side:.0f}px")
                tag = ", ".join(parts)
            else:
                tag = "none"
        self.stats[kind].append((now, stamp, bright, sharp, tag))
        line = f"[{kind}] {msg.width}x{msg.height} {msg.encoding} bright={bright:5.1f} sharp={sharp:7.1f} tag: {tag}"
        log(line)
        if self.saved[kind] < 12 and now - self.last_save[kind] > 1.0:
            self.last_save[kind] = now
            self.saved[kind] += 1
            out = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img
            cv2.imwrite(str(OUT / f"{kind}_{self.saved[kind]:02d}.png"), out)


def main():
    log(f"date: {datetime.now()}  opencv {cv2.__version__}  aruco={'yes' if DETECT else 'NO'}")
    log(f">>> Look at the AprilTag from the front (0.4-0.5 m) and hold still for {DURATION:.0f} s.")
    rclpy.init()
    node = Capture()
    # Wait (up to 40 s) for the first frame, then record for DURATION seconds.
    wait_end = time.time() + 40
    while not node.stats["raw"] and time.time() < wait_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.stats["raw"]:
        log("!!! No camera frame in 40 s. Is the app running on the headset, and is the headset ON your head")
        log("    (it pauses when taken off)? Is the '1 ROS Endpoint' tab still running?")
    end = time.time() + DURATION
    while node.stats["raw"] and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    log("\n================ SUMMARY ================")
    i = node.info
    if i is None:
        log("camera_info: NOT RECEIVED")
    else:
        log(f"camera_info: {i.width}x{i.height} model='{i.distortion_model}' D={list(i.d)}")
        log(f"  K={[round(v, 1) for v in i.k]}")
        log(f"  P={[round(v, 1) for v in i.p]}")
        if not any(i.p):
            log("  !!! P is all zeros: image_proc rectify output will be broken")
    for kind in ("raw", "rect"):
        s = node.stats[kind]
        if not s:
            log(f"{kind}: no frames received" + (" (normal if head_target is not running)" if kind == "rect" else ""))
            continue
        span = s[-1][0] - s[0][0]
        rate = (len(s) - 1) / span if span > 0 else 0
        gaps = [b[1] - a[1] for a, b in zip(s, s[1:])]
        hits = sum(1 for x in s if x[4].startswith("id"))
        log(f"{kind}: {len(s)} frames in {span:.1f}s = {rate:.2f} Hz, max gap {max(gaps) if gaps else 0:.2f}s, "
            f"brightness avg {np.mean([x[2] for x in s]):.0f}, sharpness median {np.median([x[3] for x in s]):.0f}, "
            f"tag found in {hits}/{len(s)}")
    log(f"\nSaved frames + this report: {OUT}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
