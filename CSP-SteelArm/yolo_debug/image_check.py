# Prints the size fields of a few /camera/rgb/image_raw messages and saves them to image_check.txt (same folder).
import os, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "image_check.txt")
lines = []

def cb(msg):
    n = len(msg.data)
    expected = msg.height * msg.width * 3
    line = (f"encoding={msg.encoding} width={msg.width} height={msg.height} step={msg.step} "
            f"len(data)={n} expected(h*w*3)={expected} step*h={msg.step * msg.height} missing={expected - n} "
            f"data_type={type(msg.data).__name__}")
    print(line)
    lines.append(line)
    if len(lines) >= 5:
        open(OUT, "w").write("\n".join(lines) + "\n")
        print("saved", OUT)
        rclpy.shutdown()

rclpy.init()
node = rclpy.create_node("image_check")
node.create_subscription(Image, "/camera/rgb/image_raw", cb, qos_profile_sensor_data)
try:
    rclpy.spin(node)
except Exception:
    pass
