"""Persist valid robot-base observations to newline-delimited JSON without commanding hardware."""
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger'); self.declare_parameter('topic', '/tf'); self.declare_parameter('output_path', 'calibration_observations.jsonl')
        self.sub=self.create_subscription(TFMessage, self.get_parameter('topic').value, self.on_tf, 100)
    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id != 'quest_world' or t.child_frame_id != 'robot_base': continue
            tr, ro = t.transform.translation, t.transform.rotation
            record = {'stamp': t.header.stamp.sec + t.header.stamp.nanosec*1e-9, 'translation': {'x': tr.x, 'y': tr.y, 'z': tr.z}, 'rotation': {'x': ro.x, 'y': ro.y, 'z': ro.z, 'w': ro.w}}
            with Path(self.get_parameter('output_path').value).open('a', encoding='utf-8') as f: f.write(json.dumps(record) + '\n')
def main():
    rclpy.init(); node=PoseLogger(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
