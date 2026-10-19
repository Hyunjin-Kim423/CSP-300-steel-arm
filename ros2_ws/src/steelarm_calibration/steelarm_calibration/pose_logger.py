"""Persist valid robot-base observations to newline-delimited JSON without commanding hardware."""
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger'); self.declare_parameter('topic', '/tf'); self.declare_parameter('output_path', 'calibration_observations.jsonl')
        self.sub=self.create_subscription(TransformStamped, self.get_parameter('topic').value, self.on_tf, 10)
    def on_tf(self, msg):
        if msg.header.frame_id != 'quest_world' or msg.child_frame_id != 'robot_base': return
        Path(self.get_parameter('output_path').value).open('a', encoding='utf-8').write(json.dumps({'stamp':msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, 'translation':vars(msg.transform.translation), 'rotation':vars(msg.transform.rotation)})+'\n')
def main():
    rclpy.init(); node=PoseLogger(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
