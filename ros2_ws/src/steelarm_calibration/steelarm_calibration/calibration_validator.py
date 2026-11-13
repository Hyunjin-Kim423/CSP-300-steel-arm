"""Report TF availability, freshness, and repeated robot-base pose consistency."""
import json, math
from pathlib import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from .transforms import normalize_quaternion, validate_rotation

class CalibrationValidator(Node):
    def __init__(self):
        super().__init__('calibration_validator')
        self.declare_parameter('world_frame', 'quest_world'); self.declare_parameter('base_frame', 'robot_base')
        self.declare_parameter('max_observation_age_s', 0.5); self.declare_parameter('report_path', '')
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self); self.samples = []
        self.create_timer(1.0, self.check)
    def check(self):
        try:
            t = self.buffer.lookup_transform(self.get_parameter('world_frame').value, self.get_parameter('base_frame').value, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds / 1e9
            q = (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w); validate_rotation(normalize_quaternion(q))
            if age > self.get_parameter('max_observation_age_s').value:
                self.get_logger().warn(f'robot_base TF is stale ({age:.3f}s), not a current calibration observation.'); return
            p = t.transform.translation; self.samples.append({'stamp': t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, 'translation_m': [p.x,p.y,p.z], 'quaternion_xyzw': list(normalize_quaternion(q)), 'age_s': age})
            self.get_logger().info(f'Valid quest_world -> robot_base observation; age={age:.3f}s, n={len(self.samples)}')
            self.write_report()
        except (TransformException, ValueError) as error: self.get_logger().warn(f'Calibration incomplete: {error}')
    def write_report(self):
        path = self.get_parameter('report_path').value
        if not path: return
        n=len(self.samples); means=[sum(s['translation_m'][i] for s in self.samples)/n for i in range(3)]
        std=[math.sqrt(sum((s['translation_m'][i]-means[i])**2 for s in self.samples)/n) for i in range(3)]
        Path(path).write_text(json.dumps({'note':'Consistency is not ground-truth accuracy.', 'translation_mean_m':means, 'translation_stddev_m':std, 'observations':self.samples}, indent=2))
def main():
    rclpy.init(); node=CalibrationValidator(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
