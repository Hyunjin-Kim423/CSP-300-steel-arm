"""TEST ONLY: publish a fake /quest/camera/pose that moves linearly from start_xyz to end_xyz.

After publish_for_s seconds it goes silent (the node stays alive) to exercise the pointer watchdog.
Positions are in quest_world (ROS convention: x forward, y left, z up).
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class SyntheticHeadPose(Node):
    def __init__(self):
        super().__init__('synthetic_head_pose')
        self.declare_parameter('pose_topic', '/quest/camera/pose')
        self.declare_parameter('start_xyz', [1.0, 0.5, 1.5])
        self.declare_parameter('end_xyz', [1.0, -0.5, 1.5])
        self.declare_parameter('sweep_duration_s', 8.0)
        self.declare_parameter('publish_for_s', 0.0)  # 0 = never stop
        self.declare_parameter('rate_hz', 20.0)
        self.start = list(self.get_parameter('start_xyz').value)
        self.end = list(self.get_parameter('end_xyz').value)
        self.sweep = float(self.get_parameter('sweep_duration_s').value)
        self.stop_after = float(self.get_parameter('publish_for_s').value)
        self.pub = self.create_publisher(PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.t0 = self.get_clock().now()
        self.silent = False
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.get_logger().warn('TEST ONLY: publishing a synthetic head pose.')

    def tick(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        if self.stop_after > 0.0 and elapsed >= self.stop_after:
            if not self.silent:
                self.silent = True
                self.get_logger().info(f'stopped publishing at t={elapsed:.1f}s')
            return
        f = min(elapsed / self.sweep, 1.0) if self.sweep > 0.0 else 1.0
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'quest_world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            a + (b - a) * f for a, b in zip(self.start, self.end))
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SyntheticHeadPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
