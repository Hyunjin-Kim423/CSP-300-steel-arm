"""Publish the Quest camera position, expressed in the robot base frame, as /target_point.

Only the translation of TF target_frame <- source_frame is used. Nothing is published while the TF
is missing or has not updated for max_tf_age_s (measured on the local clock), so pointer_node's watchdog can see that the headset is gone.
"""
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .head_target_logic import ArrivalAgeTracker, ValidityTracker, evaluate_sample


class HeadTargetNode(Node):
    def __init__(self):
        super().__init__('head_target_node')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'quest_camera_optical')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_tf_age_s', 0.3)
        # The listener shares this node's executor, so waiting here would also block TF reception.
        self.declare_parameter('tf_timeout_s', 0.0)
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.max_age = float(self.get_parameter('max_tf_age_s').value)
        self.timeout = Duration(seconds=float(self.get_parameter('tf_timeout_s').value))

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PointStamped, 'target_point', 10)
        self.tracker = ValidityTracker()
        self.arrival = ArrivalAgeTracker()
        self.create_timer(1.0 / float(self.get_parameter('publish_rate_hz').value), self.tick)

    def tick(self):
        translation, age, error = None, 0.0, ''
        try:
            t = self.buffer.lookup_transform(self.target_frame, self.source_frame, Time(), timeout=self.timeout)
            p = t.transform.translation
            translation = (p.x, p.y, p.z)
            # Age is counted from when a new stamp first showed up here (local clock), because the Quest
            # clock can differ from this PC's by seconds.
            stamp_ns = Time.from_msg(t.header.stamp).nanoseconds
            age = self.arrival.update(stamp_ns, self.get_clock().now().nanoseconds)
        except TransformException as e:
            error = str(e)
            self.arrival.reset()

        valid, reason = evaluate_sample(translation, age, self.max_age)
        if self.tracker.update(valid):
            if valid:
                self.get_logger().info(f'{self.source_frame} in {self.target_frame}: valid, publishing /target_point')
            else:
                self.get_logger().warn(f'not publishing /target_point: {reason}' + (f' ({error})' if error else ''))
        if not valid:
            return

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.point.x, msg.point.y, msg.point.z = translation
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HeadTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
