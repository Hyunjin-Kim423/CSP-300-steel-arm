"""Publish a hand-set static TF parent_frame -> child_frame from (x, y, z, yaw_deg).

Default use is base_link -> quest_world for the temporary manual alignment: x, y, z (metres) and yaw_deg
give the pose of quest_world in base_link, so p_base = Rz(yaw) * p_world + (x, y, z). The same node with
all zeros is used as the identity robot_base -> base_link link in tag mode.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .alignment import yaw_deg_to_quaternion


class ManualAlignmentTF(Node):
    def __init__(self):
        super().__init__('manual_alignment_tf')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'quest_world')
        for name in ('x', 'y', 'z', 'yaw_deg'):
            self.declare_parameter(name, 0.0)
        v = {n: float(self.get_parameter(n).value) for n in ('x', 'y', 'z', 'yaw_deg')}

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('parent_frame').value
        msg.child_frame_id = self.get_parameter('child_frame').value
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = v['x'], v['y'], v['z']
        q = yaw_deg_to_quaternion(v['yaw_deg'])
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(msg)
        self.get_logger().info(
            f"static TF {msg.header.frame_id} -> {msg.child_frame_id}: "
            f"xyz=({v['x']:.3f}, {v['y']:.3f}, {v['z']:.3f}) m, yaw={v['yaw_deg']:.1f} deg")


def main():
    rclpy.init()
    node = ManualAlignmentTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
