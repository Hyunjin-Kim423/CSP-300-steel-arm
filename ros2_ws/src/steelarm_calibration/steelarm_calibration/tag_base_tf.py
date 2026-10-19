"""Publish robot_tag_0 -> robot_base only after an explicit physical mount measurement."""
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from .transforms import normalize_quaternion, validate_rotation

class TagBaseTF(Node):
    def __init__(self):
        super().__init__('tag_base_tf')
        self.declare_parameter('parent_frame', 'robot_tag_0')
        self.declare_parameter('child_frame', 'robot_base')
        self.declare_parameter('translation_m', [])
        self.declare_parameter('rotation_xyzw', [])
        # Jazzy represents an empty YAML sequence as a not-set parameter.  The
        # production mount file deliberately uses empty values until a physical
        # tag-to-base measurement is available.  get_parameter_or avoids the
        # ParameterUninitializedException that get_parameter would raise.
        t = list(self.get_parameter_or(
            'translation_m', Parameter('translation_m', Parameter.Type.DOUBLE_ARRAY, [])
        ).value)
        q = list(self.get_parameter_or(
            'rotation_xyzw', Parameter('rotation_xyzw', Parameter.Type.DOUBLE_ARRAY, [])
        ).value)
        if len(t) != 3 or len(q) != 4:
            self.get_logger().error('Tag mount is incomplete: measure tag-origin -> robot-base translation (m) and rotation (xyzw); TF will not be published.')
            return
        try:
            if not all(math.isfinite(v) for v in list(t) + list(q)): raise ValueError('non-finite value')
            q = normalize_quaternion(q); validate_rotation(q)
        except ValueError as error:
            self.get_logger().error(f'Invalid tag_mount.yaml: {error}; TF will not be published.')
            return
        msg = TransformStamped(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('parent_frame').value; msg.child_frame_id = self.get_parameter('child_frame').value
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, t)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
        self.broadcaster = StaticTransformBroadcaster(self); self.broadcaster.sendTransform(msg)
        self.get_logger().info('Published measured robot_tag_0 -> robot_base static transform.')

def main():
    rclpy.init(); node = TagBaseTF(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
