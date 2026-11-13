"""Change the Unity bridge PoseStamped stream to a quest_world -> quest_camera_optical TF."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from .transforms import normalize_quaternion, quest_pose_to_optical, validate_rotation


class CameraPoseTF(Node):
    def __init__(self):
        super().__init__('camera_pose_tf')
        # The Quest app sends an inverted rotation, and it does not change the camera axes (see quest_pose_to_optical).
        # True = fix it here. Then the TF is a correct world -> optical frame, and it is aligned with gravity.
        self.declare_parameter('orientation_fix', False)
        self.fix = bool(self.get_parameter('orientation_fix').value)
        self.declare_parameter('pose_topic', '/quest/camera/pose')
        self.declare_parameter('world_frame', 'quest_world')
        self.declare_parameter('camera_frame', 'quest_camera_optical')
        self.tf = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value, self.on_pose, 10)

    def on_pose(self, pose):
        o = pose.pose.orientation
        try:
            q = normalize_quaternion((o.x, o.y, o.z, o.w))
            if self.fix:
                q = quest_pose_to_optical(q)
            validate_rotation(q)
        except ValueError as error:
            self.get_logger().warn(f'Rejected camera pose: {error}')
            return

        out = TransformStamped()
        out.header = pose.header
        out.header.frame_id = self.get_parameter('world_frame').value
        out.child_frame_id = self.get_parameter('camera_frame').value
        p = pose.pose.position
        out.transform.translation.x = p.x
        out.transform.translation.y = p.y
        out.transform.translation.z = p.z
        out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = q
        self.tf.sendTransform(out)


def main():
    rclpy.init()
    node = CameraPoseTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
