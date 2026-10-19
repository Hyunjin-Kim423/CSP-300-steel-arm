"""Turn the Unity bridge PoseStamped stream into quest_world -> quest_camera_optical TF."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from .transforms import normalize_quaternion, validate_rotation

class CameraPoseTF(Node):
    def __init__(self):
        super().__init__('camera_pose_tf')
        self.declare_parameter('pose_topic', '/quest/camera/pose')
        self.declare_parameter('world_frame', 'quest_world')
        self.declare_parameter('camera_frame', 'quest_camera_optical')
        self.tf = TransformBroadcaster(self)
        self.sub = self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value, self.on_pose, 10)
    def on_pose(self, pose):
        try: q = normalize_quaternion((pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)); validate_rotation(q)
        except ValueError as error: self.get_logger().warn(f'Rejected camera pose: {error}'); return
        out = TransformStamped(); out.header = pose.header; out.header.frame_id = self.get_parameter('world_frame').value; out.child_frame_id = self.get_parameter('camera_frame').value
        out.transform.translation.x = pose.pose.position.x; out.transform.translation.y = pose.pose.position.y; out.transform.translation.z = pose.pose.position.z
        out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = q; self.tf.sendTransform(out)
def main():
    rclpy.init(); node = CameraPoseTF(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
