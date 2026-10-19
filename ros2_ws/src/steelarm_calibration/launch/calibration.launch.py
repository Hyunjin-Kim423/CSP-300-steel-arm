"""Phase 1 launch: TF plumbing only. apriltag_ros is opt-in after Jazzy validation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    share=get_package_share_directory('steelarm_calibration')
    bridge=os.path.join(share,'config','bridge.yaml'); mount=os.path.join(share,'config','tag_mount.yaml'); apriltag=os.path.join(share,'config','apriltag_ros_candidate.yaml')
    return LaunchDescription([
      DeclareLaunchArgument('live', default_value='true', description='Reserved for Quest bridge/replay selection.'),
      DeclareLaunchArgument('enable_apriltag', default_value='false', description='Requires verified Jazzy apriltag_ros schema and measured tag size.'),
      Node(package='steelarm_calibration', executable='camera_pose_tf', parameters=[bridge]),
      Node(package='steelarm_calibration', executable='tag_base_tf', parameters=[mount]),
      Node(package='steelarm_calibration', executable='calibration_validator', parameters=[bridge]),
      Node(package='image_proc', executable='rectify_node', name='quest_rectify',
           remappings=[('image', '/quest/camera/image_raw'),
                       ('camera_info', '/quest/camera/camera_info'),
                       ('image_rect', '/quest/camera/image_rect')],
           condition=IfCondition(LaunchConfiguration('enable_apriltag'))),
      Node(package='apriltag_ros', executable='apriltag_node', name='apriltag', parameters=[apriltag],
           remappings=[('image_rect', '/quest/camera/image_rect'), ('camera_info', '/quest/camera/camera_info')], condition=IfCondition(LaunchConfiguration('enable_apriltag'))),
    ])
