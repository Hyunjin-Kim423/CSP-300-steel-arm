"""Hardware test: USB webcam -> rectification -> AprilTag detection.

The resulting TF is webcam_optical_test -> webcam_tag_0_test.  It is never a
robot-base TF and cannot replace the final Quest camera measurement.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('steelarm_calibration')
    apriltag = os.path.join(share, 'config', 'webcam_test_apriltag.yaml')
    device = LaunchConfiguration('device')
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/video0', description='V4L2 webcam device inside Ubuntu/WSL.'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15.0'),
        DeclareLaunchArgument('fourcc', default_value='MJPG', description='Requested V4L2 pixel format; MJPG is preferred for the Office G1080 over USB/IP.'),
        DeclareLaunchArgument('provisional_horizontal_fov_degrees', default_value='70.0', description='TEST ONLY; replace after lens calibration.'),
        Node(
            package='steelarm_calibration', executable='webcam_bridge', name='webcam_test_bridge',
            parameters=[{
                'device': device,
                'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
                'fps': ParameterValue(LaunchConfiguration('fps'), value_type=float),
                'fourcc': LaunchConfiguration('fourcc'),
                'provisional_horizontal_fov_degrees': ParameterValue(LaunchConfiguration('provisional_horizontal_fov_degrees'), value_type=float),
            }],
        ),
        Node(
            package='image_proc', executable='rectify_node', name='webcam_test_rectify',
            remappings=[
                ('image', '/test/webcam/image_raw'),
                ('camera_info', '/test/webcam/camera_info'),
                ('image_rect', '/test/webcam/image_rect'),
            ],
        ),
        Node(
            package='apriltag_ros', executable='apriltag_node', name='apriltag_webcam_test',
            parameters=[apriltag],
            remappings=[
                ('image_rect', '/test/webcam/image_rect'),
                ('camera_info', '/test/webcam/camera_info'),
                ('detections', '/test/webcam/detections'),
            ],
        ),
    ])
