"""Hardware-free detector integration test; never starts production mount TF."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
 share=get_package_share_directory('steelarm_calibration')
 return LaunchDescription([DeclareLaunchArgument('image_path'), Node(package='steelarm_calibration',executable='synthetic_replay',parameters=[{'image_path':LaunchConfiguration('image_path')}]), Node(package='apriltag_ros',executable='apriltag_node',name='apriltag_synthetic',parameters=[os.path.join(share,'config','synthetic_apriltag_ros.yaml')],remappings=[('image_rect','/synthetic/camera/image_rect'),('camera_info','/synthetic/camera/camera_info')])])
