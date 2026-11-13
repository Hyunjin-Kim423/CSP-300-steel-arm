"""Start pointer_node with config/params.yaml. dry_run defaults to true (read-only, never writes goals)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory("arm_pointer"), "config", "params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("dry_run", default_value="true",
                              description="true = read-only; false ENABLES TORQUE and moves the arm"),
        Node(
            package="arm_pointer",
            executable="pointer_node",
            parameters=[params, {"dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool)}],
            output="screen",
        ),
    ])
