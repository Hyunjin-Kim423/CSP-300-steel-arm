"""Quest head position -> /target_point (in base_link). You can choose how to align the base.

alignment:=manual    base_link -> quest_world comes from the manual_* arguments (a temporary hand alignment).
alignment:=tag_once  Uses the tag path with apriltag on. Detect until stable, then freeze base_link -> quest_world (tag_align_once).
alignment:=tag       Uses the old tag path (calibration.launch.py) and adds an identity robot_base -> base_link.

Each mode starts only its own edges. So base_link never has two parents, and the tree has no loop.
See steelarm_calibration/alignment.py. The unit tests check it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from steelarm_calibration.alignment import ALIGNMENT_MODES


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _setup(context):
    mode = LaunchConfiguration('alignment').perform(context)
    share = get_package_share_directory('steelarm_calibration')
    if mode not in ALIGNMENT_MODES:
        raise RuntimeError(f"alignment must be one of {ALIGNMENT_MODES}, got '{mode}'")

    if mode == 'manual':
        actions = [
            # For pointing, we need only the head position. So use the head pose stream.
            # It is sent separately from the camera frames (they are slow and big).
            # Use pose_topic:=/quest/camera/pose for the old way.
            Node(package='steelarm_calibration', executable='camera_pose_tf', parameters=[
                os.path.join(share, 'config', 'bridge.yaml'),
                {'pose_topic': LaunchConfiguration('pose_topic')}]),
            Node(package='steelarm_calibration', executable='manual_alignment_tf', name='manual_alignment_tf',
                 parameters=[{'parent_frame': 'base_link', 'child_frame': 'quest_world',
                              'x': _float('manual_x'), 'y': _float('manual_y'), 'z': _float('manual_z'),
                              'yaw_deg': _float('manual_yaw_deg')}]),
        ]
    elif mode == 'tag_once':
        actions = [
            # Same as tag mode (apriltag is always on), but no robot_base -> base_link alias.
            # tag_align_once freezes base_link -> quest_world after a stable detection.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'calibration.launch.py')),
                launch_arguments={'enable_apriltag': 'true'}.items()),
            Node(package='steelarm_calibration', executable='tag_align_once', name='tag_align_once'),
            # The camera pose above comes only with the images (they are slow).
            # If we point with it, the TF gets too old (> max_tf_age_s) and the arm stops.
            # So we point with the fast head pose. It is a separate frame under the same quest_world.
            Node(package='steelarm_calibration', executable='camera_pose_tf', name='head_pose_tf', parameters=[
                os.path.join(share, 'config', 'bridge.yaml'),
                {'pose_topic': LaunchConfiguration('pose_topic'), 'camera_frame': 'quest_head_optical'}]),
        ]
    else:
        actions = [
            # camera_pose_tf, tag_base_tf, the validator and the optional apriltag come from the old launch file.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'calibration.launch.py')),
                launch_arguments={'enable_apriltag': LaunchConfiguration('enable_apriltag')}.items()),
            Node(package='steelarm_calibration', executable='manual_alignment_tf', name='base_link_alias_tf',
                 parameters=[{'parent_frame': 'robot_base', 'child_frame': 'base_link'}]),
        ]
    source = 'quest_head_optical' if mode == 'tag_once' else 'quest_camera_optical'
    actions.append(Node(package='steelarm_calibration', executable='head_target_node',
                        parameters=[{'target_frame': 'base_link', 'source_frame': source,
                                     'max_tf_age_s': _float('max_tf_age_s')}]))
    return actions


def generate_launch_description():
    args = [DeclareLaunchArgument('alignment', default_value='manual', description='manual | tag | tag_once'),
            DeclareLaunchArgument('enable_apriltag', default_value='false', description='tag mode only'),
            DeclareLaunchArgument('max_tf_age_s', default_value='0.3'),
            DeclareLaunchArgument('pose_topic', default_value='/quest/head/pose', description='manual and tag_once modes (pointing pose)')]
    args += [DeclareLaunchArgument(n, default_value='0.0',
                                   description='manual mode: pose of quest_world in base_link (m / deg)')
             for n in ('manual_x', 'manual_y', 'manual_z', 'manual_yaw_deg')]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
