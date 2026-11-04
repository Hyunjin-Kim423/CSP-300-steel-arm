"""TF tree of the alignment modes (pure Python, no ROS imports).

In every mode, each frame must have only one parent, and the tree must have no loop.
The launch file makes the static transforms from ALIGNMENT_EDGES.
The unit tests run check_tree() on the same data.

  manual:    base_link -> quest_world -> quest_camera_optical
  tag:       quest_world -> quest_camera_optical -> robot_tag_0 -> robot_base -> base_link
  tag_once:  see ALIGNMENT_EDGES below
"""
import math

ALIGNMENT_MODES = ('manual', 'tag', 'tag_once')

# camera_pose_tf publishes this edge in every mode.
CAMERA_EDGE = ('quest_world', 'quest_camera_optical')

# (parent, child) edges that are only in one mode.
ALIGNMENT_EDGES = {
    'manual': [('base_link', 'quest_world')],
    'tag': [
        # apriltag_ros detection
        ('quest_camera_optical', 'robot_tag_0'),
        # tag_base_tf (the mount I measured)
        ('robot_tag_0', 'robot_base'),
        # Identity. pointer_node calls the base frame base_link.
        ('robot_base', 'base_link'),
    ],
    # Detect until stable, then freeze base_link -> quest_world (tag_align_once).
    # No robot_base -> base_link here. It would make a loop with the frozen edge.
    'tag_once': [
        # apriltag_ros detection (we need it only until the lock)
        ('quest_camera_optical', 'robot_tag_0'),
        # tag_base_tf (the mount I measured)
        ('robot_tag_0', 'robot_base'),
        # tag_align_once publishes this after the lock
        ('base_link', 'quest_world'),
        # Fast head pose (24 Hz) for pointing. The camera edge is only for the lock.
        ('quest_world', 'quest_head_optical'),
    ],
}


def tf_edges(mode):
    """All (parent, child) edges that the mode should put on /tf."""
    if mode not in ALIGNMENT_EDGES:
        raise ValueError(f"unknown alignment mode '{mode}', expected one of {ALIGNMENT_MODES}")
    return [CAMERA_EDGE] + list(ALIGNMENT_EDGES[mode])


def check_tree(edges):
    """Raise ValueError if a frame has two parents, or the edges have a loop."""
    parent = {}
    for p, c in edges:
        if c in parent and parent[c] != p:
            raise ValueError(f"frame '{c}' has two parents: '{parent[c]}' and '{p}'")
        parent[c] = p
    for start in parent:
        seen = {start}
        node = start
        while node in parent:
            node = parent[node]
            if node in seen:
                raise ValueError(f"loop in TF tree through '{node}'")
            seen.add(node)


def yaw_deg_to_quaternion(yaw_deg):
    """Quaternion (x, y, z, w) for a rotation of yaw_deg around +Z."""
    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def apply_yaw_translation(point, x, y, z, yaw_deg):
    """Move a point from the child frame to the parent frame, for a static (x, y, z, yaw) transform."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return (c * point[0] - s * point[1] + x, s * point[0] + c * point[1] + y, point[2] + z)
