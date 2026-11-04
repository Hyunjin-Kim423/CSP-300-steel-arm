"""Dependency-light rigid-transform validation and Unity/ROS conversion helpers."""
import math

UNITY_TO_ROS = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))

def normalize_quaternion(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        raise ValueError('zero-length quaternion')
    return tuple(v / n for v in q)

def quaternion_to_matrix(q):
    x, y, z, w = normalize_quaternion(q)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))

def validate_rotation(q, tolerance=1e-6):
    r = quaternion_to_matrix(q)
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    if any(abs(dot(r[i], r[j]) - (1.0 if i == j else 0.0)) > tolerance for i in range(3) for j in range(3)):
        raise ValueError('rotation is not orthogonal')
    det = (r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1]) - r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0]) + r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]))
    if abs(det - 1.0) > tolerance:
        raise ValueError('rotation is not proper (determinant != 1)')
    return r

def unity_position_to_ros(p):
    # Unity: X right, Y up, Z forward (left-handed); ROS optical: X right, Y down, Z forward.
    return (p[0], -p[1], p[2])

def unity_quaternion_to_ros_optical(q):
    # Coordinate-basis conversion C*R*C^-1 for Unity camera-local -> ROS optical.
    # Implemented through matrices elsewhere at runtime; this function guards only normalized input.
    return normalize_quaternion(q)


def quaternion_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_multiply(a, b):
    """Hamilton product a * b for (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# Rotation matrix [[0,0,1],[-1,0,0],[0,-1,0]].
# It writes the optical axes (x right, y down, z forward) in the ROS body frame (x forward, y left, z up).
BODY_TO_OPTICAL_QUAT = (-0.5, 0.5, -0.5, 0.5)


def quest_pose_to_optical(q):
    """Fix the camera rotation from the Quest app (CameraImagePublisher.UnityWorldCameraToRos).

    The app changes the Unity quaternion to (z, -x, y, w).
    The correct Unity -> ROS change is (-z, x, -y, w). So the app sends the opposite (inverse) rotation.
    Also, the app does not change the camera body axes to the optical axes.
    So the correct world-from-optical rotation is conj(q_published) * BODY_TO_OPTICAL.

    I checked it with tag detections when the head was in many directions (the same answer came from 2 recordings).
    I also checked with gravity (the optical "down" axis points to the floor when the headset is upright).
    """
    return normalize_quaternion(quaternion_multiply(quaternion_conjugate(normalize_quaternion(q)), BODY_TO_OPTICAL_QUAT))
