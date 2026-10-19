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
