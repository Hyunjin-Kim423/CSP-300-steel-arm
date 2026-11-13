"""Pure-Python math for tag_align_once: invert a pose, average samples, and check if they are stable.

A pose is (t, q). t = (x, y, z) in metres, q = (x, y, z, w).
It maps child -> parent: p_parent = R(q) p_child + t.
"""
import math
from collections import deque

from .transforms import normalize_quaternion, quaternion_to_matrix


def invert_pose(t, q):
    """Pose of the parent in the child frame."""
    q = normalize_quaternion(q)
    r = quaternion_to_matrix(q)
    # The new position is -R^T t.
    inv_t = tuple(-sum(r[k][i] * t[k] for k in range(3)) for i in range(3))
    return inv_t, (-q[0], -q[1], -q[2], q[3])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def quaternion_angle_deg(a, b):
    """Smallest angle between two unit quaternions (q and -q are the same rotation)."""
    d = min(1.0, abs(_dot(normalize_quaternion(a), normalize_quaternion(b))))
    return math.degrees(2.0 * math.acos(d))


def mean_pose(samples):
    """Mean of [(t, q), ...].

    First, the sign of each quaternion is matched to the first one. Then we average them.
    It is OK when the samples are close to each other.
    """
    n = len(samples)
    t = tuple(sum(s[0][i] for s in samples) / n for i in range(3))
    ref = normalize_quaternion(samples[0][1])
    acc = [0.0] * 4
    for _, q in samples:
        q = normalize_quaternion(q)
        sign = 1.0 if _dot(q, ref) >= 0.0 else -1.0
        for i in range(4):
            acc[i] += sign * q[i]
    return t, normalize_quaternion(acc)


class StableWindow:
    """Keeps the last `size` pose samples. result() gives the mean when they agree. If not, it gives None.

    A sample agrees when it is within pos_tol_m of the mean position and within ang_tol_deg of the mean rotation.
    A wrong detection is dropped by this. Also, we do not lock while the headset is moving.
    """

    def __init__(self, size=10, pos_tol_m=0.02, ang_tol_deg=3.0, min_inlier_ratio=0.8):
        if size < 2:
            raise ValueError('size must be at least 2')
        self.size, self.pos_tol_m, self.ang_tol_deg = size, pos_tol_m, ang_tol_deg
        self.min_inliers = max(2, math.ceil(min_inlier_ratio * size))
        self.samples = deque(maxlen=size)

    def add(self, t, q):
        self.samples.append((tuple(t), tuple(q)))

    def _close(self, a, b):
        return math.dist(a[0], b[0]) <= self.pos_tol_m and quaternion_angle_deg(a[1], b[1]) <= self.ang_tol_deg

    def inliers(self):
        """The biggest group of samples that are close to one member. One flipped or wrong detection is dropped."""
        best = []
        for s in self.samples:
            group = [o for o in self.samples if self._close(s, o)]
            if len(group) > len(best):
                best = group
        return best

    def spread(self):
        """(biggest position error in m, biggest rotation error in deg) of the good group, from its mean."""
        group = self.inliers()
        mt, mq = mean_pose(group)
        return (max(math.dist(t, mt) for t, _ in group),
                max(quaternion_angle_deg(q, mq) for _, q in group))

    def result(self):
        if len(self.samples) < self.size:
            return None
        group = self.inliers()
        if len(group) < self.min_inliers:
            return None
        return mean_pose(group)


def up_cos(q):
    """Cosine of the angle between the +z axis of the frame and the world up (+z). q is the rotation of the frame in the world.

    1 = z of the frame points straight up, 0 = horizontal, -1 = upside down.
    quest_world is aligned with gravity. So we can use this to find a tilted robot_base
    (a flipped PnP solution or a wrong mount).
    """
    return quaternion_to_matrix(normalize_quaternion(q))[2][2]


def level_pose(t, q, min_horizontal=0.3):
    """Same position, but the rotation becomes only a yaw around the world up axis (we remove the tilt).

    The yaw is the direction of the +x axis of the frame on the horizontal plane.
    Returns None if the +x axis is almost vertical, because the direction is not clear then.
    """
    r = quaternion_to_matrix(normalize_quaternion(q))
    hx, hy = r[0][0], r[1][0]
    if math.hypot(hx, hy) < min_horizontal:
        return None
    half = 0.5 * math.atan2(hy, hx)
    return tuple(t), (0.0, 0.0, math.sin(half), math.cos(half))
