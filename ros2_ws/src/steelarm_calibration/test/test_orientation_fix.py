"""Quest orientation correction, gravity gating and levelled lock."""
import math
import unittest

from steelarm_calibration.align_once_logic import level_pose, up_cos
from steelarm_calibration.transforms import quaternion_to_matrix, quest_pose_to_optical


def published(unity_q):
    """What CameraImagePublisher.UnityWorldCameraToRos sends for a Unity camera orientation."""
    x, y, z, w = unity_q
    return (z, -x, y, w)


def axes(q):
    r = quaternion_to_matrix(q)
    col = lambda i: tuple(r[k][i] for k in range(3))
    return col(0), col(1), col(2)          # optical right, down, forward in the world (x fwd, y left, z up)


class QuestOrientationFixTests(unittest.TestCase):
    def assertVec(self, got, want):
        for a, b in zip(got, want):
            self.assertAlmostEqual(a, b, places=9)

    def test_level_camera_looking_along_unity_forward(self):
        right, down, fwd = axes(quest_pose_to_optical(published((0, 0, 0, 1))))
        self.assertVec(fwd, (1, 0, 0))                     # Unity forward = ROS +x
        self.assertVec(down, (0, 0, -1))                   # camera down points at the floor
        self.assertVec(right, (0, -1, 0))                  # Unity right = ROS -y

    def test_turning_right_turns_the_forward_axis_toward_minus_y(self):
        h = math.radians(30) / 2                           # Unity yaw +30 deg about Unity y = turn right
        _, down, fwd = axes(quest_pose_to_optical(published((0, math.sin(h), 0, math.cos(h)))))
        self.assertVec(fwd, (math.cos(math.radians(30)), -math.sin(math.radians(30)), 0))
        self.assertVec(down, (0, 0, -1))

    def test_pitching_up_raises_the_forward_axis(self):
        h = math.radians(20) / 2                           # Unity x rotation is left handed: negative = pitch up
        right, down, fwd = axes(quest_pose_to_optical(published((-math.sin(h), 0, 0, math.cos(h)))))
        self.assertAlmostEqual(fwd[2], math.sin(math.radians(20)), places=9)
        self.assertAlmostEqual(right[2], 0.0, places=9)    # no roll introduced

    def test_published_quaternion_without_the_fix_is_rolled(self):
        # the bug: the published identity is read as an optical frame whose forward axis points up
        _, _, fwd = axes((0.0, 0.0, 0.0, 1.0))
        self.assertVec(fwd, (0, 0, 1))


class GravityTests(unittest.TestCase):
    def test_up_cos(self):
        self.assertAlmostEqual(up_cos((0, 0, 0, 1)), 1.0)
        self.assertAlmostEqual(up_cos((1, 0, 0, 0)), -1.0)                       # rolled 180 deg: upside down
        h = math.radians(60) / 2
        self.assertAlmostEqual(up_cos((math.sin(h), 0, 0, math.cos(h))), 0.5)      # tilted 60 deg

    def test_level_pose_keeps_position_and_heading_and_drops_tilt(self):
        yaw, tilt = math.radians(40), math.radians(25)
        # rotation = Rz(yaw) * Rx(tilt): heading of +x is the yaw, +z is tilted
        qz = (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
        qx = (math.sin(tilt / 2), 0, 0, math.cos(tilt / 2))
        x1, y1, z1, w1 = qz
        x2, y2, z2, w2 = qx
        q = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
        t, ql = level_pose((1.0, 2.0, 3.0), q)
        self.assertEqual(t, (1.0, 2.0, 3.0))
        self.assertAlmostEqual(up_cos(ql), 1.0)
        r = quaternion_to_matrix(ql)
        self.assertAlmostEqual(math.degrees(math.atan2(r[1][0], r[0][0])), 40.0, places=6)

    def test_level_pose_undefined_when_x_is_vertical(self):
        h = math.radians(90) / 2
        self.assertIsNone(level_pose((0, 0, 0), (0, -math.sin(h), 0, math.cos(h))))   # +x pointing straight up/down


if __name__ == '__main__':
    unittest.main()
