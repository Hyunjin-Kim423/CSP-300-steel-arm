"""tag_align_once maths: pose inversion, averaging, stability gate, tag_once TF tree."""
import math
import unittest

from steelarm_calibration.align_once_logic import StableWindow, invert_pose, mean_pose, quaternion_angle_deg
from steelarm_calibration.alignment import check_tree, tf_edges
from steelarm_calibration.transforms import quaternion_to_matrix


def apply(t, q, p):
    r = quaternion_to_matrix(q)
    return tuple(sum(r[i][k] * p[k] for k in range(3)) + t[i] for i in range(3))


def yaw_q(deg):
    h = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


class InvertTests(unittest.TestCase):
    def test_round_trip(self):
        t, q = (0.3, -1.2, 0.7), (0.5, -0.5, 0.5, 0.5)
        it, iq = invert_pose(t, q)
        for a, b in zip(apply(it, iq, apply(t, q, (0.4, 0.1, -0.9))), (0.4, 0.1, -0.9)):
            self.assertAlmostEqual(a, b, places=9)

    def test_known_yaw(self):
        it, iq = invert_pose((1.0, 0.0, 0.0), yaw_q(90))  # child origin at (1,0,0), rotated 90 deg
        for a, b in zip(apply(it, iq, (0.0, 0.0, 0.0)), (0.0, 1.0, 0.0)):  # parent origin seen from child
            self.assertAlmostEqual(a, b, places=9)


class MeanTests(unittest.TestCase):
    def test_sign_flipped_quaternion_is_the_same_rotation(self):
        q = yaw_q(30)
        _, m = mean_pose([((0, 0, 0), q), ((0, 0, 0), tuple(-v for v in q))])
        self.assertAlmostEqual(quaternion_angle_deg(m, q), 0.0, places=6)

    def test_angle(self):
        self.assertAlmostEqual(quaternion_angle_deg(yaw_q(0), yaw_q(10)), 10.0, places=6)


class WindowTests(unittest.TestCase):
    def feed(self, w, n, jitter=0.0, jitter_deg=0.0):
        for i in range(n):
            s = (-1) ** i
            w.add((1.0 + s * jitter, 0.0, 0.0), yaw_q(20 + s * jitter_deg))

    def test_not_ready_until_full(self):
        w = StableWindow(5)
        self.feed(w, 4)
        self.assertIsNone(w.result())
        self.feed(w, 1)
        self.assertIsNotNone(w.result())

    def test_position_jitter_blocks_lock(self):
        w = StableWindow(6, pos_tol_m=0.02)
        self.feed(w, 6, jitter=0.05)
        self.assertIsNone(w.result())

    def test_angle_jitter_blocks_lock(self):
        w = StableWindow(6, ang_tol_deg=3.0)
        self.feed(w, 6, jitter_deg=10.0)
        self.assertIsNone(w.result())

    def test_two_outliers_in_ten_are_ignored(self):
        w = StableWindow(10)                     # needs 8 of 10 to agree
        w.add((2.0, 0.5, 0.0), yaw_q(170))       # flipped / wrong detections
        w.add((0.3, -1.0, 0.4), yaw_q(-60))
        self.feed(w, 8)
        t, q = w.result()
        self.assertAlmostEqual(t[0], 1.0, places=6)
        self.assertAlmostEqual(quaternion_angle_deg(q, yaw_q(20)), 0.0, places=6)

    def test_three_outliers_in_ten_block_lock(self):
        w = StableWindow(10)
        for i in range(3):
            w.add((2.0 + i, 0.5, 0.0), yaw_q(170))
        self.feed(w, 7)
        self.assertIsNone(w.result())

    def test_outlier_does_not_pull_the_mean(self):
        w = StableWindow(5)                      # needs 4 of 5
        w.add((9.0, 9.0, 9.0), yaw_q(90))
        self.feed(w, 4)
        t, q = w.result()
        self.assertAlmostEqual(t[0], 1.0, places=9)
        self.assertAlmostEqual(t[1], 0.0, places=9)


class TreeTests(unittest.TestCase):
    def test_tag_once_is_a_valid_tree(self):
        check_tree(tf_edges('tag_once'))

    def test_tag_once_has_no_alias_edge(self):
        self.assertNotIn(('robot_base', 'base_link'), tf_edges('tag_once'))

    def test_tag_once_cannot_run_with_tag(self):
        with self.assertRaises(ValueError):  # robot_base -> base_link + base_link -> quest_world closes a loop
            check_tree(tf_edges('tag_once') + tf_edges('tag'))


if __name__ == '__main__':
    unittest.main()
