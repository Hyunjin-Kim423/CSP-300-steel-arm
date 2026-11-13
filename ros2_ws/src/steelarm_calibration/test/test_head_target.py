import math
import unittest
from steelarm_calibration.alignment import (
    ALIGNMENT_MODES, apply_yaw_translation, check_tree, tf_edges, yaw_deg_to_quaternion)
from steelarm_calibration.head_target_logic import ArrivalAgeTracker, ValidityTracker, evaluate_sample


class ArrivalAgeTests(unittest.TestCase):
    S = 1_000_000_000

    def test_first_sample_has_zero_age(self):
        self.assertEqual(ArrivalAgeTracker().update(5 * self.S, 100 * self.S), 0.0)

    def test_age_grows_while_stamp_is_unchanged(self):
        t = ArrivalAgeTracker()
        t.update(5 * self.S, 100 * self.S)
        self.assertAlmostEqual(t.update(5 * self.S, 100 * self.S + 400_000_000), 0.4)

    def test_new_stamp_resets_age(self):
        t = ArrivalAgeTracker()
        t.update(5 * self.S, 100 * self.S)
        t.update(5 * self.S, 101 * self.S)
        self.assertEqual(t.update(5 * self.S + 100_000_000, 101 * self.S + 50_000_000), 0.0)

    def test_sender_clock_offset_is_ignored(self):
        # stamps 1 s ahead of the local clock must not look stale while samples keep arriving
        t = ArrivalAgeTracker()
        for i in range(20):
            now = 100 * self.S + i * 100_000_000
            self.assertEqual(t.update(now + self.S, now), 0.0)

    def test_reset_forgets_last_sample(self):
        t = ArrivalAgeTracker()
        t.update(5 * self.S, 100 * self.S)
        t.reset()
        self.assertEqual(t.update(5 * self.S, 200 * self.S), 0.0)


class HeadTargetLogicTests(unittest.TestCase):
    def test_fresh_tf_is_valid(self):
        self.assertEqual(evaluate_sample((1.0, 0.2, 0.5), 0.1, 0.3), (True, 'ok'))

    def test_tf_exactly_at_limit_is_valid(self):
        self.assertTrue(evaluate_sample((0, 0, 0), 0.3, 0.3)[0])

    def test_stale_tf_rejected(self):
        valid, reason = evaluate_sample((1.0, 0.0, 0.0), 0.31, 0.3)
        self.assertFalse(valid)
        self.assertIn('stale', reason)

    def test_missing_tf_rejected(self):
        self.assertEqual(evaluate_sample(None, 0.0, 0.3), (False, 'tf unavailable'))

    def test_non_finite_rejected(self):
        self.assertFalse(evaluate_sample((float('nan'), 0.0, 0.0), 0.0, 0.3)[0])
        self.assertFalse(evaluate_sample((0.0, float('inf'), 0.0), 0.0, 0.3)[0])

    def test_tracker_reports_only_state_changes(self):
        t = ValidityTracker()
        changes = [t.update(v) for v in (False, False, True, True, True, False, False, True)]
        self.assertEqual(changes, [True, False, True, False, False, True, False, True])


class ManualAlignmentTests(unittest.TestCase):
    def assertPoint(self, actual, expected):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, places=9)

    def test_identity(self):
        self.assertPoint(apply_yaw_translation((1, 2, 3), 0, 0, 0, 0), (1, 2, 3))

    def test_translation_only(self):
        self.assertPoint(apply_yaw_translation((1, 0, 0), -0.5, 0.25, 0.1, 0), (0.5, 0.25, 0.1))

    def test_yaw_90_maps_world_x_to_base_y(self):
        self.assertPoint(apply_yaw_translation((1, 0, 0), 0, 0, 0, 90), (0, 1, 0))

    def test_yaw_quaternion_is_unit_and_about_z(self):
        x, y, z, w = yaw_deg_to_quaternion(90)
        self.assertEqual((x, y), (0.0, 0.0))
        self.assertAlmostEqual(math.sqrt(z * z + w * w), 1.0)
        self.assertAlmostEqual(z, math.sqrt(0.5))


class TfModeSeparationTests(unittest.TestCase):
    def test_each_mode_is_a_valid_tree(self):
        for mode in ALIGNMENT_MODES:
            check_tree(tf_edges(mode))

    def test_both_modes_have_base_link_reachable_from_camera(self):
        for mode in ALIGNMENT_MODES:
            edges = tf_edges(mode)
            connected = {'quest_camera_optical'}
            for _ in edges:
                for p, c in edges:
                    if p in connected or c in connected:
                        connected |= {p, c}
            self.assertIn('base_link', connected, mode)

    def test_running_both_modes_together_is_rejected(self):
        with self.assertRaises(ValueError):
            check_tree(tf_edges('manual') + tf_edges('tag'))

    def test_two_parents_detected(self):
        with self.assertRaises(ValueError):
            check_tree([('a', 'c'), ('b', 'c')])

    def test_unknown_mode_rejected(self):
        with self.assertRaises(ValueError):
            tf_edges('auto')

    def test_manual_mode_does_not_publish_the_tag_edges(self):
        self.assertNotIn(('robot_base', 'base_link'), tf_edges('manual'))
        self.assertNotIn(('base_link', 'quest_world'), tf_edges('tag'))
