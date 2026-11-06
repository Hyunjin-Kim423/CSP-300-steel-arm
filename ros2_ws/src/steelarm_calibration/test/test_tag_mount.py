"""tag_mount.yaml geometry check. Tag frame follows apriltag_ros 3.4.0 (x viewer-left, y down, z out of face)."""
import os
import unittest
import yaml
from steelarm_calibration.transforms import quaternion_to_matrix, validate_rotation

CFG = os.path.join(os.path.dirname(__file__), '..', 'config', 'tag_mount.yaml')

def load():
    p = yaml.safe_load(open(CFG))['/**']['ros__parameters']
    return p, validate_rotation(p['rotation_xyzw']), p['translation_m']

def apply(R, t, v):  # parent <- child : p_tag = R p_base + t
    return tuple(sum(R[i][k]*v[k] for k in range(3)) + t[i] for i in range(3))

def inverse_apply(R, t, v):  # p_base = R^T (p_tag - t)
    d = [v[i]-t[i] for i in range(3)]
    return tuple(sum(R[k][i]*d[k] for k in range(3)) for i in range(3))

class TagMountTests(unittest.TestCase):
    def test_frames(self):
        p, _, _ = load()
        self.assertEqual((p['parent_frame'], p['child_frame']), ('robot_tag_0', 'robot_base'))

    def test_base_origin_in_tag_frame(self):
        _, R, t = load()
        for a, b in zip(apply(R, t, (0, 0, 0)), (0.0, 0.14, -0.14)): self.assertAlmostEqual(a, b, places=9)  # tag upside down

    def test_tag_centre_is_ahead_and_below_base(self):
        _, R, t = load()
        for a, b in zip(inverse_apply(R, t, (0, 0, 0)), (0.14, 0.0, -0.14)): self.assertAlmostEqual(a, b, places=9)

    def test_point_one_metre_in_front_of_tag_face(self):
        _, R, t = load()   # tag +z = out of face toward camera
        for a, b in zip(inverse_apply(R, t, (0, 0, 1.0)), (1.14, 0.0, -0.14)): self.assertAlmostEqual(a, b, places=9)

    def test_tag_axes_in_base_frame(self):
        _, R, _ = load()   # columns of R^T applied to tag unit vectors
        unit = lambda v: tuple(sum(R[k][i]*v[k] for k in range(3)) for i in range(3))
        for got, want in ((unit((1, 0, 0)), (0, 1, 0)), (unit((0, 1, 0)), (0, 0, 1)), (unit((0, 0, 1)), (1, 0, 0))):  # tag upside down: base z = tag +y (up)
            for a, b in zip(got, want): self.assertAlmostEqual(a, b, places=9)

if __name__ == '__main__': unittest.main()
