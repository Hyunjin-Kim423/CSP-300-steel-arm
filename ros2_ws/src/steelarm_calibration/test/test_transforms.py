import math
import unittest
from steelarm_calibration.transforms import normalize_quaternion, quaternion_to_matrix, unity_position_to_ros, validate_rotation

def matmul(a, b): return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))
def transpose(a): return tuple(zip(*a))

class TransformTests(unittest.TestCase):
 def test_known_composition_and_inverse_is_identity(self):
    # 90 degrees around +Z composed with its inverse.
    r = quaternion_to_matrix((0, 0, math.sqrt(.5), math.sqrt(.5)))
    actual = matmul(r, transpose(r))
    for i in range(3):
        for j in range(3): self.assertAlmostEqual(actual[i][j], 1.0 if i == j else 0.0)

 def test_unity_camera_axes_to_ros_optical_axes(self):
    self.assertEqual(unity_position_to_ros((1., 2., 3.)), (1., -2., 3.))

 def test_quaternion_normalizes_and_validates(self):
    self.assertEqual(normalize_quaternion((0, 0, 0, 2)), (0., 0., 0., 1.))
    validate_rotation((0, 0, 0, 3))

 def test_zero_quaternion_rejected(self):
    with self.assertRaises(ValueError): normalize_quaternion((0, 0, 0, 0))

if __name__ == '__main__': unittest.main()
