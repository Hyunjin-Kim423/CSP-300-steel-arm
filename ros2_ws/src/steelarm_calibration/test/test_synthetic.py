import unittest
from steelarm_calibration.synthetic import *
class SyntheticTests(unittest.TestCase):
 def test_known_tag_projection_at_one_metre(self):
    pixels=[project_point(p,600,600,320,240) for p in translate(tag_corners(.10),(0,0,1))]
    self.assertEqual(pixels, [(290.,210.),(350.,210.),(350.,270.),(290.,270.)]); self.assertEqual(reprojection_rmse(pixels,pixels),0.)
 def test_invalid_synthetic_size_rejected(self):
    with self.assertRaises(ValueError): tag_corners(0)
 def test_behind_camera_rejected(self):
    with self.assertRaises(ValueError): project_point((0,0,0),1,1,0,0)
