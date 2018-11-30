#!/usr/bin/env python
import unittest, rosunit
import numpy as np
import tf.transformations as tr
from apriltags2_ros_post_proces.rotation_utils import inverse_homogeneous_transform

class RotationUtils(unittest.TestCase):
    def test_inverse_homogenous_transform(self):
        T = []
        self.assertEqual(0, 0)
    """
    def test_inverse_homogenous_transform2(self):
        self.assertEqual(1, 0)
    """
if __name__ == '__main__':
    rosunit.unitrun('apriltags2_ros', 'test_rotation_utils', RotationUtils)
