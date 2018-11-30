#!/usr/bin/env python
import unittest, rosunit
import numpy as np

class TestPostProcessor(unittest.TestCase):
    def test_inverse_homogenous_transform(self):
        self.assertEqual(0, 0)

if __name__ == '__main__':
    rosunit.unitrun('apriltags2_ros', 'test_post_processing', TestPostProcessor)
