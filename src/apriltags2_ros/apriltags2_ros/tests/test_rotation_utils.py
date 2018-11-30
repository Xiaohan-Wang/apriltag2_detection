#!/usr/bin/env python
import unittest, rosunit, rospy
import numpy as np
import tf.transformations as tr
from apriltags2_ros_post_process.rotation_utils import col, row, inverse_homogeneous_transform

class RotationUtils(unittest.TestCase):

    def test_col(self):
        '''
        Convert a numpy singleton into a column array
        '''
        v = np.array([1,2,3])
        v_col = col(v)
        self.assertEqual(v_col.shape, (3,1))

    def test_row(self):
        '''
        Convert a numpy singleton into a row array
        '''
        v = np.array([1,2,3])
        v_row = row(v)
        self.assertEqual(v_row.shape, (1,3))

    def test_inverse_homogenous_transform_trivial(self):
        T = tr.identity_matrix()
        T_inv = inverse_homogeneous_transform(T)
        np.testing.assert_almost_equal(T,T_inv)

    def test_inverse_homogenous_transform_non_zero_translation(self):
        T = tr.identity_matrix()
        T[0:3,3] = np.array([1,1,1])

        T_inv_expected = tr.identity_matrix()
        T_inv_expected[0:3,3] = np.array([-1,-1,-1])

        T_inv = inverse_homogeneous_transform(T)
        np.testing.assert_almost_equal(T_inv, T_inv_expected)

    """
    def test_inverse_homogenous_transform_translation_combined(self):
        T = tr.identity_matrix()
        T = np.array([[0,1,0],[-1,0,0],[0,0,0]]) # rotation around z axis by 90 degrees
        T[0:3,3] = np.array([1,0,0]) # shifted by 1 unit in positive x

        T_inv_expected = tr.identity_matrix()
        T_inv_expected[0:3,3] = np.array([-1,-1,-1])

        T_inv = inverse_homogeneous_transform(T)
        np.testing.assert_almost_equal(T_inv, T_inv_expected)


    def test_inverse_homogenous_transform2(self):
        self.assertEqual(1, 0)
    """
if __name__ == '__main__':
    rosunit.unitrun('apriltags2_ros', 'test_rotation_utils', RotationUtils)
