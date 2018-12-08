#!/usr/bin/env python
import unittest, rosunit, rospy
import numpy as np
import math
import tf.transformations as tr
from apriltags2_ros_post_process.rotation_utils import *

class RotationUtils(unittest.TestCase):

    def test_col(self):
        '''
        tests whether a numpy singleton is converted into a column array correctly,
        (technically a numpy matrix of shape (n,1)).
        '''
        v = np.array([1,2,3])
        v_col = col(v)
        self.assertEqual(v_col.shape, (3,1))

    def test_row(self):
        '''
        tests whether a numpy singleton is converted into a row array correctly,
        (technically a numpy matrix of shape (1,n)).
        '''
        v = np.array([1,2,3])
        v_row = row(v)
        self.assertEqual(v_row.shape, (1,3))

    def test_inverse_homogenous_transform_trivial(self):
        '''
        tests whether inverse_homogeneous_transform yields  T = inv(T) when T is the identity matrix.
        '''
        T = tr.identity_matrix()
        T_inv = inverse_homogeneous_transform(T)
        np.testing.assert_almost_equal(T,T_inv)

    def test_inverse_homogenous_transform_non_zero_translation(self):
        '''
        tests whether inverse_homogeneous_transform performs the inversion correctly for T with non-zero translation vector.
        '''
        T = tr.identity_matrix()
        T[0:3,3] = np.array([1,1,1])

        T_inv_expected = tr.identity_matrix()
        T_inv_expected[0:3,3] = np.array([-1,-1,-1])

        T_inv = inverse_homogeneous_transform(T)
        np.testing.assert_almost_equal(T_inv, T_inv_expected)

    def test_inverse_homogenous_transform_translation_rotation(self):
        '''
        tests whether inverse_homogeneous_transform performs the inversion correctly for
        T with non-zero translation vector and a non-identity rotation matrix.
        '''
        # Spesifically tests matrix inversion used in the function.
        a_T_b = tr.identity_matrix()
        a_T_b[0:3,0:3] = np.array([[0,-1,0],[1,0,0],[0,0,1]]) # rotation around z axis by 90 degrees. frame B is rotated with respect to az
        a_T_b[0:3,3] = np.array([1,1,0]) # translation by 1 unit in positive x. translation from from frame A to frame B expressed in frame A.

        b_T_a_exp = tr.identity_matrix()
        b_T_a_exp[0:3,3] = np.matmul(np.array([[0,1,0],[-1,0,0],[0,0,1]]), -np.array([1,1,0]))
        b_T_a_exp[0:3,0:3] = np.array([[0,1,0],[-1,0,0],[0,0,1]]) # R_inv * -t

        T_inv = inverse_homogeneous_transform(a_T_b)
        np.testing.assert_almost_equal(T_inv, b_T_a_exp)

    def test_camzTcamx(self):
        '''
        tests wether camz_T_camx returns homogeneous transformation that expresses camx_p in camz frame.
        '''
        camx_p = np.array([1,2,3,1])
        camz_T_camx = camzTcamx()
        camz_p = np.matmul(camz_T_camx, camx_p)

        camz_p_exp = np.array([-2, -3, 1, 1])

        np.testing.assert_almost_equal(camz_p, camz_p_exp)
    """
    def test_get_robot_pose(self):
        q = [0.9585 0.0263 -0.2431 -0.1467]
        t = [0.016 -0.041 0.295]

        veh_R_tag, veh_t_tag = get_robot_pose(q,t)

    """
    def test_euler_XYZ_from_rotation_matrix(self):
        """
        r in 'rxyz' is 'r'otating frames indicating rotations are applied consecutively with respect to current
        frames' axes, "relative-rotation". Order of rotations are first X, second Y and third Z. Rotation matrix
        composition rule for relative rotations: Rx * Ry * Rz.

        s in 'sxyz' is 's'tationary frames indicating rotations are applied with respect to the coordinate frame
        of the INITIAL frame, "fixed-axis rotation". Rotation matrix composition rule for fixed-axis rotations:
        Rz * Ry * Rx.
        """

        Rx_90 = tr.rotation_matrix(math.pi / 4, [1, 0, 0]) # first, rotate 90 degrees around x axis
        Ry_90 = tr.rotation_matrix(math.pi / 5, [0, 1, 0])  # second, 45 degrees around y axis of the current frame
        Rz_90 = tr.rotation_matrix(math.pi / 6, [0, 0, 1])  # third, 30 degrees around z axis of the current frame

        R1 = np.matmul(Rz_90, Ry_90)
        R = np.matmul(R1, Rx_90)

        euler_angles = tr.euler_from_matrix(R, 'sxyz')
        euler_angles_expected = [math.pi / 4, math.pi / 5, math.pi / 6]

        np.testing.assert_almost_equal(euler_angles, euler_angles_expected)

if __name__ == '__main__':
    rosunit.unitrun('apriltags2_ros', 'test_rotation_utils', RotationUtils)
