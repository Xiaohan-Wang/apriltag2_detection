import tf.transformations as tr
import numpy as np
import math

TILT_ANGLE = 18 * math.pi / 180.0


def col(v):
    """ Convert a numpy singleton into a column array """
    return np.expand_dims(v, axis=1)
def row(v):
    """ Convert a numpy singleton into a row array """
    return np.expand_dims(v, axis=0)


def camztiltedTtag(q,t):
    """
    Expresesses the apriltag2_ros output (message includes a quaterion and a translation vector)
    in homogeneous transform matrix.
    """
    # quaterion convention (x,y,z,w)
    T_q = tr.quaternion_matrix(q)
    T = T_q.copy()
    T[:3, 3] = t

    return T

def inverse_homogeneous_transform(T):
    """ Inverts a homogeneous transformation matrix using the formula 2.92 of [1]. """
    t = T[0:3,3]
    R = T[0:3,0:3]

    T_inv = tr.identity_matrix()
    R_inv = np.transpose(R)

    t_inv = np.matmul(R_inv,-t)

    T_inv[0:3,0:3] = R_inv
    T_inv[0:3,3] = t_inv

    return T_inv

def camztiltedTcamz_BARIS(TILT_ANGLE):
    """ Returns homogeneous transformation that expresses a camz_p in camz frame. """
    d = 0.05

    camztilted_T_camz = tr.rotation_matrix(TILT_ANGLE, [1,0,0])
    t = np.array([0, d*(1 - math.cos(TILT_ANGLE)), d*math.sin(TILT_ANGLE)])
    camztilted_T_camz[0:3,3] = t

    return camztilted_T_camz

def camztiltedTcamz(TILT_ANGLE):
    """ Returns homogeneous transformation that expresses a camz_p in camz frame. """
    camztilted_T_camz = tr.rotation_matrix(TILT_ANGLE, [1,0,0])

    return camztilted_T_camz

def camzTcamztilted():
    """ Returns homogeneous transformation that expresses camztilted_p in camz frame. """
    camztilted_T_camz = camztiltedTcamz(TILT_ANGLE)
    camz_T_camztilted = inverse_homogeneous_transform(camztilted_T_camz)

    return camz_T_camztilted

def camzTcamx():
    """ Returns homogeneous transformation that expresses camx_p in camz frame. """
    T1 = tr.rotation_matrix(math.pi / 2, [1,0,0])
    T2 = tr.rotation_matrix(math.pi / 2, [0,0,1])
    camz_T_camx = np.matmul(T1, T2)
    return camz_T_camx

def camxTcamz():
    """ Returns homogeneous transformation that expresses camz_p in camx frame. """
    camz_T_camx = camzTcamx()
    camx_T_camz = inverse_homogeneous_transform(camz_T_camx)
    return camx_T_camz

def vehTcamx(tOvehOcamx):
    """ Returns homogeneous transformation that expresses camx_p in vehicle (robot) frame. """
    veh_T_camx = tr.translation_matrix(tOvehOcamx)
    return veh_T_camx

def camxTveh(tOvehOcamx):
    """ Returns homogeneous transformation that expresses veh_p in camx frame. """
    veh_T_camx = vehTcamx(tOvehOcamx)
    camx_T_veh = inverse_homogeneous_transform(veh_T_camx)
    return camx_T_veh

def tagTworld():
    """ Returns homogeneous transformation that expresses world_p in tag frame. """
    T_x = tr.rotation_matrix(-math.pi / 2, [1, 0, 0])
    T_z = tr.rotation_matrix(math.pi / 2, [0, 0, 1])

    tag_T_world = np.matmul(T_x, T_z)

    return tag_T_world

def robot_pose_in_word_frame(q_at,t_at):
    """
    expresses the apriltags2_ros output in robot cf.

    Args:
        q_at (numpy.array): quaternion representing relative orientation of camera frame with respect to tag frame.
        t_at (numpy.array): translation vector from cameras cf to tags cf expressed in camera cf.

    Returns:
        veh_R_world (numpy.array): rotation matrix orientation that expresses world_p in robot cf.
        veh_t_world (numpy.array): translation vector from robots cf to apriltag's cf expressed in robot cf.
    """

    tOvehOcamx = np.array([0.10,0.0,0.05])

    camztilted_T_tag = camztiltedTtag(q_at,t_at)
    camz_T_camztilted = camzTcamztilted()
    camx_T_camz = camxTcamz()
    veh_T_camx = vehTcamx(tOvehOcamx)

    D1 = np.matmul(veh_T_camx,camx_T_camz)
    D2 = np.matmul(D1, camz_T_camztilted)

    veh_T_tag = np.matmul(D2, camztilted_T_tag)

    tag_T_world = tagTworld()
    veh_T_world = np.matmul(veh_T_tag, tag_T_world)

    veh_R_world = veh_T_world[0:3, 0:3]
    veh_t_world= veh_T_world[0:3, 3]

    return veh_R_world, veh_t_world

'''
[NOMENCLATURE]

Z camera frame:
    seeing what camera sees:
        X -> right
        Y -> down
        Z -> forward
X camera frame:
    seeing what camera sees:
        X -> forward
        Y -> left
        Z -> up

endFrame_Tp_initFrame:
Represents passive Transformation, i.e represented vector remains the same wrt to a fixed world frame.
The matrix describes how the basis vectors of are related to each other, spesifically it translates
a vector expressed in initFrame into a vector expressed in endFrame.

endFrame_R_initFrame:
represents relative orientation of endFrame wrt. initFrame, and is obtained by a series of consecutive relative rotations with rotations post-multiplied:
R = R1 * R2 * ...

endFrame_t_initFrame:
represent the position vector from the origin of initFrame to the origin of endFrame expressend in initFrame

FRAME_p:
position vector p expressed in FRAME frame, i.e  WORLD_p means p is in WORLD coordinate frame.

[ABBREVIATIONS]

cf: coordinate frame
'''

'''
[RESOURCES]

[1] "Robot Dynamics and Control", Second Edition, Mark W. Spong, Seth Hutchinson, and M. Vidyasagar.

[2] ROS TF library for python: https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py
'''

if __name__ == '__main__':
    q = [0.924, -0.082, 0.350, -0.130]
    t = [-0.011, 0.07, 0.44]

    # Quaternions ix + jy + kz + w are represented as [x, y, z, w]
    veh_R_world, veh_t_world = robot_pose_in_word_frame(q, t)

    euler_angles_w = tr.euler_from_matrix(veh_R_world, 'sxyz')
    euler_angles_w_np = np.asarray(euler_angles_w)
    euler_angles_w_np *= 180/math.pi

    print "Selcuk"