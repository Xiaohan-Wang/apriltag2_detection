import tf.transformations as tr
import numpy as np
import rospy
import math

TILT_ANGLE = 18 * math.pi / 180.0


def col(v):
    '''
    Convert a numpy singleton into a column array
    '''
    return np.expand_dims(v, axis=1)
def row(v):
    '''
    Convert a numpy singleton into a row array
    '''
    return np.expand_dims(v, axis=0)


def camztiltedTtag(q,t):
    '''
    Expresesses the apriltag2_ros output (message includes a quaterion and a translation vector)
    in homogeneous transform matrix.
    '''
    # quaterion convention (x,y,z,w)

    """
    q = [q.x, q.y, q.z, q.w]
    t = [t.x, t.y, t.z]
    """

    T_q = tr.quaternion_matrix(q)
    T = T_q.copy()
    T[:3, 3] = t

    return T

def inverse_homogeneous_transform(T):
    '''
    Inverts a homogeneous transformation matrix using the formula 2.92 of [1].
    '''
    t = T[0:3,3]
    R = T[0:3,0:3]

    T_inv = tr.identity_matrix()
    R_inv = np.transpose(R)

    t_inv = np.matmul(R_inv,-t)

    T_inv[0:3,0:3] = R_inv
    T_inv[0:3,3] = t_inv

    return T_inv

def camztiltedTcamz_BARIS(TILT_ANGLE):
    '''
    Returns homogeneous transformation that expresses a camz_p in camz frame.
    '''
    d = 0.05

    camztilted_T_camz = tr.rotation_matrix(TILT_ANGLE, [1,0,0])
    t = np.array([0, d*(1 - math.cos(TILT_ANGLE)), d*math.sin(TILT_ANGLE)])
    camztilted_T_camz[0:3,3] = t

    return camztilted_T_camz

def camztiltedTcamz(TILT_ANGLE):
    '''
    Returns homogeneous transformation that expresses a camz_p in camz frame.
    '''
    camztilted_T_camz = tr.rotation_matrix(TILT_ANGLE, [1,0,0])

    return camztilted_T_camz

def camzTcamztilted():
    '''
    Returns homogeneous transformation that expresses camztilted_p in camz frame.
    '''
    camztilted_T_camz = camztiltedTcamz(TILT_ANGLE)
    camz_T_camztilted = inverse_homogeneous_transform(camztilted_T_camz)

    return camz_T_camztilted

def camzTcamx():
    '''
    Returns homogeneous transformation that expresses camx_p in camz frame.
    '''
    T1 = tr.rotation_matrix(math.pi / 2, [1,0,0])
    T2 = tr.rotation_matrix(math.pi / 2, [0,0,1])
    camz_T_camx = np.matmul(T1, T2)
    return camz_T_camx

def camxTcamz():
    '''
    Returns homogeneous transformation that expresses camz_p in camx frame.
    '''
    camz_T_camx = camzTcamx()
    camx_T_camz = inverse_homogeneous_transform(camz_T_camx)
    return camx_T_camz

def vehTcamx(tOvehOcamx):
    veh_T_camx = tr.translation_matrix(tOvehOcamx)
    return veh_T_camx

def camxTveh(tOvehOcamx):
    veh_T_camx = vehTcamx(tOvehOcamx)
    camx_T_veh = inverse_homogeneous_transform(veh_T_camx)
    return camx_T_veh

def get_robot_pose(q_at,t_at):
    tOvehOcamx = np.array([0.10,0.0,0.05])

    camztilted_T_tag = camztiltedTtag(q_at,t_at)
    camztilted_T_camz = camztiltedTcamz(TILT_ANGLE)
    camz_T_camztilted = camzTcamztilted()
    camx_T_camz = camxTcamz()
    veh_T_camx = vehTcamx(tOvehOcamx)

    #Z1 = np.matmul(camz_T_camztilted, camztilted_T_tag)
    #Z2 = np.matmul(camztilted_T_camz, camztilted_T_tag)

    D1 = np.matmul(veh_T_camx,camx_T_camz)
    #D2 = np.matmul(D1, camztilted_T_camz)
    D2 = np.matmul(D1, camz_T_camztilted)
    veh_T_tag = np.matmul(D2,camztilted_T_tag)

    veh_R_tag = veh_T_tag[0:3, 0:3]
    veh_t_tag = veh_T_tag[0:3, 3]

    """
    
    T1 = camztilted_T_tag * camz_T_camztilted
    T2 = T1 * camx_T_camz
    veh_T_tag =  T2 * veh_T_camx

    """

    """
    camztilted_T_tag = camztiltedTtag(q_at, t_at)
    camztilted_T_camz = camztiltedTcamz(TILT_ANGLE)
    camz_T_camx = camzTcamx()
    camx_T_veh = camxTveh(tOvehOcamx)

    T1 = camztilted_T_tag * camztilted_T_camz
    T2 = T1 * camz_T_camx
    tag_T_veh = T2 * camx_T_veh
    veh_R_tag = tag_T_veh[0:3, 0:3]
    veh_t_tag = tag_T_veh[0:3, 3]
    """

    return veh_R_tag, veh_t_tag

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
'''

'''
Resources:

[1] "Robot Dynamics and Control", Second Edition, Mark W. Spong, Seth Hutchinson, and M. Vidyasagar.

[2] ROS TF library for python: https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py
'''

if __name__ == '__main__':
    """
    q = [0.9585, 0.0263, -0.2431, -0.1467]
    t = [0.016, -0.041, 0.295]
    """
    q = [0.965, -0.0118, -0.255, 0.0607]
    t = [-0.011, 0.07, 0.44]

    veh_R_tag, veh_t_tag = get_robot_pose(q, t)
    euler_angles = tr.euler_from_matrix(veh_R_tag, 'rxyz')
    euler_angles_np = np.asarray(euler_angles)
    euler_angles_np *= 180/math.pi
    print "Selcuk"