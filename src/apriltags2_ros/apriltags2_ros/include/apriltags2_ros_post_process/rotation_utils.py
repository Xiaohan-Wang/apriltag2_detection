import tf.transformations as tr

TILT  = 19

def QuaternionToEuler( x , y , z , w ):
    q0 = w;
    q1 = x;
    q2 = y;
    q3 = z;

    rx = (atan2( 2 * (q2*q3 + q0*q1), (q0*q0 - q1*q1 - q2*q2 + q3*q3)));
    ry = (asin( -2 * (q1*q3 - q0*q2)));
    rz = (atan2( 2 * (q1*q2 + q0*q3), (q0*q0 + q1*q1 - q2*q2 - q3*q3)));

    rx*=180.0/3.141592653589793;
    ry*=180.0/3.141592653589793;
    rz*=180.0/3.141592653589793;

    print ("[QuaternionToEuler] quaternion w: {}\tx: {}\ty: {}\tz: {}\t".format(w,x,y,z))
    print ("[QuaternionToEuler] rx: {}\try: {}\trz: {}\t".format(rx,ry,rz))

    return (rx,ry,rz)

def TagDetection2HomegenousTransformation(q,t):
    # quaterion convention (x,y,z,w)
    """
    q = [0.5, 0, 0, 1]
    t = [1,1,1]
    """
    q = [q.x, q.y, q.z, q.w]
    t = [t.x, t.y, t.z]

    T_q = tr.quaternion_matrix(q)
    T = T_q.copy()
    T[:3, 3] = t

    return T

def inverse_homogeneous_transform(T):
    t = T[0:3,3]
    R = T[0:3,0:3]

    T_inv = tr.identity_matrix()
    R_inv = tr.inverse_matrix(R)

    t_inv = R_inv * -t

    T_inv[0:3,0:3] = R_inv
    T_inv[0:3,3] = t_inv

    return T_inv

def Camz2CamzTilt():
    """
        Returns Homogeneous Transformation that corresponds to Coordinate Transformation (active rotation) from z camera to x camera frame.
        Z camera frame:
            Seeing through camera:
                X -> right
                Y -> down
                Z -> forward
        X camera frame:
            Seeing through camera:
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
    """
    # Initially compensate for the tilt of the camera frame.
    camztilted_T_camz = tr.rotation_matrix(TILT, [1,0,0])
    camz_T_camztilted = inverse_homogeneous_transform(camztilted_T_camz)

    print camz_R_camztilted
    #camz_T_camztilted = tr.identity_matrix()
    #camz_T_camztilted[0:3,0:3] = camz_R_camztilted

    #print camz_T_camztilted
    return camz_T_camztilted
