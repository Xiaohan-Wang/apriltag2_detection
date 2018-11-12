#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from apriltags2_ros.msg import AprilTagDetectionArray
from math import atan2,asin
count = 0
position = []
orientation = []
class Storage:
    publisher = None

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
    return (rx,ry,rz)

def cbDetection(msg):
    global count
    global position
    global orientation
    #print(msg)
    if(count < 10 and len(msg.detections)>0 ):
        pos = msg.detections[0].pose.pose.pose.position
        ori = msg.detections[0].pose.pose.pose.orientation
        position.append((pos.x,pos.y,pos.z))
        orientation.append(QuaternionToEuler(ori.x,ori.y,ori.z,ori.w))
        count = count + 1
        print(count)
    if(count == 10):
        #print(position[0])
        #print(position)
        x,y,z = zip(*position)
        ox,oy,oz = zip(*orientation)
        print('sumary for last %d times detections'  % (count))
        print('position:')
        print('x { mean:%22.18f , min:%22.18f , max:%22.18f , range:%22.18f , variance:%22.18f }' % (np.mean(x),min(x),max(x),max(x)-min(x),np.var(x)) )
        print('y { mean:%22.18f , min:%22.18f , max:%22.18f , range:%22.18f , variance:%22.18f }' % (np.mean(y),min(y),max(y),max(y)-min(y),np.var(y)) )
        print('z { mean:%22.18f , min:%22.18f , max:%22.18f , range:%22.18f , variance:%22.18f }' % (np.mean(z),min(z),max(z),max(z)-min(z),np.var(z)) )
        print('orientation:')
        print('ox { mean:%22.18f , min:%22.18f , max:%22.18f , range:%22.18f , variance:%22.18f }' % (np.mean(ox),min(ox),max(ox),max(ox)-min(ox),np.var(ox)) )
        print('oy { mean:%22.18f , min:%22.18f , max:%22.18f , range:%22.18f , variance:%22.18f }' % (np.mean(oy),min(oy),max(oy),max(oy)-min(oy),np.var(oy)) )
        print('oz { mean:%22.18f , min:%22.18f , max:%22.18f , range:%22.18f , variance:%22.18f }' % (np.mean(oz),min(oz),max(oz),max(oz)-min(oz),np.var(oz)) )
        rospy.signal_shutdown("work down!")

if __name__ == '__main__': 
    rospy.init_node('~detection_post_process', anonymous=False)
    sub_img = rospy.Subscriber("/duckiebot/tag_detections", AprilTagDetectionArray, cbDetection)
    rospy.spin()


