#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import datetime
from ruamel import yaml
from os import path
from os import makedirs

from apriltags2_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String
from math import atan2,asin

class WorkSpaceParams(object):
    des_number_of_images = None
    recieved_images = 0 # topic /mete/tag_detections
    recieved_subprocess_time = 0 # topic /mete/subprocess_timings
    relative_pose = [] # relative pose of each detection
    subprocess_time = [] # the time of each subprocess of each detection
    single_result_folder_path = None
    summary_folder_path = None
    date = None
    decimate = None

    def __init__(self):
        self.prepareWorkSpace()

    def prepareWorkSpace(self):
        #obtain date and file path
        self.date=datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
        path_name=path.dirname(__file__)
        self.parent_path=path.dirname(path_name)

        self.single_result_folder_path = path.join(self.parent_path, "test_result", "single_result")
        if not path.isdir(self.single_result_folder_path):
            makedirs(self.single_result_folder_path)

        self.summary_folder_path = path.join(self.parent_path, "test_result", "summary")
        if not path.isdir(self.summary_folder_path):
            makedirs(self.summary_folder_path)

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

def cbDetection(msg, ws_params):
    if(ws_params.recieved_images < ws_params.des_number_of_images and len(msg.detections)>0 ):
        ws_params.relative_pose.append(msg.detections[0])
        print("[POST-PROCESSNG NODE] recorded scene number {} ".format(str(ws_params.recieved_images + 1)))
        ws_params.recieved_images += 1
        if(ws_params.recieved_subprocess_time == ws_params.des_number_of_images and ws_params.recieved_images == ws_params.des_number_of_images):
            outputToFile(ws_params)


def cbSubprocessTime(msg, ws_params):
    if(ws_params.recieved_subprocess_time < ws_params.des_number_of_images):  
        ws_params.subprocess_time.append(msg)
        print("[POST-PROCESSNG NODE] recorded subprocess time number {} ".format(str(ws_params.recieved_subprocess_time + 1)))
        #print(msg)
        ws_params.recieved_subprocess_time += 1
        if(ws_params.recieved_subprocess_time == ws_params.des_number_of_images and ws_params.recieved_images == ws_params.des_number_of_images):     
            outputToFile(ws_params)


def outputToFile(ws_params):
    position = []
    orientation = []
    for num in range(0,ws_params.des_number_of_images):
        pos = ws_params.relative_pose[num].pose.pose.pose.position
        ori = ws_params.relative_pose[num].pose.pose.pose.orientation
        ori_degree=QuaternionToEuler(ori.x,ori.y,ori.z,ori.w)
       
        position.append((pos.x,pos.y,pos.z))
        orientation.append(ori_degree)
        
        #save every single result into .yaml file
        single_result={
            'count': num + 1,
            'pos':{'x':pos.x,'y':pos.y,'z':pos.z},
            'ori':{'ox':ori_degree[0],'oy':ori_degree[1],'oz':ori_degree[2]},
            'subprocess_time':ws_params.subprocess_time[num].data
        }
        single_result_file = path.join(ws_params.single_result_folder_path , ws_params.date + '_' + str(num + 1) + '.yaml')
        #print(single_result)   
        with open(single_result_file,'a') as f:
            yaml.dump(single_result,f,Dumper=yaml.RoundTripDumper)
   
    
    x,y,z = zip(*position)
    ox,oy,oz = zip(*orientation)
    #save summary result into ws_params.summary
    summary={
        'decimate':ws_params.decimate,
        'total':ws_params.des_number_of_images,
        'x':{'mean':float(np.mean(x)),'min':min(x),'max':max(x),'range':max(x)-min(x),'variance':float(np.var(x))},
        'y':{'mean':float(np.mean(y)),'min':min(y),'max':max(y),'range':max(y)-min(y),'variance':float(np.var(y))},
        'z':{'mean':float(np.mean(z)),'min':min(z),'max':max(z),'range':max(z)-min(z),'variance':float(np.var(z))},
        'ox':{'mean':float(np.mean(ox)),'min':min(ox),'max':max(ox),'range':max(ox)-min(ox),'variance':float(np.var(ox))},
        'oy':{'mean':float(np.mean(oy)),'min':min(oy),'max':max(oy),'range':max(oy)-min(oy),'variance':float(np.var(oy))},
        'oz':{'mean':float(np.mean(oz)),'min':min(oz),'max':max(oz),'range':max(oz)-min(oz),'variance':float(np.var(oz))},
        }
    summary_file = path.join(ws_params.summary_folder_path, ws_params.date + "_" + str(ws_params.decimate) + "_" + str(ws_params.des_number_of_images) + '.yaml')    
    with open(summary_file,'w') as f:
        yaml.dump(summary,f,Dumper=yaml.RoundTripDumper)
    rospy.signal_shutdown("work down!")


if __name__ == '__main__':
    rospy.loginfo("[POST-PROCESSNG NODE] initializing")
    rospy.init_node('detection_post_processer_node', anonymous=False)

    rospy.loginfo("[POST-PROCESSNG NODE] create analysis result folders at parent_path/test_result")
    ws_params = WorkSpaceParams()
    host_name = rospy.get_namespace() # /robot_name/package_name
    #print package_ws

    ws_params.des_number_of_images = rospy.get_param( host_name + "detection_post_processer_node/number_of_images")
    ws_params.decimate = rospy.get_param( host_name + "apriltag2_detector_node/decimate")
    
    sub_img = rospy.Subscriber("tag_detections", AprilTagDetectionArray, cbDetection, ws_params)
    sub_time = rospy.Subscriber("subprocess_timings", String, cbSubprocessTime, ws_params)
    rospy.spin()
