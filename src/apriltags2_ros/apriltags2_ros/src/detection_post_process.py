#!/usr/bin/env python
import rospy
import numpy as np
import datetime
from ruamel import yaml
from os import path
from os import makedirs
import tf.transformations as tr

from apriltags2_ros.msg import AprilTagDetectionArray

from apriltags2_ros.rotation_utils import QuaternionToEuler


from math import atan2,asin

class WorkSpaceParams(object):
    des_number_of_images = None
    recieved_images = 0
    position = []
    orientation = []
    single_result_folder_path = None
    summary_folder_path = None
    data = None

    """
    self.camera_x     = self.setupParam("~camera_x", 0.065)
    self.camera_y     = self.setupParam("~camera_y", 0.0)
    self.camera_z     = self.setupParam("~camera_z", 0.11)
    self.camera_theta = self.setupParam("~camera_theta", 19.0)
    self.scale_x     = self.setupParam("~scale_x", 1)
    self.scale_y     = self.setupParam("~scale_y", 1)
    self.scale_z = self.setupParam("~scale_z", 1)
    """
    def __init__(self):
        self.prepareWorkSpace()

    def prepareWorkSpace(self):
        #obtain date and file path
        self.date=datetime.datetime.now().strftime('%Y-%m-%d__%H:%M:%S')
        path_name=path.dirname(__file__)
        self.parent_path=path.dirname(path_name)

        self.single_result_folder_path = path.join(self.parent_path, "test_result", "single_result")
        if not path.isdir(self.single_result_folder_path):
            makedirs(self.single_result_folder_path)

        self.summary_folder_path = path.join(self.parent_path, "test_result", "summary")
        if not path.isdir(self.summary_folder_path):
            makedirs(self.summary_folder_path)

def cbDetection(msg, ws_params):
    date = ws_params.date

    if(ws_params.recieved_images < ws_params.des_number_of_images and len(msg.detections)>0 ):
        pos = msg.detections[0].pose.pose.pose.position
        ori = msg.detections[0].pose.pose.pose.orientation

        #tag_T_camz = toHomegenousTransformation(ori,pos)
        T_camz = TagDetection2HomegenousTransformation(ori,pos)
        camz_T_camztilt = Camz2CamzTilt()
        #T_camx = from_T_to(T_camz, camz_T_camx)
        #T_camz =from_T_to(tag_R_camx, tag_t_camx)
        #T_robot =from_T_to(tag_R_camx, tag_t_camx)
        #ori_degree=QuaternionToEuler(ori.x,ori.y,ori.z,ori.w)

        ws_params.position.append((pos.x,pos.y,pos.z))
        #orientation.append(QuaternionToEuler(ori.x,ori.y,ori.z,ori.w))
        ws_params.orientation.append(ori_degree)

        #save every single result into .yaml file
        single_result={
            'count':ws_params.recieved_images,
            'pos':{'x':pos.x,'y':pos.y,'z':pos.z},
            'ori':{'ox':ori_degree[0],'oy':ori_degree[1],'oz':ori_degree[2]}
        }

        single_result_file = path.join(ws_params.parent_path, "test_result", "single_result", date + '_' + str(ws_params.recieved_images + 1) + '.yaml')
        with open(single_result_file,'a') as f:
            yaml.dump(single_result,f,Dumper=yaml.RoundTripDumper)

        print("[POST-PROCESSNG NODE] recorded scene number {} ".format(str(ws_params.recieved_images + 1)))
        ws_params.recieved_images += 1

    if(ws_params.recieved_images == ws_params.des_number_of_images):
        x,y,z = zip(*ws_params.position)
        ox,oy,oz = zip(*ws_params.orientation)
        #save summary result into .yaml file
        summary={
            'x':{'mean':float(np.mean(x)),'min':min(x),'max':max(x),'range':max(x)-min(x),'variance':float(np.var(x))},
            'y':{'mean':float(np.mean(y)),'min':min(y),'max':max(y),'range':max(y)-min(y),'variance':float(np.var(y))},
            'z':{'mean':float(np.mean(z)),'min':min(z),'max':max(z),'range':max(z)-min(z),'variance':float(np.var(z))},
            'ox':{'mean':float(np.mean(ox)),'min':min(ox),'max':max(ox),'range':max(ox)-min(ox),'variance':float(np.var(ox))},
            'oy':{'mean':float(np.mean(oy)),'min':min(oy),'max':max(oy),'range':max(oy)-min(oy),'variance':float(np.var(oy))},
            'oz':{'mean':float(np.mean(oz)),'min':min(oz),'max':max(oz),'range':max(oz)-min(oz),'variance':float(np.var(oz))},
	    }
        summary_file = path.join(ws_params.parent_path, "test_result", "summary", date + '.yaml')
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

    sub_img = rospy.Subscriber("tag_detections", AprilTagDetectionArray, cbDetection, ws_params)
    rospy.spin()
