#!/usr/bin/env python
import rospy
import numpy as np
import datetime
from ruamel import yaml
from os import path
from os import makedirs
from apriltags2_ros.msg import AprilTagDetectionArray
from std_msgs.msg import String
from ros_statistics_msgs.msg import NodeStatistics
from math import atan2,asin
from apriltags2_ros.msg import VehiclePoseEuler
from apriltags2_ros_post_process.rotation_utils import *

class WorkSpaceParams(object):
    host_name = None
    des_number_of_images = None
    recieved_images = 0 # topic /mete/tag_detections
    recieved_subprocess_time = 0 # topic /mete/subprocess_timings
    recieved_pose_local_frame = 0 #topic tag_detections_local_frame
    relative_pose = [] # relative pose of each detection
    relative_pose_local_frame = [] # publisded by tag_detections_local_frame
    subprocess_time_str = [] # the time of each subprocess of each detection
    det_statistics = [] #topic /node_statistics
    single_result_folder_path = None
    summary_folder_path = None
    date = None
    decimate = None

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
        self.date=datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
        path_name=path.dirname(__file__)
        self.parent_path=path.dirname(path_name)

        self.single_result_folder_path = path.join(self.parent_path, "test_result", "single_result")
        if not path.isdir(self.single_result_folder_path):
            makedirs(self.single_result_folder_path)

        self.summary_folder_path = path.join(self.parent_path, "test_result", "summary")
        if not path.isdir(self.summary_folder_path):
            makedirs(self.summary_folder_path)

def cbDetection(msg, ws_params):
    if(ws_params.recieved_images < ws_params.des_number_of_images and len(msg.detections)>0 ):
        ws_params.relative_pose.append(msg.detections[0])
        print("[POST-PROCESSNG NODE] recorded scene number {} ".format(str(ws_params.recieved_images + 1)))
        ws_params.recieved_images += 1
        if(ws_params.recieved_subprocess_time == ws_params.des_number_of_images 
            and ws_params.recieved_pose_local_frame == ws_params.des_number_of_images
            and ws_params.recieved_images == ws_params.des_number_of_images):
            outputToFile(ws_params)

def cbVehPoseEuler(msg, ws_params):
    if(ws_params.recieved_pose_local_frame < ws_params.des_number_of_images):
        ws_params.relative_pose_local_frame.append(msg)
        print("[POST-PROCESSNG NODE] recorded scene number (local_fram) {} ".format(str(ws_params.recieved_pose_local_frame + 1)))
        ws_params.recieved_pose_local_frame += 1
        if(ws_params.recieved_subprocess_time == ws_params.des_number_of_images 
            and ws_params.recieved_pose_local_frame == ws_params.des_number_of_images
            and ws_params.recieved_images == ws_params.des_number_of_images):
            outputToFile(ws_params)

def cbSubprocessTime(msg, ws_params):
    if(ws_params.recieved_subprocess_time < ws_params.des_number_of_images):  
        ws_params.subprocess_time_str.append(msg)
        print("[POST-PROCESSNG NODE] recorded subprocess time number {} ".format(str(ws_params.recieved_subprocess_time + 1)))
        #print(msg)
        ws_params.recieved_subprocess_time += 1
        if(ws_params.recieved_subprocess_time == ws_params.des_number_of_images 
            and ws_params.recieved_pose_local_frame == ws_params.des_number_of_images 
            and ws_params.recieved_images == ws_params.des_number_of_images):
            outputToFile(ws_params)

def cbDetStatistic(msg, ws_params):
    if(msg.node == ws_params.host_name + 'apriltag2_detector_node'):
        if(not(ws_params.recieved_subprocess_time == ws_params.des_number_of_images 
            and ws_params.recieved_pose_local_frame == ws_params.des_number_of_images 
            and ws_params.recieved_images == ws_params.des_number_of_images)):
            ws_params.det_statistics.append(msg)



def outputToFile(ws_params):
    position = [] #position: camera wrt apriltag
    position_l = [] #position: robot wrt world
    orientation = [] #quaterion: camera wrt apriltag
    orientation_l = [] #euler: robot wrt world
    subprocess_time = []
    subprocess_name = []
    subprocess_number = 12 # if we add the step of relative pose estimation, it should be 12

    #save every single result into .yaml file
    for num in range(0,ws_params.des_number_of_images):
        position_l.append((round(ws_params.relative_pose_local_frame[num].posx,5), 
            round(ws_params.relative_pose_local_frame[num].posy,5), 
            round(ws_params.relative_pose_local_frame[num].posz,5)))
        orientation_l.append((round(ws_params.relative_pose_local_frame[num].rotx,5), 
            round(ws_params.relative_pose_local_frame[num].roty,5), 
            round(ws_params.relative_pose_local_frame[num].rotz,5)))

        pos_temp = ws_params.relative_pose[num].pose.pose.pose.position
        ori_temp = ws_params.relative_pose[num].pose.pose.pose.orientation
        position.append((round(pos_temp.x,5),round(pos_temp.y,5),round(pos_temp.z,5)))
        orientation.append((round(ori_temp.x,5),round(ori_temp.y,5),round(ori_temp.z,5),round(ori_temp.w,5)))

        subprocess_time_str =  ws_params.subprocess_time_str[num].data
        subprocess_time_str_split = filter(None, subprocess_time_str.split('  '))
        sub_time = {}  
        subprocess_time.append([]) 
        for i in range(0, subprocess_number):
                time = round(float(subprocess_time_str_split[3*i+2].split()[0]),5)
                subprocess_time[num].append(time)
                if(num == 0):
                    name = subprocess_time_str_split[3*i+1].strip()
                    subprocess_name.append(name)
                sub_time[subprocess_name[i]]= time
        if (num == 0):
            subprocess_name.append('total time consumption')
        sub_time[subprocess_name[-1]] = round(float(subprocess_time_str_split[-1].split()[0]),5)
        subprocess_time[num].append(sub_time[subprocess_name[-1]])

        single_result = {
            'image ID': num + 1,
            'pos_local_frame (m)':{'x':position_l[num][0],'y':position_l[num][1],'z':position_l[num][2]},
            'ori_local_frame (degree)':{'ox':orientation_l[num][0],'oy':orientation_l[num][1],'oz':orientation_l[num][2]},
            'pose (m)':{'x':position[num][0],'y':position[num][1],'z':position[num][2]},
            'ori (quaterion)':{'x':orientation[num][0],'y':orientation[num][1],'z':orientation[num][2], 'w':orientation[num][3]},
            'subprocess_time (ms)':sub_time
        }
        single_result_file = path.join(ws_params.single_result_folder_path , ws_params.date + '_' + str(num + 1) + '.yaml')

        with open(single_result_file,'a') as f:
            yaml.dump(single_result,f,Dumper=yaml.RoundTripDumper)
    
    #save summary result into ws_params.summary
    x,y,z = zip(*position_l)
    ox,oy,oz = zip(*orientation_l)

    apriltag_output = {
        'x (m)':{'mean':float('%0.5f' %np.mean(x)),'min':min(x),'max':max(x),'range':float('%0.5f' %(max(x)-min(x))),'variance':float('%0.5f' %np.var(x))},
        'y (m)':{'mean':float('%0.5f' %np.mean(y)),'min':min(y),'max':max(y),'range':float('%0.5f' %(max(y)-min(y))),'variance':float('%0.5f' %np.var(y))},
        'z (m)':{'mean':float('%0.5f' %np.mean(z)),'min':min(z),'max':max(z),'range':float('%0.5f' %(max(z)-min(z))),'variance':float('%0.5f' %np.var(z))},
        'ox (degree)':{'mean':float('%0.5f' %np.mean(ox)),'min':min(ox),'max':max(ox),'range':float('%0.5f' %(max(ox)-min(ox))),'variance':float('%0.5f' %np.var(ox))},
        'oy (degree)':{'mean':float('%0.5f' %np.mean(oy)),'min':min(oy),'max':max(oy),'range':float('%0.5f' %(max(oy)-min(oy))),'variance':float('%0.5f' %np.var(oy))},
        'oz (degree)':{'mean':float('%0.5f' %np.mean(oz)),'min':min(oz),'max':max(oz),'range':float('%0.5f' %(max(oz)-min(oz))),'variance':float('%0.5f' %np.var(oz))},
    }

    subprocess_time_comsumption=zip(*subprocess_time)
    time_consumption = {}
    for i in range(0, subprocess_number + 1): # subprocesses + total time consumption
        time_consumption[subprocess_name[i]] = {
            'mean':float('%0.5f' %np.mean(subprocess_time_comsumption[i])),
            'min':min(subprocess_time_comsumption[i]),
            'max':max(subprocess_time_comsumption[i]),
            'range':float('%0.5f' %(max(subprocess_time_comsumption[i])-min(subprocess_time_comsumption[i]))),
            'variance':float('%0.5f' %np.var(subprocess_time_comsumption[i]))
        }

    #cpu/ram
    cpu_summary = {}
    cpu_summary['node'] = ws_params.det_statistics[0].node
    cpu_summary['samples'] = ws_params.det_statistics[0].samples
    cpu_summary['cpu_load'] = {'mean':[], 'std':[], 'max':[]}
    cpu_summary['virt_mem'] = {'mean':[], 'std':[], 'max':[]}
    cpu_summary['real_mem'] = {'mean':[], 'std':[], 'max':[]}

    for data in ws_params.det_statistics:
        cpu_summary['cpu_load']['mean'].append(round(data.cpu_load_mean/1000000,5))
        cpu_summary['cpu_load']['std'].append(round(data.cpu_load_std/1000000,5))
        cpu_summary['cpu_load']['max'].append(round(data.cpu_load_max/1000000,5))
        cpu_summary['virt_mem']['mean'].append(round(data.virt_mem_mean/1000000,5))
        cpu_summary['virt_mem']['std'].append(round(data.virt_mem_std/1000000,5))
        cpu_summary['virt_mem']['max'].append(round(data.virt_mem_max/1000000))
        cpu_summary['real_mem']['mean'].append(round(data.real_mem_mean/1000000,5))
        cpu_summary['real_mem']['std'].append(round(data.real_mem_std/1000000,5))
        cpu_summary['real_mem']['max'].append(round(data.real_mem_max/1000000,5))

    summary = {
        'decimate':ws_params.decimate,
        'total number of images':ws_params.des_number_of_images,
        'apriltag_output':apriltag_output,
        'time consumption (ms)':time_consumption,
        'cpu/ram (MB)':cpu_summary
    }

    summary_file = path.join(ws_params.summary_folder_path, ws_params.date + "_" + str(ws_params.decimate) + "_" + str(ws_params.des_number_of_images) + '.yaml')    
    with open(summary_file,'w') as f:
        yaml.dump(summary, f, Dumper=yaml.RoundTripDumper)
    rospy.signal_shutdown("work down!")



if __name__ == '__main__':
    rospy.loginfo("[POST-PROCESSNG NODE] initializing")
    rospy.init_node('detection_post_processer_node', anonymous=False)

    rospy.loginfo("[POST-PROCESSNG NODE] create analysis result folders at parent_path/test_result")
    ws_params = WorkSpaceParams()
    host_name = rospy.get_namespace() # /robot_name/package_name
    
    ws_params.host_name = host_name 
    ws_params.des_number_of_images = rospy.get_param( host_name + "detection_post_processer_node/number_of_images")
    ws_params.decimate = rospy.get_param( host_name + "apriltag2_detector_node/decimate")
    
    sub_img = rospy.Subscriber("tag_detections", AprilTagDetectionArray, cbDetection, ws_params)
    veh_pose_euler = rospy.Subscriber("tag_detections_local_frame", VehiclePoseEuler, cbVehPoseEuler, ws_params)
    sub_time = rospy.Subscriber("subprocess_timings", String, cbSubprocessTime, ws_params)
    detection_statistics = rospy.Subscriber("/node_statistics", NodeStatistics, cbDetStatistic, ws_params)
    rospy.spin()
