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
    groundtruth_x = None
    groundtruth_y = None
    des_number_of_images = None # desired number of pose estimation
    recieved_images = 0 # number of received pose estimation
    recieved_subprocess_time = 0 # number of received messages of processing time 
    recieved_pose_local_frame = 0 # number of received pose estimation with frame transformation
    relative_pose = [] # relative pose estimation
    relative_pose_local_frame = [] # received pose estimation with frame transformation
    subprocess_time_str = [] # received processing time
    det_statistics = [] #received cpu/ram usage
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
        
        # single result folder is used to save info of each pose estimation
        self.single_result_folder_path = path.join(self.parent_path, "test_result", "single_result")
        if not path.isdir(self.single_result_folder_path):
            makedirs(self.single_result_folder_path)
        
        # summary folder is used to save summary info (max, min, mean, cpu/ram usage etc) 
        # computed from all the single pose estimation
        self.summary_folder_path = path.join(self.parent_path, "test_result", "summary")
        if not path.isdir(self.summary_folder_path):
            makedirs(self.summary_folder_path)

# pose estimation
def cbDetection(msg, ws_params):
    if(ws_params.recieved_images < ws_params.des_number_of_images and len(msg.detections)>0 ):
        ws_params.relative_pose.append(msg.detections[0])
        print("[POST-PROCESSNG NODE] recorded scene number {} ".format(str(ws_params.recieved_images + 1)))
        ws_params.recieved_images += 1
        if(ws_params.recieved_subprocess_time == ws_params.des_number_of_images 
            and ws_params.recieved_pose_local_frame == ws_params.des_number_of_images
            and ws_params.recieved_images == ws_params.des_number_of_images):
            outputToFile(ws_params)

# pose estimation with frame transformation
def cbVehPoseEuler(msg, ws_params):
    if(ws_params.recieved_pose_local_frame < ws_params.des_number_of_images):
        ws_params.relative_pose_local_frame.append(msg)
        print("[POST-PROCESSNG NODE] recorded scene number (local_fram) {} ".format(str(ws_params.recieved_pose_local_frame + 1)))
        ws_params.recieved_pose_local_frame += 1
        if(ws_params.recieved_subprocess_time == ws_params.des_number_of_images 
            and ws_params.recieved_pose_local_frame == ws_params.des_number_of_images
            and ws_params.recieved_images == ws_params.des_number_of_images):
            outputToFile(ws_params)

# processing time
def cbSubprocessTime(msg, ws_params):
    if(ws_params.recieved_subprocess_time < ws_params.des_number_of_images):  
        ws_params.subprocess_time_str.append(msg)
        print("[POST-PROCESSNG NODE] recorded subprocess time number {} ".format(str(ws_params.recieved_subprocess_time + 1)))
        ws_params.recieved_subprocess_time += 1
        if(ws_params.recieved_subprocess_time == ws_params.des_number_of_images 
            and ws_params.recieved_pose_local_frame == ws_params.des_number_of_images 
            and ws_params.recieved_images == ws_params.des_number_of_images):
            outputToFile(ws_params)

# cpu/ram usage
def cbDetStatistic(msg, ws_params):
    if(msg.node == ws_params.host_name + 'apriltag2_detector_node'):
        if(not(ws_params.recieved_subprocess_time == ws_params.des_number_of_images 
            and ws_params.recieved_pose_local_frame == ws_params.des_number_of_images 
            and ws_params.recieved_images == ws_params.des_number_of_images)):
            ws_params.det_statistics.append(msg)


# deal with all the received messages and output them to single and summary file
def outputToFile(ws_params):
    position = [] #position: camera wrt apriltag
    position_l = [] #position with frame transformation: robot wrt world
    orientation = [] #orientation in quaterion: camera wrt apriltag
    orientation_l = [] #orientation with frame trasformation in euler: robot wrt world
    subprocess_time = []
    subprocess_name = [] 
    subprocess_number = 12 # there are 12 subprocesses of in pose estimation by apriltag2

    #save every single result into .yaml file
    for num in range(0,ws_params.des_number_of_images):
        #pose estimation with frame trasformation
        position_l.append((round(ws_params.relative_pose_local_frame[num].posx*100,5), 
            round(ws_params.relative_pose_local_frame[num].posy*100,5), 
            round(ws_params.relative_pose_local_frame[num].posz*100,5)))
        orientation_l.append((round(ws_params.relative_pose_local_frame[num].rotx,5), 
            round(ws_params.relative_pose_local_frame[num].roty,5), 
            round(ws_params.relative_pose_local_frame[num].rotz,5)))
        
        #pose estimation
        pos_temp = ws_params.relative_pose[num].pose.pose.pose.position
        ori_temp = ws_params.relative_pose[num].pose.pose.pose.orientation
        position.append((round(pos_temp.x*100,5),round(pos_temp.y*100,5),round(pos_temp.z*100,5)))
        orientation.append((round(ori_temp.x,5),round(ori_temp.y,5),round(ori_temp.z,5),round(ori_temp.w,5)))

        #extract name and time consumption of each subprocess in pose estimation from received message
        subprocess_time_str =  ws_params.subprocess_time_str[num].data
        subprocess_time_str_split = filter(None, subprocess_time_str.split('  '))
        sub_time = {}  
        subprocess_time.append([]) 
        for i in range(0, subprocess_number):
            #extract time consumption
            time = round(float(subprocess_time_str_split[3*i+2].split()[0]),5)
            subprocess_time[num].append(time)
            #extract name 
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
            'pos with frame trasformation (cm)':{'x':position_l[num][0],'y':position_l[num][1],'z':position_l[num][2]},
            'ori with frame trasformation (degree)':{'ox':orientation_l[num][0],'oy':orientation_l[num][1],'oz':orientation_l[num][2]},
            'pose (cm)':{'x':position[num][0],'y':position[num][1],'z':position[num][2]},
            'ori (quaterion)':{'x':orientation[num][0],'y':orientation[num][1],'z':orientation[num][2], 'w':orientation[num][3]},
            'subprocess_time (ms)':sub_time
        }
        single_result_file = path.join(ws_params.single_result_folder_path , ws_params.date + '_' + str(num + 1) + '.yaml')
        with open(single_result_file,'a') as f:
            yaml.dump(single_result,f,Dumper=yaml.RoundTripDumper)
    

    #save summary result into ws_params.summary
    #only save pose estimation with frame trasformation in summary file
    x,y,z = zip(*position_l)
    ox,oy,oz = zip(*orientation_l)
    apriltag_output = {
        'x (cm)':{'mean':float('%0.5f' %np.mean(x)),'min':min(x),'max':max(x),'range':float('%0.5f' %(max(x)-min(x))),'variance':float('%0.5f' %np.var(x))},
        'y (cm)':{'mean':float('%0.5f' %np.mean(y)),'min':min(y),'max':max(y),'range':float('%0.5f' %(max(y)-min(y))),'variance':float('%0.5f' %np.var(y))},
        'z (cm)':{'mean':float('%0.5f' %np.mean(z)),'min':min(z),'max':max(z),'range':float('%0.5f' %(max(z)-min(z))),'variance':float('%0.5f' %np.var(z))},
        'ox (degree)':{'mean':float('%0.5f' %np.mean(ox)),'min':min(ox),'max':max(ox),'range':float('%0.5f' %(max(ox)-min(ox))),'variance':float('%0.5f' %np.var(ox))},
        'oy (degree)':{'mean':float('%0.5f' %np.mean(oy)),'min':min(oy),'max':max(oy),'range':float('%0.5f' %(max(oy)-min(oy))),'variance':float('%0.5f' %np.var(oy))},
        'oz (degree)':{'mean':float('%0.5f' %np.mean(oz)),'min':min(oz),'max':max(oz),'range':float('%0.5f' %(max(oz)-min(oz))),'variance':float('%0.5f' %np.var(oz))},
    }
    
    #compute mean, min, max, range and variance of time comsumption of each subprocess 
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

    #unpack received cpu/ram messages
    cpu_ram_info = {}
    cpu_ram_info['node'] = ws_params.det_statistics[0].node
    cpu_ram_info['samples'] = ws_params.det_statistics[0].samples
    cpu_ram_info['cpu_load (%)'] = {'mean':[], 'std':[], 'max':[]}
    cpu_ram_info['virt_mem (MB)'] = {'mean':[], 'std':[], 'max':[]}
    cpu_ram_info['real_mem (MB)'] = {'mean':[], 'std':[], 'max':[]}

    for data in ws_params.det_statistics:
        cpu_ram_info['cpu_load (%)']['mean'].append(round(data.cpu_load_mean,5))
        cpu_ram_info['cpu_load (%)']['std'].append(round(data.cpu_load_std,5))
        cpu_ram_info['cpu_load (%)']['max'].append(round(data.cpu_load_max,5))
        cpu_ram_info['virt_mem (MB)']['mean'].append(round(data.virt_mem_mean/1024/1024,5))
        cpu_ram_info['virt_mem (MB)']['std'].append(round(data.virt_mem_std/1024/1024,5))
        cpu_ram_info['virt_mem (MB)']['max'].append(round(data.virt_mem_max/1024/1024))
        cpu_ram_info['real_mem (MB)']['mean'].append(round(data.real_mem_mean/1024/1024,5))
        cpu_ram_info['real_mem (MB)']['std'].append(round(data.real_mem_std/1024/1024,5))
        cpu_ram_info['real_mem (MB)']['max'].append(round(data.real_mem_max/1024/1024,5))
    
    #compute mean and max of cpu/ram usage
    cpu_load_mean = float(np.mean(cpu_ram_info['cpu_load (%)']['mean']))
    cpu_load_max = max(cpu_ram_info['cpu_load (%)']['max'])
    virt_mem_mean = float(np.mean(cpu_ram_info['virt_mem (MB)']['mean']))
    virt_mem_max = max(cpu_ram_info['virt_mem (MB)']['max'])
    real_mem_mean = float(np.mean(cpu_ram_info['real_mem (MB)']['mean']))
    real_mem_max = max(cpu_ram_info['real_mem (MB)']['max'])

    #print(cpu_ram_info)

    #record mean and max of cpu/ram usage, and the index of max value
    cpu_summary = {
        'total number of cpu/ram messages': len(ws_params.det_statistics),
        'cpu_load (%)':{
            'mean':round(cpu_load_mean,5),
            'max':cpu_load_max,
            'index of max':cpu_ram_info['cpu_load (%)']['max'].index(cpu_load_max)
        },
        'virt_mem (MB)':{
            'mean':round(virt_mem_mean,5),
            'max':virt_mem_max,
            'index of max': cpu_ram_info['virt_mem (MB)']['max'].index(virt_mem_max)
        },
        'real_mem (MB)':{
            'mean':round(real_mem_mean,5),
            'max':real_mem_max,
            'index of max': cpu_ram_info['real_mem (MB)']['max'].index(real_mem_max)
        }
    }

    summary = {
        'decimate':ws_params.decimate,
        'total number of images':ws_params.des_number_of_images,
        'apriltag_output':apriltag_output,
        'time consumption (ms)':time_consumption,
        'cpu/ram':cpu_summary
    }

    summary_file = path.join(ws_params.summary_folder_path, ws_params.date + "_" 
                    + str(ws_params.groundtruth_x) + "_" + str(ws_params.groundtruth_y) + "_"
                    + str(ws_params.decimate) + "_" + str(ws_params.des_number_of_images) + '.yaml')    
    with open(summary_file,'w') as f:
        yaml.dump(summary,f,Dumper=yaml.RoundTripDumper)

    #pusbish the path of summary file
    relative_pose_estimation_analysis.publish(summary_file)
    rospy.signal_shutdown("work down!")



if __name__ == '__main__':
    rospy.loginfo("[POST-PROCESSNG NODE] initializing")
    rospy.init_node('detection_post_processer_node', anonymous=False)

    rospy.loginfo("[POST-PROCESSNG NODE] create analysis result folders at parent_path/test_result")
    ws_params = WorkSpaceParams()
    host_name = rospy.get_namespace() # /robot_name/package_name
    
    ws_params.groundtruth_x = input("groundtruth x:")
    ws_params.groundtruth_y = input("groundtruth y:")

    ws_params.host_name = host_name 
    ws_params.des_number_of_images = rospy.get_param( host_name + "detection_post_processer_node/number_of_images")
    ws_params.decimate = rospy.get_param( host_name + "apriltag2_detector_node/decimate")
    
    sub_img = rospy.Subscriber("tag_detections", AprilTagDetectionArray, cbDetection, ws_params)
    veh_pose_euler = rospy.Subscriber("tag_detections_local_frame", VehiclePoseEuler, cbVehPoseEuler, ws_params)
    sub_time = rospy.Subscriber("subprocess_timings", String, cbSubprocessTime, ws_params)
    detection_statistics = rospy.Subscriber("/node_statistics", NodeStatistics, cbDetStatistic, ws_params)
    relative_pose_estimation_analysis= rospy.Publisher("relative_pose_estimation_analysis", String, queue_size=1, latch=True)
    rospy.spin()
