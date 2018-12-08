#!/usr/bin/env python
import rospy
import numpy as np
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros_post_process.rotation_utils import *
from apriltags2_ros.msg import VehiclePoseEuler

class ToLocalPose:
    def __init__(self):
        """
        listens to pose estimation returned by apriltag2_ros node and converts it
        into robot pose expressed in the global frame
        """

        host_package = rospy.get_namespace() # as defined by <group> in launch file
        node_name = 'publish_detections_in_local_frame' # node name , as defined in launch file
        host_package_node = host_package + node_name
        veh = host_package.split('/')[1]
        # Subscriber
        sub_topic_name =  '/' + veh + '/tag_detections'
        self.sub_img = rospy.Subscriber(sub_topic_name, AprilTagDetectionArray, self.cbDetection)

        # Publisher
        pub_topic_name = host_package_node + '/tag_detections_local_frame'
        self.pub_detection_in_robot_frame = rospy.Publisher(pub_topic_name ,VehiclePoseEuler,queue_size=1)

    def cbDetection(self,msg):
        if(len(msg.detections)>0): # non-emtpy detection message
            # unpack the position and orientation returned by apriltags2 ros
            t_msg = msg.detections[0].pose.pose.pose.position
            q_msg = msg.detections[0].pose.pose.pose.orientation

            # convert the message content into a numpy array as robot_pose_in_world_frame requires so.
            t = np.array([t_msg.x, t_msg.y, t_msg.z])
            q = np.array([q_msg.x, q_msg.y, q_msg.z, q_msg.w])

            # express relative rotation of the robot wrt the global frame.
            veh_R_world, veh_t_world = robot_pose_in_word_frame(q,t)
            veh_feaXYZ_world = rotation_matrix_to_euler(veh_R_world)

            # convert from numpy float to standart python float to be written into the message
            veh_t_world = veh_t_world.tolist()
            veh_feaXYZ_world = veh_feaXYZ_world.tolist()

            # form message to publish
            veh_pose_euler_msg = VehiclePoseEuler()
            veh_pose_euler_msg.header.stamp = rospy.Time.now()
            # position
            veh_pose_euler_msg.posx = veh_t_world[0]
            veh_pose_euler_msg.posy = veh_t_world[1]
            veh_pose_euler_msg.posz = veh_t_world[2]
            # orientation
            veh_pose_euler_msg.rotx = veh_feaXYZ_world[0]
            veh_pose_euler_msg.roty = veh_feaXYZ_world[1]
            veh_pose_euler_msg.rotz = veh_feaXYZ_world[2]
            # finally publish the message
            self.pub_detection_in_robot_frame.publish(veh_pose_euler_msg)

if __name__ == '__main__':
    rospy.loginfo("[TO-LOCAL-POSE NODE] initializing")
    rospy.init_node('publish_detections_in_local_frame_node', anonymous=False)

    to_local_pose = ToLocalPose()

    rospy.spin()
