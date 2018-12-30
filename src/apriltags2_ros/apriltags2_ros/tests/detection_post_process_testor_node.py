#!/usr/bin/env python
import rospy
import unittest
import rostest
from std_msgs.msg import String
from ruamel import yaml
import numpy as np

class DetectionPostProcessTestorNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('detection_post_process_testor_node', anonymous=False)
        self.relative_pose_analysis = False
        self.allowed_virt_memory = rospy.get_param("detection_post_process_testor_node/allowed_virt_memory")
        self.allowed_real_memory = rospy.get_param("detection_post_process_testor_node/allowed_real_memory")
        self.allowed_cpu = rospy.get_param("detection_post_process_testor_node/allowed_cpu")
        self.allowed_p_range = rospy.get_param("detection_post_process_testor_node/allowed_p_range")
        self.allowed_p_var = rospy.get_param("detection_post_process_testor_node/allowed_p_var")
        self.allowed_r_range = rospy.get_param("detection_post_process_testor_node/allowed_r_range")
        self.allowed_r_var = rospy.get_param("detection_post_process_testor_node/allowed_r_var")

        # Setup the subscriber
        self.pose_analysis = rospy.Subscriber( "relative_pose_estimation_analysis", String, self.tagCallback)

        # Wait for the node to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while self.pose_analysis.get_num_connections() < 1 and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def tagCallback(self, msg):
        self.relative_pose_analysis = True
        self.summary_file_path = msg.data

    def test_publisher_and_subscriber(self):
        '''
        test whether the name of the topic is correct
        '''
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pose_analysis.get_num_connections(), 1, "No connections found on pose_analysis topic")

        
    def test_relative_pose_estimation(self):
        '''
        test cpu, memory usage and range and standard deviance of pose estimation,
        require them to be smaller than a certain value
        '''
        self.setup()    # Setup the node
        
        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(15) # Wait at most 15 seconds for the node to reply
        while not self.relative_pose_analysis and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLess(rospy.Time.now(), timeout, "Waiting for pose estimation analysis timed out.")

        with open(self.summary_file_path, 'r') as f:
            summary = yaml.load(f.read())
            #avarage value of cpu/ram usage
            virt_memory = summary['cpu/ram']['virt_mem (MB)']['mean']
            real_memory = summary['cpu/ram']['real_mem (MB)']['mean']
            cpu_load = summary['cpu/ram']['cpu_load (%)']['mean']
            #range of positon estimation
            p_range = [summary['apriltag_output']['x (cm)']['range'],
                       summary['apriltag_output']['y (cm)']['range'],
                       summary['apriltag_output']['z (cm)']['range']]
            #variance of positon estimation
            p_var = [summary['apriltag_output']['x (cm)']['variance'],
                       summary['apriltag_output']['y (cm)']['variance'],
                       summary['apriltag_output']['z (cm)']['variance']]
            #range of orientation estimation
            r_range = [summary['apriltag_output']['ox (degree)']['range'],
                       summary['apriltag_output']['oy (degree)']['range'],
                       summary['apriltag_output']['oz (degree)']['range']]
            #variance of orientation estimation
            r_var = [summary['apriltag_output']['ox (degree)']['variance'],
                       summary['apriltag_output']['oy (degree)']['variance'],
                       summary['apriltag_output']['oz (degree)']['variance']]

        self.assertLessEqual(virt_memory, self.allowed_virt_memory) 
        self.assertLessEqual(real_memory, self.allowed_real_memory)
        self.assertLessEqual(cpu_load, self.allowed_cpu)

        self.assertLessEqual(p_range, [self.allowed_p_range]*3)
        self.assertLessEqual(p_var, [self.allowed_p_var]*3)
        self.assertLessEqual(r_range, [self.allowed_r_range]*3)
        self.assertLessEqual(r_var, [self.allowed_r_var]*3)
                
if __name__ == '__main__':
    rostest.rosrun('apriltags2_ros', 'detection_post_process_testor_node', DetectionPostProcessTestorNode)
