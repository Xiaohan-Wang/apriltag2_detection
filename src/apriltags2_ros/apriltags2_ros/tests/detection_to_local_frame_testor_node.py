#!/usr/bin/env python
import rospy
import unittest
import rostest
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import cv2
from tf import transformations as tr
import numpy as np
from apriltags2_ros.msg import VehiclePoseEuler
import os


class DetectionToLocalFrameTestorNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('detection_to_local_frame_testor_node', anonymous=False)
        self.msg_received = 0
        self.vehicle_pose_euler = []

        #allowed_mismatch for postion : am_p
        #allowed_mismatch for rotation : am_r
        self.am_p = rospy.get_param("detection_to_local_frame_testor_node/am_p")
        self.am_r = rospy.get_param("detection_to_local_frame_testor_node/am_r")
        # Setup the publisher and subscriber
        self.pub_rect  = rospy.Publisher("/image_rect", Image, queue_size=1, latch=True)
        self.pub_info  = rospy.Publisher("/rect_camera_info", CameraInfo, queue_size=1, latch=True)
        self.sub_tag = rospy.Subscriber( "/detection_to_local_frame", VehiclePoseEuler, self.tagCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while (self.pub_rect.get_num_connections() < 1 or self.pub_info.get_num_connections() < 1 or
                self.sub_tag.get_num_connections() < 1) and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def tagCallback(self, msg):
        self.vehicle_pose_euler.append(msg)
        self.msg_received += 1

    def test_publisher_and_subscriber(self):
        '''
        test whether those topic are published or subscribed by corresponding nodes
        i.e. whether the name of those topics are correct
        '''
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pub_rect.get_num_connections(), 1, "No connections found on image_rect topic")
        self.assertGreaterEqual(self.pub_info.get_num_connections(), 1, "No connections found on rect_camera_info topic")
        self.assertGreaterEqual(self.sub_tag.get_num_connections(), 1, "No connections found on tag_detections topic")
    
    def test_with_known_image(self):
        '''
        test whether 'publish_detections_in_local_frame' node publishes correct relative pose
        test folder is set to /tests/test_images by default, but it can also be appointed when starting this test node
        all images in the test folder are used 
        '''
        path = rospy.get_param("detection_to_local_frame_testor_node/path")
        self.setup()    # Setup the node
        
        total_test_num = 0   # total number of published test images
        for filename in os.listdir(path):
            ab_path = path + '/' + filename
            if(not os.path.isfile(ab_path)):
                continue
            groundtruth = float(filename.split('.')[0])  # name of test image should be set to its groundtruth
            rospy.loginfo("we are testing image %d : %d degree", total_test_num, groundtruth)

            # Publish the camera info
            msg_info = CameraInfo()
            msg_info.height = 792
            msg_info.width = 1056
            msg_info.K = [329.8729619143081, 0.0, 528.0, 0.0, 332.94611303946357, 396.0, 0.0, 0.0, 1.0]
            self.pub_info.publish(msg_info)
            rospy.loginfo("Publish the camera info")

            # Publish the test image
            img = cv2.imread(ab_path)
            cvb = CvBridge()
            msg_rect = cvb.cv2_to_imgmsg(img, encoding="bgr8")
            self.pub_rect.publish(msg_rect)
            rospy.loginfo("Publish the test image")
            total_test_num += 1

            # Wait for the message to be received
            timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
            while self.msg_received < total_test_num and not rospy.is_shutdown() and rospy.Time.now() < timeout:
                rospy.sleep(0.1)
            self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")
            
            rospy.loginfo("posx: %f, posy: %f, posz: %f, rotx: %f, roty: %f, rotz: %f",
                self.vehicle_pose_euler[-1].posx * 100, self.vehicle_pose_euler[-1].posy * 100,
                self.vehicle_pose_euler[-1].posz * 100, self.vehicle_pose_euler[-1].rotx,
                self.vehicle_pose_euler[-1].roty, self.vehicle_pose_euler[-1].rotz)

            # The second parameter is groundtruth, and the third one is allowed mismatch
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].posx * 100, 30, delta = self.am_p) 
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].posy * 100, 0, delta = self.am_p)
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].posz * 100, 5, delta = self.am_p)
            
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].rotx, 0, delta = self.am_r)
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].roty, 0, delta = self.am_r)
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].rotz, groundtruth, delta = self.am_r)

'''
[RESOURCE]

[1] template: https://github.com/duckietown/Software/blob/master18/catkin_ws/src/20-indefinite-navigation/apriltags_ros/apriltags_ros/tests/apriltags_tester_node.py

[2] rectified camera info: https://github.com/selcukercan/Software/blob/master18/catkin_ws/src/05-teleop/pi_camera/src/image_rect_full_ratio.py

[3] value of groundtruth: https://github.com/selcukercan/apriltag2-addon/blob/master/src/apriltags2_ros/apriltags2_ros/include/apriltags2_ros_post_process/rotation_utils.py
'''
                
if __name__ == '__main__':
    rostest.rosrun('apriltags2_ros', 'detection_to_local_frame_testor_node', DetectionToLocalFrameTestorNode)
