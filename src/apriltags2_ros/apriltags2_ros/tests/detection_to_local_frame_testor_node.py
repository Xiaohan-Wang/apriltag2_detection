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


class DetectionToLocalFrameTestorNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('detection_to_local_frame_testor_node', anonymous=False)
        self.msg_received = 0
        self.msg_tags = []
        #allowed_mismatch for postion : am_p
        #allowed_mismatch for rotation : am_r
        self.am_p = rospy.get_param("/testbot/detection_to_local_frame_testor_node/am_p")
        self.am_r = rospy.get_param("/testbot/detection_to_local_frame_testor_node/am_r")

        # Setup the publisher and subscriber
        self.pub_rect  = rospy.Publisher("/image_rect", Image, queue_size=1, latch=True)
        self.pub_info  = rospy.Publisher("/rect_camera_info", CameraInfo, queue_size=1, latch=True)
        self.sub_tag = rospy.Subscriber( "/detection_to_local_frame", VehiclePoseEuler, self.tagCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while (self.pub_rect.get_num_connections() < 1 or self.pub_info.get_num_connections() < 1 or
                self.sub_tag.get_num_connections() < 1) and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def tagCallback(msg, self):
        self.msg_tags = VehiclePoseEuler.append(msg)
        self.msg_received += 1

    def test_publisher_and_subscriber(self):
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pub_rect.get_num_connections(), 1, "No connections found on image_rect topic")
        self.assertGreaterEqual(self.pub_info.get_num_connections(), 1, "No connections found on rect_camera_info topic")
        self.assertGreaterEqual(self.sub_tag.get_num_connections(), 1, "No connections found on tag_detections topic")
    
    def test_with_known_image(self):
        path = rospy.get_param("/testbot/detection_to_local_frame_testor_node/path")
        self.setup()    # Setup the node
        
        # Publish the camera info
        msg_info = CameraInfo()
        msg_info.height = 480
        msg_info.width = 640
        msg_info.K = [331.026328, 0.0, 319.035097, 0.0, 335.330339, 216.450133, 0.0, 0.0, 1.0]
        self.pub_info.publish(msg_info)
        
        for num in range(0,1):
            filename = path + '/' + str((num-3)*10) + '.png'
            # Publish the test image
            img = cv2.imread(filename)
            cvb = CvBridge()
            msg_rect = cvb.cv2_to_imgmsg(img, encoding="bgr8")
            self.pub_rect.publish(msg_rect)

            # Wait for the message to be received
            timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
            while self.msg_received < num+1 and not rospy.is_shutdown() and rospy.Time.now() < timeout:
                print(self.msg_received)
                rospy.sleep(0.1)
            self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")


            self.assertAlmostEqual(self.msg_tags[num].posx, 0.0, delta = self.am_p) 
            self.assertAlmostEqual(self.msg_tags[num].posy, 0.0, delta = self.am_p)
            self.assertAlmostEqual(self.msg_tags[num].posz, 0.3, delta = self.am_p)
            
            self.assertAlmostEqual(self.msg_tags[num].rotx, 0, delta = self.am_r)
            self.assertAlmostEqual(self.msg_tags[num].roty, (num-3)*10, delta = self.am_r)
            self.assertAlmostEqual(self.msg_tags[num].rotz, 0, delta = self.am_r)

                
if __name__ == '__main__':
    rostest.rosrun('apriltags2_ros', 'detection_to_local_frame_testor_node', DetectionToLocalFrameTestorNode)
