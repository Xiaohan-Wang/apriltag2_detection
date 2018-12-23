#!/usr/bin/env python
import rospy
import unittest
import rostest
from std_msgs.msg import String
import os


class DetectionPostProcessTestorNode(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('detection_post_process_testor_node', anonymous=False)

        self.allowed_virt_memory = rospy.get_param("detection_post_process_testor_node/allowed_virt_memory")
        self.allowed_real_memory = rospy.get_param("detection_post_process_testor_node/allowed_real_memory")
        self.allowed_cpu = rospy.get_param("detection_post_process_testor_node/allowed_cpu")
        self.allowed_p_range = rospy.get_param("detection_post_process_testor_node/allowed_p_range")
        self.allowed_p_var = rospy.get_param("detection_post_process_testor_node/allowed_p_var")
        self.allowed_r_range = rospy.get_param("detection_post_process_testor_node/allowed_r_range")
        self.allowed_r_var = rospy.get_param("detection_post_process_testor_node/allowed_r_var")

        # Setup the subscriber
        self.pose_analysis = rospy.Subscriber( "relative_pose_estimation_analysis", String, self.tagCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while self.pose_analysis.get_num_connections() and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

    def tagCallback(self, msg):
        self.relative_pose_analysis = True

    def test_publisher_and_subscriber(self):
        '''
        test whether the name of the topic is correct
        '''
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pose_analysis.get_num_connections(), 1, "No connections found on pose_analysis topic")
    
    def test_with_known_image(self):
        '''
        test whether 'publish_detections_in_local_frame' node publishes correct relative pose
        test folder is set to /tests/test_images by default, but it can also be appointed when starting this test node
        all images in the test folder are used 
        '''
        path = rospy.get_param("detection_to_local_frame_testor_node/path")
        self.setup()    # Setup the node
        
        total_test_num = 0
        for filename in os.listdir(path):
            ab_path = path + '/' + filename
            if(not os.path.isfile(ab_path)):
                continue
            total_test_num += 1
            groundtruth = float(filename.split('.')[0])
            rospy.loginfo("we are testing image %d : %d degree", total_test_num, groundtruth)

            # Publish the camera info
            msg_info = CameraInfo()
            msg_info.height = 792
            msg_info.width = 1056
            msg_info.K = [329.8729619143081, 0.0, 528.0, 0.0, 332.94611303946357, 396.0, 0.0, 0.0, 1.0]
            #msg_info.K = [315.3043824949326, 0.0, 528.0, 0.0, 317.96035132826944, 396.0, 0.0, 0.0, 1.0]
            self.pub_info.publish(msg_info)
            rospy.loginfo("Publish the camera info")

            # Publish the test image
            img = cv2.imread(ab_path)
            cvb = CvBridge()
            msg_rect = cvb.cv2_to_imgmsg(img, encoding="bgr8")
            self.pub_rect.publish(msg_rect)
            rospy.loginfo("Publish the test image")

            # Wait for the message to be received
            timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
            while self.msg_received < total_test_num and not rospy.is_shutdown() and rospy.Time.now() < timeout:
                rospy.sleep(0.1)
            self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")
            
            rospy.loginfo("posx: %f, posy: %f, posz: %f, rotx: %f, roty: %f, rotz: %f",
                self.vehicle_pose_euler[-1].posx, self.vehicle_pose_euler[-1].posy,
                self.vehicle_pose_euler[-1].posz, self.vehicle_pose_euler[-1].rotx,
                self.vehicle_pose_euler[-1].roty, self.vehicle_pose_euler[-1].rotz)

            # The second parameter is groundtruth, and the third one is allowed mismatch
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].posx, 0.3, delta = self.am_p) 
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].posy, 0, delta = self.am_p)
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].posz, 0.05, delta = self.am_p)
            
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].rotx, 0, delta = self.am_r)
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].roty, 0, delta = self.am_r)
            self.assertAlmostEqual(self.vehicle_pose_euler[-1].rotz, groundtruth, delta = self.am_r)
                
if __name__ == '__main__':
    rostest.rosrun('apriltags2_ros', 'detection_post_process_testor_node', DetectionPostProcessTestorNode)
