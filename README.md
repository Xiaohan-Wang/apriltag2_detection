# apriltag2_detection
This package is used to analyze the relative pose of single duckiebot estimated by using AprilTag2. Except for the normal pipeline of AprilTag2, it also provides post-processing srcipts to get result analysis, as well as several unit-tests for testing the correctness of the system.

## result analysis
This package provides `detection_post_process.launch` to analyze the performance of relative pose estimation by using AprilTag2. Each time it collects a certain number of estimated pose (both postion and rotation) and then compute the following:
* max, min, range, mean, variance of these pose estimation.
* time used for each subprocess in the pipeline of relative pose estimation (detecting AprilTag + computing relative pose).
* cpu/ram comsumption for relative pose estimation.

1. `catkin_make`
2. `source devel/setup.bash`
3. `roslaunch pi_camera camera_apriltag_demo.launch veh:=duckiebot`
4. `roslaunch apriltags2_ros apriltag2_demo.launch veh:=duckiebot` 
	  * you can also add `decimate:= ![num]` (num >= 1) to lessen the resolution to the 1/num of the original one, so that apriltag detection could run faster. It is set to 1.0 by default.
5. `roslaunch apriltags2_ros detection_to_local_frame.launch veh:=duckiebot`
6. `rosparam set enable_statistics true` & `rosrun rosprofiler rosprofiler`
7. `roslaunch apriltags2_ros detection_post_process.launch veh:=duckiebot`
    * you can also add `number_of_images:= ![num]` (num >= 1) to control the number of images you want to take in the same place, each of these images is used for computing one relative pose, and then all estimated poses are used for analysis which is mentioned above.

## unit test
Several test are provided in this package for testing the correctness of the system.

### detection_to_local_frame_testor_node.test
1. This is used to test the correctness of pose estimation. All images in _tests/test_image_ are tested and the name of the images are their groundtruth in degree. If you want to specify one image, for now you should put only that image in the above folder. 
2. In the test, each time we publish one of these images as well as camera info. `apriltag_detector_node` is launched to detect apriltag and publish pose estimation (camera frame with respect to tag frame).  `detection_to_local_frame` is launched for coverting pose estimation into appropriate frame (robot wrt world), which is the ultimate estimation of pose we are using for tests.
3. If you are using images taken with your own duckiebot and AprilTag, don't forget to adjust the groundtruth of the tests. Especially pay attention to the following:
    1. the results is in X camera frame (seeing what camera sees):
        * X -> forward
        * Y -> left
        * Z -> up
    2. the groundtruth for z is related to the relative height difference between camera and AprilTag. 
    3. the groundtruth for rotx and roty essentially depends on whether the Apriltag is strictly perpendicular in both x and y direction in world frame.           
    4. It is hard to guarantee that different duckiebot and Apriltag have the very same condition, so the groundtruth of above three tests must be adjusted based on your own duckiebot and Apriltag if you want to use your own images.
   
* `rostest detection_to_local_frame_testor_node.test`
* `rostest --text detection_to_local_frame_testor_node.test` to get console output to the screen
* you can also add `am_p:=![num1]` and `am_r:=![num2]` to set the allow mismatch for position and orientation. am_p = 2 and am_r = 5 by default.  

### detection_post_process_testor_node.test
1. This test focuses on the analysis produced by `detection_post_process.launch`.
2. In this test, you need to put an Apriltag in front of a duckiebot. When you execute this test, we automaticly start the nodes decribed in _result analysis_ part, collect a certain number of estimated pose, and then analyze them.
3. To successfully pass the test, the mean of cpu/ram comsumption and the range and variance of each dimension of pose estimation are required to be smaller than a certain value (you can pass these values as parameters when you launch this test).
4. It should be noted that the cpu/ram usage tests only make sense for rasp pi as they will run faster on local PC. Also pay attention that the upper limit for cpu/ram comsumption should be adjusted based on your own duckiebot when you use this test.

* `rostest detection_post_process_testor_node.test veh:=duckiebot`
* `rostest --text detection_post_process_testor_node.test veh:=duckiebot` to get console output to the screen
* optional: 
    * `number_of_sample_images:=![num]` : same as the description above
    * `decimate:= ![num]` : same as the description above
    * `allowed_cpu:=![num1]` : allowed cpu/ram usage on one core (it could be larger than 100%)
    * `allowed_real_memory:=![num2]` : allowed real memory usage
    * `allowed_virt_memory:=![num3]` : allowed virtual memory usage
    * `allowed_p_range:=![num4]` : range of position estimation
    * `allowed_p_var:=![num5]` : variance of position estimation
    * `allowed_r_range:=![num6]` : range of orientation estimation
    * `allowed_r_var:=![num7]` : variance of orientation estimation
