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
5. `roslaunch apriltags2_ros detection_to_local_frame.launch veh:=duckiebot`
6. `rosparam set enable_statistics true` & `rosrun rosprofiler rosprofiler`
7. `roslaunch apriltags2_ros detection_post_process.launch veh:=duckiebot`

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
    4. It is hard to guarantee that different duckiebot and Apriltag have the very same condition, so the groundtruth of above three tests must be adjusted based on your own duckiebot and Apriltag if you want to use them.
   
* `rostest detection_to_local_frame_testor_node.test`
* `rostest --text detection_to_local_frame_testor_node.test` to get console output to the screen

### detection_post_process_testor_node.test
1. This test focuses on the analysis produced by `detection_post_process.launch`.
2. In this test, you need to put an Apriltag in front of a duckiebot. When you execute this test, we automaticly start the nodes decribed in _result analysis_ part, collect a certain number of estimated pose, and then analyze them.
3. To successfully pass the test, the mean of cpu/ram comsumption and the range and variance of each dimension of pose estimation are required to be smaller than a certain value (you can pass these values as parameters when you launch this test).
4. It should be noted that the cpu/ram usage tests only make sense for rasp pi as they will run faster on local PC. Also pay attention that the upper limit for cpu/ram comsumption should be adjusted based on your own duckiebot when you use this test.

* `rostest detection_post_process_testor_node.test`
* `rostest --text detection_post_process_testor_node.test` to get console output to the screen
