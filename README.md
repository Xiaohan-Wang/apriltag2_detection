# apriltag2_detection
This package is used to analyze the relative pose of single duckiebot estimated by using AprilTag2. Except for the normal pipeline of AprilTag2, it also provides post-processing srcipts to get result analysis, as well as several unit-tests for testing the correctness of the system.

## result analysis
This package provide `detection_post_process.launch` to analyze the performance of relative pose estimation by using AprilTag2. Each time it collects a certain number of estimated pose (both postion and rotation) and then compute the following:
* max, min, range, mean, variance of these pose estimation.
* time used for each subprocess in the pipeline of relative pose estimation (detecting AprilTag + computing relative pose).
* cpu/ram comsumption for relative pose estimation.

1. `catkin_make`
2. `source devel/setup.bash`
3. `roslaunch pi_camera camera_apriltag_demo.launch veh:=duckiebot`
4. `roslaunch apriltags2_ros apriltag2_demo.launch veh:=duckiebot`
5. `roslaunch apriltags2_ros detection_to_local_frame.launch veh:=duckiebot`
6. `roslaunch apriltags2_ros detection_post_process.launch veh:=duckiebot`

## unit test
Several test are provided in this package for testing the correctness of the system.

### detection_to_local_frame_testor_node.test
1. This is used to test the correctness of pose estimation. All images in _tests/test_image_ are tested and the name of the images are their groundtruth in degree. If you want to specify one image, for now you should put only that image in the above folder. 
2. In the test, each time we publish one of these images as well as camera info. `apriltag_detector_node` is launched to detect apriltag and publish pose estimation (camera frame with respect to tag frame).  `detection_to_local_frame` is launched for coverting pose estimation into appropriate frame (robot wrt world), which is the ultimate estimation of pose we are using for tests.
* `rostest detection_to_local_frame_testor_node.test`
* `rostest --text detection_to_local_frame_testor_node.test` to get console output to the screen
