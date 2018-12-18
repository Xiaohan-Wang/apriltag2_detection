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

1. 
