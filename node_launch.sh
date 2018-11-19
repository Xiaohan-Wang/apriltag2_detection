#!/bin/bash

# setup ros environment

#set -e -x
#source /home/software/docker/env.sh

echo "Launching apriltag2_demo.launch"
roslaunch apriltags2_ros apriltag2_demo.launch veh:=$HOSTNAME
