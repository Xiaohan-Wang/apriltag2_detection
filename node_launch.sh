#!/bin/bash

# setup ros environment

set -e
source /home/software/docker/env.sh

#echo "[NOTICE6] sourced /catkin_ws/devel/setup.bash"

# roslaunch joy_cli joy_cli.launch veh:=$DUCKIEBOT_NAME
