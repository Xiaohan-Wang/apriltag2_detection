#!/bin/bash

# setup ros environment
source "/node-ws/devel/setup.bash"
roslaunch joy_cli joy_cli.launch veh:=$DUCKIEBOT_NAME
