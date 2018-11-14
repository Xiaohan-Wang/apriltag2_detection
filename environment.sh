#!/bin/bash
# New policy: DUCKIETOWN_ROOT is implicit in the choice of running this script

[ -z "$HOSTNAME"        ] && { echo -e "\n\nThe variable HOSTNAME is not set. I need this info for setting up ROS. \n\n\n\n"; return 2;       }

# Do not compile Lisp messages
# XXX: not sure if this is the place to put this.
export ROS_LANG_DISABLE=gennodejs:geneus:genlisp

shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/kinetic/setup.$shell

echo "Activating development environment at..."
echo $PWD/devel/setup.$shell
source $PWD/devel/setup.$shell

export HOSTNAME=$HOSTNAME
export ROS_HOSTNAME=$HOSTNAME.local
echo "Set ROS_HOSTNAME to: $ROS_HOSTNAME"

export PYTHONPATH=$PWD/src:$PYTHONPATH
echo "Set PYTHONPATH to: $PYTHONPATH"

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
