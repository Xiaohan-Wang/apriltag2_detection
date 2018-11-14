FROM duckietown/rpi-duckiebot-base:master18

LABEL maintainer="Selcuk Ercan ercans@ethz.ch"

# REQUIRED ENVIRONMENT VARIABLES THAT HAVE TO BE PASSED WHEN RUNNING THE CONTAINER:
# ROS_MASTER_URI - the hostname and port of the roscore master, typically http://hostname:11311 - ALWAYS REQUIRED!
# DUCKIEBOT_NAME - the hostname of the Duckiebot, e.g. duckiebot

RUN [ "cross-build-start" ]

RUN /bin/bash -c "mkdir -p custom_ws/src/"

COPY /src /custom_ws/src
COPY .gitignore /custom_ws
COPY .catkin_workspace /custom_ws
COPY node_launch.sh /custom_ws
COPY ata.robot.yaml /data/config/

ENV ROS_HOSTNAME localhost

#RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_init_workspace && cd ../.."
#RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash"
#RUN /bin/bash -c "cd /custom_ws && catkin_make -C /custom_ws"
#RUN /bin/bash -c "source /custom_ws/devel/setup.bash"

#Do not change the below line! This ensures that your workspace is overlayed on top of the Duckietown stack!
#MAKE sure this line is present in the build: This workspace overlays: /home/software/catkin_ws/devel;/opt/ros/kinetic
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_init_workspace && cd ../.."
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_make -j -C /custom_ws"
RUN echo "source /custom_ws/devel/setup.bash" >> ~/.bashrc

ENV DUCKIETOWN_ROOT /home/software
ENV DUCKIEFLEET_ROOT /data/config

RUN /bin/bash -c "set -e && source /home/software/docker/env.sh"
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && cd /home/software && make build-machines"

RUN [ "cross-build-end" ]

WORKDIR /custom_ws

#CMD [ "/custom_ws/node_launch.sh" ]
