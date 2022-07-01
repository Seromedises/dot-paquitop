#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/dot-paquitop/HighLevelSW/devel/setup.bash
export ROS_IP=172.21.15.99
export ROS_MASTER_URI=http://172.21.15.100:11311
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2
export DISPLAY=":0.0"

exec "$@"
