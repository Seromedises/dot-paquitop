#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/dot-paquitiop/HighLevelSW/devel/setup.bash

export ROS_IP=172.21.15.99
export ROS_MASTER_URI=http://172.21.15.100:11311

exec "$@"
