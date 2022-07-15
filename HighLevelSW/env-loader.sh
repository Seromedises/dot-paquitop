#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/dot-paquitop/HighLevelSW/devel/setup.bash
my_ip=$(hostname -I | cut -d " " -f 1)
if [ "$my_ip" == 172.21.15.99 ]; then
	export ROS_IP=$my_ip
	export ROS_MASTER_URI=http://172.21.15.100:11311
fi
if [ "$my_ip" == 192.168.8.229 ]; then
	export ROS_IP=$my_ip
	export ROS_MASTER_URI=http://192.168.8.219:11311	
fi
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2
export DISPLAY=":0.0"

exec "$@"
