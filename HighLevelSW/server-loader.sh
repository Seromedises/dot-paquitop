#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/dot-paquitiop/HighLevelSW/devel/setup.bash
# export ROS_IP=172.21.15.100
# export ROS_MASTER_URI=http://172.21.15.100:11311
if [ "$my_ip" == 172.21.15.100 ]; then
	export ROS_IP=$my_ip
	export ROS_MASTER_URI=$my_ip:11311
fi
if [ "$my_ip" == 192.168.8.229 ]; then
	export ROS_IP=$my_ip
	export ROS_MASTER_URI=$my_ip:11311	
fi

exec "$@"
