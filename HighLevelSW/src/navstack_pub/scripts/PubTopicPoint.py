#!/usr/bin/env python
import rospy
import time
import numpy
import actionlib
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import rospkg


if __name__ == '__main__':
	rospack = rospkg.RosPack()
	folder = rospack.get_path('navstack_pub')
	folder = folder + "/trajectory_point/point.txt"

	f = open(folder,'r')
	
	rospy.init_node('waypoints_publisher')
	publisher = rospy.Publisher("/addpose", PoseWithCovarianceStamped, queue_size=20)
	rate = rospy.Rate(0.5)
	
	pose = PoseWithCovarianceStamped()
	
	pose.header.frame_id = 'map'
	pose.pose.covariance = numpy.zeros(36)

	values = numpy.zeros(7)
	end = False
	while not rospy.is_shutdown() and not end: 		
		count = 0
		while count < 7 and not end:
			line = f.readline()
			if line:
				# print(line)
				values[count] = float(line.strip())
				# [float(x.strip('-')) for x in range(7) ]
				count = count+1
			else:
				end = True
		
		line = f.readline()

		pose.pose.pose.position.x = values[0]
		pose.pose.pose.position.y = values[1]
		pose.pose.pose.position.z = values[2]

		pose.pose.pose.orientation.x = values[3]
		pose.pose.pose.orientation.y = values[4]
		pose.pose.pose.orientation.z = values[5]
		pose.pose.pose.orientation.w = values[6]  
		rate.sleep()
		publisher.publish(pose)

		
