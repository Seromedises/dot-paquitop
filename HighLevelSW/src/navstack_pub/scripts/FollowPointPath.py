#!/usr/bin/env python
import rospy
import time
import numpy
import actionlib
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
a=b=c= 0.0

def subs(msg):
    
	a = msg.poses[0].pose.position.x
	b = msg.poses[0].pose.position.y
	a = float(a)
	b = float(b)
	vet = [a,b]
	feedback = list(vet)
	
def movebase_client(goal):

	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	client.send_goal(goal)
	client.wait_for_result()
	return client.get_result()

if __name__ == '__main__':
	f = open("/home/paquitop/catkin_ws/src/navstack_pub/trajectory_point/point.txt",'r')
	
	global feedback
	feedback = numpy.zeros(2)
	path_incomplete = True
	values = numpy.zeros(7)

	while path_incomplete and not rospy.is_shutdown(): 		
		count = 0
		rospy.Subscriber('/trajectory', Path, subs)

		if abs(values[0]-feedback[0])<0.30 and abs( values[1]-feedback[1])<0.30:
			rospy.loginfo("Goal reached")

			while count < 7:
				line = f.readline()
				if line:
					print(line)
					values[count] = float(line.strip())
					# [float(x.strip('-')) for x in range(7) ]
					count = count+1
				else: 
					path_incomplete = False
					rospy.loginfo("Path end")			
	
			goal = MoveBaseGoal()	
			goal.target_pose.pose.position.x = values[0]
			goal.target_pose.pose.position.y = values[1]
			goal.target_pose.pose.position.z = values[2]
			 
			goal.target_pose.pose.orientation.x = values[3]
			goal.target_pose.pose.orientation.y = values[4]
			goal.target_pose.pose.orientation.z =	values[5]
			goal.target_pose.pose.orientation.w = values[6]
			line = f.readline()
  
		try:
				rospy.init_node('movebase_client_py')
				rospy.loginfo("pose send")
				movebase_client(goal)
				
		except rospy.ROSInterruptException:
				rospy.loginfo("Navigation test finished.")
  
