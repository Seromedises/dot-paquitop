#!/usr/bin/env python
import rospy
import time
import numpy
import actionlib
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(goal):

  client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
  client.wait_for_server()

  goal.target_pose.header.frame_id = "map"
  goal.target_pose.header.stamp = rospy.Time.now()
  
  client.send_goal(goal)
  wait = client.wait_for_result()
  if not wait:
      rospy.logerr("Action server not available!")
      rospy.signal_shutdown("Action server not available!")
  else:
      return client.get_result()

if __name__ == '__main__':
	f = open("/home/paquitop/catkin_ws/src/navstack_pub/trajectory_point/point.txt",'r')

	values = numpy.zeros(7)
	while True: 		
		count = 0
		while count < 7:
			line = f.readline()
			if line:
				print(line)
				values[count] = float(line.strip())
				# [float(x.strip('-')) for x in range(7) ]
				count = count+1
			
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
			result = movebase_client(goal)
			if result:
				rospy.loginfo("Goal execution done!")
		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")
