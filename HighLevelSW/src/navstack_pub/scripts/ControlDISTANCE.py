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
import tf
from tf import TransformListener
import math
import tf2_ros


rospy.init_node('test')
waypoint = PoseWithCovarianceStamped()
waypoint.pose.pose.position.x = 0.0
waypoint.pose.pose.position.y = 0.0
listener = tf.TransformListener()
"""
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
"""
# now = rospy.Time.now()
now = rospy.Time(0)

listener.waitForTransform("map", "base_footprint", now, rospy.Duration(4.0))
trans,rot = listener.lookupTransform("map", "base_footprint", now)


distance = math.sqrt(pow(waypoint.pose.pose.position.x-trans[0],2)+pow(waypoint.pose.pose.position.y-trans[1],2))

print(distance)




		
