#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
import time
a=b=c= 0.0

def subs(msg):
    
    a = msg.poses[0].pose.position.x
    b = msg.poses[0].pose.position.y
    c = msg.poses[0].pose.position.z
    d = msg.poses[0].pose.orientation.x
    e = msg.poses[0].pose.orientation.y
    f = msg.poses[0].pose.orientation.z
    g = msg.poses[0].pose.orientation.w
    
    #h = open("demo.txt", "w")
    h = open("/home/paquitop/catkin_ws/src/navstack_pub/trajectory_point/point.txt", "a")
    
    l= str(a).strip('-')+ "\n" +str(b).strip('-')+ "\n" +str(c).strip('-')+ "\n" +str(d).strip('-')+ "\n" +str(e).strip('-')+ "\n" +str(f).strip('-')+ "\n" +str(g).strip('-')
    l = l + "\n\n"
    # print(l + "\n")

    h.write(str(l))
    h.close()
    #print a
    #print b
    #print c
    
    time.sleep(5)

h = open("/home/paquitop/catkin_ws/src/navstack_pub/trajectory_point/point.txt", "w")    
rospy.init_node('check_pose')
odom_sub = rospy.Subscriber('/trajectory', Path, subs)

rospy.spin()
