#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Empty, Bool
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

def pub_pose(data):
    global all_point_published
    global pub_pose_counter
    global in_movement

    if pub_pose_counter == 0:
        goal = "Letto 2"

    if pub_pose_counter == 1:
        goal = "Laboratorio"

    if pub_pose_counter == 2:
        goal = "Sala Prelievi"


    if data.data and in_movement:
        rospack = rospkg.RosPack()
        folder = rospack.get_path('navstack_pub')
        folder = folder + "/trajectory_point/" + goal + ".txt"

        f = open(folder,'r')

        publisher = rospy.Publisher("/addpose", PoseWithCovarianceStamped, queue_size=20)
        rate = rospy.Rate(0.5) # rospy.Rate(0.5)

        pose = PoseWithCovarianceStamped()

        pose.header.frame_id = 'map'
        pose.pose.covariance = np.zeros(36)

        values = np.zeros(7)
        end = False
        while not rospy.is_shutdown() and not end: 		
            count = 0
            while count < 7 and not end:
                line = f.readline()
                if line:
                   
                    values[count] = round(float(line.strip()),5)
                    
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

        pub_pose_counter = pub_pose_counter + 1
        all_point_published = True
        

def startPAQUITOP(data):
    global all_point_published
    
    print(data.data)
    #print(all_point_published)
    if data.data:# and all_point_published: 
        all_point_published = False
        count = 0
        while count < 3:
            cout = count +1
            Start = Empty()
            publisher = rospy.Publisher('/path_ready', Empty, queue_size=20)
            publisher.publish(Start)
         
    

if __name__ == '__main__':
    rospy.init_node('backend')
    # global variables    
    global pub_pose_counter
    pub_pose_counter = 0  

    # Subscribers functions
    while not rospy.is_shutdown():
        
        rospy.Subscriber("/extract_tablet", Bool, pub_pose)
        rospy.Subscriber("/tablet_stored", Bool, startPAQUITOP)
        time.sleep(0.5)


    
