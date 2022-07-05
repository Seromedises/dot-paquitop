#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Empty, Bool
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped

def pub_pose(self, data):
    global pub_pose_counter

    if pub_pose_counter == 0:
        goal = "Letto 1"

    if pub_pose_counter == 1:
        goal = "Letto 2"

    if pub_pose_counter == 2:
        goal = "Laboratorio"


    if data.data:
        rospack = rospkg.RosPack()
        folder = rospack.get_path('navstack_pub')
        folder = folder + "/trajectory_point/" + goal + ".txt"

        f = open(folder,'r')

        # rospy.init_node('waypoints_publisher')
        publisher = rospy.Publisher("/addpose", PoseWithCovarianceStamped, queue_size=20)
        rate = rospy.Rate(0.75) # rospy.Rate(0.5)

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
                    # print(line)
                    values[count] = round(float(line.strip()),5)
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

        pub_pose_counter = pub_pose_counter + 1

if __name__ == '__main__':
    global pub_pose_counter
    pub_pose_counter = 0

    rospy.Subscriber('/extract_tablet', Bool, pub_pose)