#!/usr/bin/env python3

import cv2
import time
import numpy as np
from math import pi
import rospy
import rospkg
from kortex_driver.srv import *
from kortex_driver.msg import *
from std_srvs.srv import Empty
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
from ExampleFullArmMovement import *
from gui_interface.msg import face_detection

class faceFollowing():

    def __init__(self, controlFlag):

        rospy.init_node("gui_orient")
        rospy.Subscriber("/orient_gui", Bool, self.activateRutine)
        rospy.Subscriber("/faces", face_detection, self.loadFaces)

        # Initialize comunicaiton with kinova:
        self.example = ExampleFullArmMovement() 
        notif = self.example.example_subscribe_to_a_robot_notification()
        self.example.example_clear_faults()
        self.numfaces = 2
        self.face = np.zeros(4)

    def main(self):

        # Define the desired position for the detected face inside the frame:
        x_g = 640/2
        y_g = 480/4
        x_tol = 640/10
        y_tol = 480/10

        # Define variables for kinova movements:
        scanning_counter = 0
        scanning_sign = 1
        scanning_vel = 0.1
        orient_vel = 0.2

        while (not rospy.is_shutdown()):

            if self.controlFlag:

                if self.numfaces == 0:
                    # No self.faces detected: scan for sameone
                    if scanning_counter <=5:
                        if scanning_sign == 1:
                            jd0 = scanning_vel
                        elif scanning_sign == -1:
                            jd0 = -scanning_vel
                        joint_vel = [jd0, 0.0, 0.0, 0.0, 0.0, 0.0] # rad/s
                        self.example.publish_joint_velocity(joint_vel)
                        scanning_counter += 1
                    elif scanning_counter <= 20:
                        jd0 = -jd0
                        joint_vel = [jd0, 0.0, 0.0, 0.0, 0.0, 0.0] # rad/s
                        self.example.publish_joint_velocity(joint_vel)
                        scanning_counter += 1
                    else:
                        scanning_counter = 0

                elif self.numfaces == 1:
                    # One face detected: follow it
                    scanning_counter = 0
                    x = self.face[0]
                    y = self.face[1]
                    w = self.face[2]
                    h = self.face[3]
                    
                    if abs(int(x+w/2)-x_g) > x_tol:
                        if int(x+w/2) < x_g:
                            joint_vel = [-orient_vel, 0.0, 0.0, 0.0, 0.0, 0.0] # rad/s
                            self.example.publish_joint_velocity(joint_vel)
                            scanning_sign = -1
                        else:
                            joint_vel = [orient_vel, 0.0, 0.0, 0.0, 0.0, 0.0] # rad/s
                            self.example.publish_joint_velocity(joint_vel)
                            scanning_sign = +1
                    else: 
                        joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # rad/s
                        self.example.publish_joint_velocity(joint_vel)

                    if abs(int(y+h/2)-y_g) > y_tol:
                        if int(y+h/2) < y_g:
                            msg = TwistCommand()
                            msg.twist.linear_x =  0.0
                            msg.twist.linear_y = 0.0
                            msg.twist.linear_z =  0.0
                            msg.twist.angular_x =  orient_vel/2
                            msg.twist.angular_y =  0.0
                            msg.twist.angular_z =  0.0
                            msg.duration = 1
                            publisher = rospy.Publisher('/my_gen3_lite/in/cartesian_velocity',TwistCommand, queue_size=1)
                            publisher.publish(msg)

                        else:
                            msg = TwistCommand()
                            msg.twist.linear_x =  0.0
                            msg.twist.linear_y = 0.0
                            msg.twist.linear_z =  0.0
                            msg.twist.angular_x =  -orient_vel/2
                            msg.twist.angular_y =  0.0
                            msg.twist.angular_z =  0.0
                            msg.duration = 1
                            publisher = rospy.Publisher('/my_gen3_lite/in/cartesian_velocity',TwistCommand, queue_size=1)
                            publisher.publish(msg)
        
                else: 
                    # Multiple faces detected: wait
                    scanning_counter = 0
                    joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # rad/s
                    self.example.publish_joint_velocity(joint_vel)
                
            else:
                scanning_counter = 0
                joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # rad/s
                self.example.publish_joint_velocity(joint_vel)
                time.sleep(5)
                break
            
            time.sleep(0.4)    

    def activateRutine(self, data):  
        self.controlFlag = data.data
        self.main()

    def loadFaces(self, data):  
        self.numfaces = data.num_faces
        self.face = data.face

if __name__ == '__main__':

    faceFollowing(False)