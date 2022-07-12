#!/usr/bin/env python3

import cv2
import time
import numpy as np
from math import pi
import rospy
import rospkg
import pyrealsense2 as rs
from kortex_driver.srv import *
from kortex_driver.msg import *
from std_srvs.srv import Empty
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import Twist
from ExampleFullArmMovement import *

class faceFollowing():

    def __init__(self, controlFlag):

        # Initialize comunicaiton with L515:
        self.controlFlag = controlFlag
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.pipelineActive = True
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        print("Communication with L515 initialized")

        # Initialize comunicaiton with kinova:
        self.example = ExampleFullArmMovement() 
        notif = self.example.example_subscribe_to_a_robot_notification()
        self.example.example_clear_faults()
        print("Communication with KINOVA initialized")

        # Create the haar cascade
        self.rospack = rospkg.RosPack()
        self.directoryPath = self.rospack.get_path('gui_interface')
        self.subdirectory = "scripts"
        self.cascName = "haarcascade_frontalface_default.xml"
        self.faceCascade = cv2.CascadeClassifier(self.directoryPath+"/"+self.subdirectory+"/"+self.cascName)
        test = self.faceCascade.load('haarcascade_frontalface_default.xml')
        print("Haar cascade created")

        print("Init completed")

    def main(self):
        
        if not self.pipelineActive:
            self.profile = self.pipeline.start(self.config)
            self.pipelineActive = True

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        frame = color_image
        frame_h, frame_w, frame_c = frame.shape

        # Define the desired position for the detected face inside the frame:
        x_g = frame_w/2
        y_g = frame_h/2
        x_tol = frame_w/10
        y_tol = frame_h/10

        # Define variables for kinova movements:
        scanning_counter = 0
        scanning_sign = 1
        scanning_vel = 0.1
        orient_vel = 0.2

        while (not rospy.is_shutdown()):

            if self.controlFlag:
            
                # Get frameset of color and depth:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                color_image = np.asanyarray(color_frame.get_data())
                frame = cv2.flip(color_image,1)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = self.faceCascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=10, minSize=(40, 40), flags = cv2.CASCADE_SCALE_IMAGE)
                
                if len(faces) == 0:
                    # No faces detected: scan for sameone
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

                elif len(faces) == 1:
                    # One face detected: follow it
                    scanning_counter = 0
                    faces = faces[0]
                    x = faces[0]
                    y = faces[1]
                    w = faces[2]
                    h = faces[3]
                    
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
                if self.pipelineActive:
                    self.pipeline.stop()
                    self.pipelineActive = False
                    scanning_counter = 0
                    joint_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # rad/s
                    self.example.publish_joint_velocity(joint_vel)
                    time.sleep(1)
                break
            
            time.sleep(0.4)    

def activateRutine(data):  
    FF.controlFlag = data.data
    FF.main()

if __name__ == '__main__':

    FF = faceFollowing(False)
    rospy.init_node("gui_orient")
    rospy.Subscriber("/orient_gui", Bool, activateRutine)
    rospy.spin()