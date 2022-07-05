#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

from cgitb import reset
import imp
from itertools import count
import re
import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from std_msgs.msg import Empty, Bool
from math import pi
import math
import numpy as np
from kortex_driver.srv import *
from kortex_driver.msg import *


class ExampleFullArmMovement:
    def __init__(self):
        try:
            rospy.init_node('robot_mover_paquitop')
            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 6)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", True)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
            rospy.wait_for_service(play_cartesian_trajectory_full_name)
            self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name, PlayCartesianTrajectory)

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)


        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def protection_zone_mod(self,radius):
        
        protection_zone_reader = '/' + self. robot_name + '/base/read_all_protection_zones'
        rospy.wait_for_service(protection_zone_reader)
        self.protection_zone_reader = rospy.ServiceProxy(protection_zone_reader, ReadAllProtectionZones)
        protection_zone_read = ReadAllProtectionZonesRequest()
        
        zone = self.protection_zone_reader(protection_zone_read)
        
        protection_zone_service = '/' + self.robot_name + '/base/update_protection_zone'
        rospy.wait_for_service(protection_zone_service)
        self.protection_zone_service = rospy.ServiceProxy(protection_zone_service, UpdateProtectionZone)
        
        send = UpdateProtectionZoneRequest(zone.output.protection_zones[0])     
        
        send.input.shape.dimensions = [0.35, radius]
        send.input.is_enabled = True

        self.protection_zone_service(send)        

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
    
    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def example_send_cartesian_pose(self, pose):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        # feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = PlayCartesianTrajectoryRequest()
        req = pose
        pose_speed = CartesianSpeed()
        pose_speed.translation = 0.1
        pose_speed.orientation = 15

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object : 
        req.input.constraint.oneof_type.speed.append(pose_speed)
        

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_joint_angles(self, joint):
        self.last_action_notif_type = None
        # Create the list of angles
        req = PlayJointTrajectoryRequest()
        # Here the arm is vertical (all zeros)
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle() 
            temp_angle.joint_identifier = i
            temp_angle.value = joint[i]
            req.input.joint_angles.joint_angles.append(temp_angle)
        
        # Send the angles
        rospy.loginfo("Sending the joint command...")
        try:
            self.play_joint_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def gripper_fb(self):
        
        fb = rospy.wait_for_message('/my_gen3_lite/base_feedback', BaseCyclic_Feedback)
        
        gripper_pos = fb.interconnect.oneof_tool_feedback.gripper_feedback.position
        gripper_pos = gripper_pos/100 #divided by 100 because gripper is commanded between 0 and 1
        return gripper_pos

    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 1
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")
        difference = 1
        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(2.0)
            return True

    def example_cartesian_waypoint_action(self):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
        
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, 90.6, -1.0, 150, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, 90.6, -1.0, 150, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, 90.6, -1.0, 150, 0))
        else:
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  90, 0, 90, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.0,   0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.61, 0.22,  0.4,  90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.63, -0.22, 0.45, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.65, 0.05,  0.45, 90, 0, 90, 0))
        
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        # Call the service
        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def stop(self,stop):
        global last
        if last != stop.data:
            last = stop.data
            if last:
                stop_pub = rospy.Publisher('/my_gen3_lite/in/emergency_stop',Empty, queue_size=1)
                message_stop = Empty()
                stop_pub.publish(message_stop)
            if not last: 
                self.example_subscribe_to_a_robot_notification()
                self.example_clear_faults()

def extract_tablet(data):
    global rest_position
    if data.data and rest_position:
        gripper_open = 0.0
        gripper_close = 1.0
        reach = False
        joint = np.zeros(6)

        # open gripper
        reach = example.example_send_gripper_command(gripper_open)
        rospy.loginfo('gripper opened')
        

        # reconfigure position
        joint[0] = 65.459
        joint[1] = 38.516
        joint[2] = -55.806
        joint[3] = 9.371
        joint[4] = -120
        joint[5] = 10.311
            
        reach = example.example_send_joint_angles(joint)
        if reach:
            rospy.loginfo('joint done')
        
        
        # tablet approach position
        joint[0] = 68.452
        joint[1] = 82.279
        joint[2] = -77.663
        joint[3] = 67.203
        joint[4] = -108.603
        joint[5] = 82.363
            
        reach = example.example_send_joint_angles(joint)
        if reach:
            rospy.loginfo('joint done')
        
        # tablet pick position
        joint[0] = 58.75
        joint[1] = 73.103
        joint[2] = -110.242
        joint[3] = 58.704
        joint[4] = -87.14
        joint[5] = 91.737
            
        reach = example.example_send_joint_angles(joint)
        
        # close gripper
        reach = example.example_send_gripper_command(gripper_close)
        rospy.loginfo('gripper closed')
        
        
        # lift pose
        pose = PlayCartesianTrajectoryRequest()
        pose.input.target_pose.x = -0.18
        pose.input.target_pose.y = -0.032
        pose.input.target_pose.z = 0.156
        pose.input.target_pose.theta_x = -90
        pose.input.target_pose.theta_y = 0
        pose.input.target_pose.theta_z = 0
        
        reach = False
        while not reach: 
            reach = example.example_send_cartesian_pose(pose)

        # lift pose 2
        pose = PlayCartesianTrajectoryRequest()
        pose.input.target_pose.x = -0.18 
        pose.input.target_pose.y = -0.05
        pose.input.target_pose.z = 0.25
        pose.input.target_pose.theta_x = -90
        pose.input.target_pose.theta_y = 0
        pose.input.target_pose.theta_z = 0
        reach = False
        while not reach: 
            reach = example.example_send_cartesian_pose(pose)
        
        # tablet deliver
        joint[0] = -69.73
        joint[1] = 35.37
        joint[2] = 60.48
        joint[3] = -9.63
        joint[4] = -63.40
        joint[5] = -116.43
            
        reach = example.example_send_joint_angles(joint)
        rest_position = False

    if data.data:
        tablet_extracted = rospy.Publisher("/tablet_extracted", Bool, queue_size=1)
        tablet_extracted_msg = Bool()
        tablet_extracted.data = True
        tablet_extracted.publish(tablet_extracted_msg)
        

def retrain_tablet(data):
    global rest_position
    if data.data and not rest_position:
        gripper_open = 0.0
        gripper_close = 1.0
        reach = False
        joint = np.zeros(6)
        
        # tablet lift 2
        joint[0] = 53.69
        joint[1] = 33.71
        joint[2] = -130.425
        joint[3] = 52.569
        joint[4] = -102.714
        joint[5] = 80.438
            
        reach = example.example_send_joint_angles(joint)

        # lift pose 
        pose = PlayCartesianTrajectoryRequest()
        pose.input.target_pose.x = -0.18 
        pose.input.target_pose.y = -0.032
        pose.input.target_pose.z = 0.156
        pose.input.target_pose.theta_x = -90
        pose.input.target_pose.theta_y = 0
        pose.input.target_pose.theta_z = 0
        reach = False
        while not reach: 
            reach = example.example_send_cartesian_pose(pose)

        # tablet 
        joint[0] = 58.75
        joint[1] = 73.103
        joint[2] = -110.242
        joint[3] = 58.704
        joint[4] = -87.14
        joint[5] = 91.737
            
        reach = example.example_send_joint_angles(joint)
        
        # open gripper
        reach = example.example_send_gripper_command(gripper_open)
        rospy.loginfo('gripper opened') 
        
        # tablet approach position
        joint[0] = 68.452
        joint[1] = 82.279
        joint[2] = -77.663
        joint[3] = 67.203
        joint[4] = -108.603
        joint[5] = 82.363
            
        reach = example.example_send_joint_angles(joint)

        # reconfigure position
        joint[0] = 65.459
        joint[1] = 38.516
        joint[2] = -55.806
        joint[3] = 9.371
        joint[4] = -120
        joint[5] = 10.311
            
        reach = example.example_send_joint_angles(joint)

        # rest position
        joint[0] = 0.0
        joint[1] = 105.0
        joint[2] = 148.0
        joint[3] = 90.0
        joint[4] = 45.0
        joint[5] = 90.0
            
        reach = example.example_send_joint_angles(joint)

        rest_position = True
        
    if data.data:
              
        tablet_stored = rospy.Publisher("/tablet_stored", Bool, queue_size=1)
        tablet_stored_msg = Bool()
        tablet_stored_msg.data = True
        tablet_stored.publish(tablet_stored_msg)

  

def main():
    global rest_position
    rest_position = True
    global last
    last = False
    global example
    example = ExampleFullArmMovement()

    # For testing purposes
    example.is_init_success
    
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass
    
    example.protection_zone_mod(0.08)
    example.example_clear_faults()
    example.example_subscribe_to_a_robot_notification()

    while not rospy.is_shutdown():
        # rospy.Subscriber("/stop_arm", Bool, example.stop)
        rospy.Subscriber('/extract_tablet', Bool, extract_tablet)
        rospy.Subscriber('/retrain_tablet', Bool, retrain_tablet)
        time.sleep(0.1)
    
  
if __name__ == '__main__':
  main()
