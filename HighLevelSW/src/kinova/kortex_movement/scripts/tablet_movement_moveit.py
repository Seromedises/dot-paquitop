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

import imp
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

class Quaternion():
	def __init__(self,w,x,y,z):
		self.w = w
		self.x = x
		self.y = y
		self.z = z

def ToQuaternion(yaw,pitch,roll):
 #yaw (Z), pitch (Y), roll (X)

	# Abbreviations for the various angular functions
	cy = math.cos(yaw )
	sy = math.sin(yaw )
	cp = math.cos(pitch )
	sp = math.sin(pitch )
	cr = math.cos(roll )
	sr = math.sin(roll )
	
	w = .5*math.sqrt(cr * cp  + cp* cy + cr*cy + sr * sp * sy +1)
	x = .5*np.sign(cp*sr+cy*sr-cr*sp*sy)*math.sqrt(-cr * cp  + cp* cy - cr*cy - sr * sp * sy +1)
	y = .5*np.sign(sp + sr+sy + cr*cy*sp)*math.sqrt(-cr * cp  - cp* cy + cr*cy + sr * sp * sy +1)
	z = .5*np.sign(cp*sy+cr*sy-cy*sp*sr)*math.sqrt(+cr * cp  - cp* cy - cr*cy - sr * sp * sy +1)
	q = Quaternion(w,x,y,z)	
	return q

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('tablet_mover')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 6)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.arm_group.set_max_velocity_scaling_factor(1)
      self.arm_group.set_max_acceleration_scaling_factor(1)
      self.arm_group.set_num_planning_attempts(50)
      self.arm_group.set_planning_time(20)
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, joint, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    # rospy.loginfo("Printing current joint positions before movement :")
    # for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = joint[0]
      joint_positions[1] = joint[1]
      joint_positions[2] = joint[2]
      joint_positions[3] = joint[3]
      joint_positions[4] = joint[4]
      joint_positions[5] = joint[5]
      joint_positions[6] = joint[6]
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = joint[0]
      joint_positions[1] = joint[1]
      joint_positions[2] = joint[2]
      joint_positions[3] = joint[3]
      joint_positions[4] = joint[4]
      joint_positions[5] = joint[5]
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    # new_joint_positions = arm_group.get_current_joint_values()
    # rospy.loginfo("Printing current joint positions after movement :")
    # for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    #rospy.loginfo("Actual cartesian pose is : ")
    #rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

def extract_tablet(data):
  if data:
    tolerance = 0.01
    gripper_open = 1
    gripper_close = 0
    reach = False
    joint = np.zeros(6)

    # open gripper
    reach = example.reach_gripper_position(gripper_open)
    rospy.loginfo('gripper opened')
    reach = False

    # reconfigure position
    joint[0] = 65.459
    joint[1] = 38.516
    joint[2] = -55.806
    joint[3] = 9.371
    joint[4] = -120
    joint[5] = 10.311
    joint = joint*pi/180
    while not reach:
      reach = example.reach_joint_angles(joint, tolerance)
    rospy.loginfo('reconfigure position')
    reach = False

    # tablet approach position
    q = ToQuaternion(0,0, -90*pi/180)
    pose = example.get_cartesian_pose()
    pose.orientation.w = q.w
    pose.orientation.x = q.x
    pose.orientation.y = q.y
    pose.orientation.z = q.z
    pose.position.x = -0.18
    pose.position.y = -0.15
    pose.position.z = 0.106
    while not reach:
      reach = example.reach_cartesian_pose(pose, tolerance, constraints=None)
    rospy.loginfo('tablet approach')
    reach = False

    # tablet pick position
    q = ToQuaternion(0,0, -90*pi/180)
    pose = example.get_cartesian_pose()
    pose.orientation.w = q.w
    pose.orientation.x = q.x
    pose.orientation.y = q.y
    pose.orientation.z = q.z
    pose.position.x = -0.18
    pose.position.y = -0.0
    pose.position.z = 0.106
    while not reach:
      reach = example.reach_cartesian_pose(pose, tolerance, constraints=None)
    rospy.loginfo('tablet pick')
    reach = False

    # close gripper
    reach = example.reach_gripper_position(gripper_close)
    rospy.loginfo('gripper closed')
    reach = False

    # lift pose
    q = ToQuaternion(0,0, -90*pi/180)
    pose = example.get_cartesian_pose()
    pose.orientation.w = q.w
    pose.orientation.x = q.x
    pose.orientation.y = q.y
    pose.orientation.z = q.z
    pose.position.x = -0.18
    pose.position.y = -0.032
    pose.position.z = 0.156
    while not reach:
      reach = example.reach_cartesian_pose(pose, tolerance, constraints=None)
    rospy.loginfo('lift pose')
    reach = False

    # lift pose 2
    pose.position.y = 0.05
    pose.position.z = 0.25
    while not reach:
      reach = example.reach_cartesian_pose(pose, tolerance, constraints=None)
    rospy.loginfo('lift pose 2')
    reach = False

    # tablet deliver
    joint[0] = -66.16
    joint[1] = 30.5
    joint[2] = 49.09
    joint[3] = -22.72
    joint[4] = -82.59
    joint[5] = -107.11
    joint = joint*pi/180
    while not reach:
      reach = example.reach_joint_angles(joint, tolerance)
    rospy.loginfo('tablet deliver')
    reach = False

def retrain_tablet(data):
  if data: 
    print (data) 

def main():
  global example
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  
  
  while not rospy.is_shutdown():
    # wait = rospy.wait_for_message('extract_tablet')
    extract = rospy.Subscriber('/extract_tablet', Bool, extract_tablet )
    # wait = rospy.wait_for_message('retain_tablet')
    retain = rospy.Subscriber('/retain_tablet', Bool, retrain_tablet)
    time.sleep(2)

if __name__ == '__main__':
  main()