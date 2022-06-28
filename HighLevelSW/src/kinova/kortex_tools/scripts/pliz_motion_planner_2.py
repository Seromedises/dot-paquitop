#!/usr/bin/env python

import sys
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import actionlib
from math import pi
from std_srvs.srv import Empty
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import *

__ROBOT_VELOCITY__ = 0.8

class Quaternion():
	def __init__(self, w, x, y, z):
		self.w = w
		self.x = x
		self.y = y
		self.z = z

def ToQuaternion(yaw, pitch, roll):
#yaw (Z), pitch (Y), roll (X)

	# Abbreviations for the various angular functions
	cy = math.cos(yaw * 0.5)
	sy = math.sin(yaw * 0.5)
	cp = math.cos(pitch * 0.5)
	sp = math.sin(pitch * 0.5)
	cr = math.cos(roll * 0.5)
	sr = math.sin(roll * 0.5)

	w = cr * cp * cy + sr * sp * sy
	x = sr * cp * cy - cr * sp * sy
	y = cr * sp * cy + sr * cp * sy
	z = cr * cp * sy - sr * sp * cy
	
	q = Quaternion(w,x,y,z)	
	
	return q

class MoveGroupPythonIntefaceTutorial(object):

	def __init__(self):

		# Initialize the node
		super(MoveGroupPythonIntefaceTutorial, self).__init__()
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('pilz_motion_planner', anonymous=True)

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
			self.arm_group.set_max_velocity_scaling_factor(__ROBOT_VELOCITY__)
			self.arm_group.set_max_acceleration_scaling_factor(0.4)
			self.arm_group.set_num_planning_attempts(20)
			self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
								                    moveit_msgs.msg.DisplayTrajectory,
								                    queue_size=50)
			self.msi.req = self.arm_goup.construct_motion_plan_request()
				

			if self.is_gripper_present:
				gripper_group_name = "gripper"
				self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

			rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
		except Exception as e:
			print (e)
			self.is_init_success = False
		else:
			self.is_init_success = True
			
	def get_cartesian_pose(self):
		arm_group = self.arm_group

		# Get the current pose and display it
		pose = arm_group.get_current_pose()
		rospy.loginfo("Actual cartesian pose is : ")
		rospy.loginfo(pose.pose)

		return pose.pose
		
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
			
def main():
	r = MoveGroupPythonIntefaceTutorial()
	
	
	# cartesian_goal=Pose(position=Point(-0.04, -0.53, 0.13), orientation=from_euler(0, math.radians(180), math.radians(0)))
	
	"""
	pose = r.get_cartesian_pose()
	pose.position.x = 0.3
	pose.position.y = -0.25
	pose.position.z = 0.45

	yaw = 90*pi/180
	pitch = 0*pi/180
	roll = 90*pi/180
	
	q = ToQuaternion(yaw,  pitch,  roll) #yaw (Z), pitch (Y), roll (X)
	
	pose.orientation.w = q.w
	pose.orientation.x = q.x 
	pose.orientation.y = q.y
	pose.orientation.z = q.z
	
	cartesian_goal = Pose(position=Point(0.3, -0.25, 0.45), orientation=from_euler(90*pi/180, 0, 90*pi/180))
	r.arm_group.set_pose_target(Ptp(goal=cartesian_goal, vel_scale=__ROBOT_VELOCITY__))
	r.arm_group.go(wait=True)
	r.arm_group.go(Ptp(goal=cartesian_goal, vel_scale=__ROBOT_VELOCITY__))
	r.arm_group.set_pose_target(Ptp(goal=cartesian_goal, vel_scale=0.9))
	"""
	sequence_move_group_srv = actionlib.SimpleActionClient("/sequence_move_group", moveit_msgs.msg.MoveGroupSequenceAction) 
	pose1 = Pose(position=Point(0.3, -0.25, 0.45), orientation=from_euler(90*pi/180, 0, 90*pi/180))
	pose2 = Pose(position=Point(0.3, -0.45, 0.45), orientation=from_euler(90*pi/180, 0, 90*pi/180))
	pose3 = Pose(position=Point(0.3, -0.45, 0.25), orientation=from_euler(90*pi/180, 0, 90*pi/180))
	pose4 = Pose(position=Point(0.3, -0.25, 0.25), orientation=from_euler(90*pi/180, 0, 90*pi/180))
	pose5 = Pose(position=Point(0.3, -0.25, 0.45), orientation=from_euler(90*pi/180, 0, 90*pi/180))
	
	
	motion_plan_requests = []
	# just in case, set the first point as the current pose
	r.arm_group.set_pose_target(r.arm_group.get_current_pose())
	
	# Build request items
	msi = moveit_msgs.msg.MotionSequenceItem()
	msi.req = r.arm_group.construct_motion_plan_request()
	msi.blend_radius = 0.0

	motion_plan_requests.append(msi)

	waypoints = [(pose1, 0.01), (pose2, 0.01), (pose3, 0.01), (pose4, 0.01), (pose5, 0.01)] # Pairs of pose, blend radius
	for wp, blend_radius in waypoints:
		r.arm_group.clear_pose_targets() 
		r.arm_group.set_pose_target(wp)
		msi = moveit_msgs.msg.MotionSequenceItem()
		msi.req = r.arm_group.construct_motion_plan_request()
		msi.req.start_state = moveit_msgs.msg.RobotState() # only the first point can have a non-empty start state
		msi.blend_radius = blend_radius
		motion_plan_requests.append(msi)
		
	# Force last point to be 0.0 to avoid raising an error in the planner
	motion_plan_requests[-1].blend_radius = 0.0
	
	# Make MotionSequence Request
	goal = moveit_msgs.msg.MoveGroupSequenceGoal()
	goal.request = moveit_msgs.msg.MotionSequenceRequest()
	goal.request.items = motion_plan_requests

	sequence_move_group_srv.send_goal_and_wait(goal)
	response = sequence_move_group_srv.get_result()
    
	r.arm_group.clear_pose_targets()
	
	if response.response.error_code.val == "1": 
		# The response is an array but I think there is always only one plan returned
		plan = response.planned_trajectories[0]
		r.arm_group.execute(plan)
if __name__ == '__main__':

	main()
