#!/usr/bin/env python

import sys
from threading import active_count
import time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import time
import numpy as np
from math import pi, sin
from std_srvs.srv import Empty
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class Quaternion():
	def __init__(self,w,x,y,z):
		self.w = w
		self.x = x
		self.y = y
		self.z = z

def ToQuaternion(yaw,pitch,roll):
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

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

def TransformMatrix(joint_number,joint_values):
	# Denavit-Hartenberg Modified parameters Luigi
	alpha = [0, pi/2, pi, pi/2, pi/2, -pi/2] #rad
	a = [0, 0, 0.280, 0, 0, 0] #meter
	d = [0.2433, 0, -0.01, 0.245, 0.057, 0.235] #meter
	theta = [joint_values[0], pi/2+joint_values[1], pi/2+joint_values[2], pi/2+joint_values[3], joint_values[4], -pi/2+joint_values[5]] #rad
	# matrix construction
	index = 0
	T = np.identity(4)

	while index < joint_number:

		# Submatrix that compose the Denavit-Hartenberg modified matrix
		Rot_X_alpha = [[1, 0, 0, 0],[0, math.cos(alpha[index]), -math.sin(alpha[index]), 0],[0, math.sin(alpha[index]), math.cos(alpha[index]),0],[0,0,0,1]]
		Trans_X_a = [[1,0,0,a[index]],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
		Trans_Z_d = [[1,0,0,0],[0,1,0,0],[0,0,1,d[index]],[0,0,0,1]]
		Rot_Z_theta = [[math.cos(theta[index]), -math.sin(theta[index]),0,0],[math.sin(theta[index]), math.cos(theta[index]),0,0],[0,0,1,0],[0,0,0,1]]
		
		# multiplcation between al the matrix
		B = np.matmul(Rot_X_alpha,Trans_X_a)
		B = np.matmul(B,Trans_Z_d)
		
		# Denavit-Hartenberg matrix betwenn the i-th link and the (i-1)-th link
		Tnew = np.matmul(B,Rot_Z_theta)
		
		# Upgrading matrix to build the total matrix between zero and the selected joint (defined by joint_number)
		T = np.matmul(T,Tnew)
		# print("\n\nMatrice Denavit numero "+str(index+1)+" fra base ed terminale:\n"+str(Tnew)+"\n\n")
		index = index+1

	return T

class MoveGroupPythonIntefaceTutorial(object):

	def __init__(self):

		# Initialize the node
		super(MoveGroupPythonIntefaceTutorial, self).__init__()
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('constrain_path', anonymous=True)

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
			self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
								                    moveit_msgs.msg.DisplayTrajectory,
								                    queue_size=50)

			if self.is_gripper_present:
				gripper_group_name = "gripper"
				self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

			rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
		except Exception as e:
			print (e)
			self.is_init_success = False
		else:
			self.is_init_success = True

      
      
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
		  
	def get_cartesian_pose(self):
		arm_group = self.arm_group

		# Get the current pose and display it
		pose = arm_group.get_current_pose()
		# rospy.loginfo("Actual cartesian pose is : ")
		rospy.loginfo(pose.pose)

		return pose.pose

	def ValidateMatrix(self,number_joint):
		pose = self.arm_group.get_current_pose()
		actual_pose = pose.pose
		joint = self.arm_group.get_current_joint_values()
		T = TransformMatrix(number_joint,joint)
		vect_pos = [[actual_pose.position.x], [actual_pose.position.y], [actual_pose.position.z],[1]]
		res = np.subtract(vect_pos,np.matmul(T,[[0],[0],[0],[1]]))
		print("\n\nPosizione organo terminale, differenza fra calcoalto e teorico:\n"+str(res)+"\n\n")


def main():

	example = MoveGroupPythonIntefaceTutorial()

	# For testing purposes
	success = example.is_init_success
	rospy.loginfo(example.is_init_success)

	try:
	  rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
	except:
	  pass
	  
	
	
	rospy.loginfo("Reach Cartesian Pose without constraints...")
	# Get actual pose
	actual_pose = example.get_cartesian_pose()
	actual_pose.position.x = 0.3
	actual_pose.position.y = -0.25
	actual_pose.position.z = 0.45

	yaw = 90*pi/180
	pitch = 0*pi/180
	roll = 90*pi/180
	
	q = ToQuaternion(yaw,  pitch,  roll) #yaw (Z), pitch (Y), roll (X)
	
	actual_pose.orientation.w = q.w
	actual_pose.orientation.x = q.x 
	actual_pose.orientation.y = q.y
	actual_pose.orientation.z = q.z

	# Send the goal
	success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None) 
	#time.sleep(2)
	example.ValidateMatrix(6)
     
	rospy.loginfo("Reach Cartesian Pose with constraints...")
	# Get actual pose
	actual_pose = example.get_cartesian_pose()
	actual_pose.position.x += 0
	actual_pose.position.y -= 0.2
	actual_pose.position.z += 0
	
	q = ToQuaternion(yaw,  pitch,  roll) #yaw (Z), pitch (Y), roll (X)
	# Orientation constraint (we want the end effector to stay the same orientation)
	constraints = moveit_msgs.msg.Constraints()
	orientation_constraint = moveit_msgs.msg.OrientationConstraint()
	orientation_constraint.orientation.w = q.w
	orientation_constraint.orientation.x = q.x 
	orientation_constraint.orientation.y = q.y
	orientation_constraint.orientation.z = q.z
	 
	constraints.orientation_constraints.append(orientation_constraint)

	# Send the goal
	success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
	#time.sleep(2)
	example.ValidateMatrix(6)

	rospy.loginfo("Reach Cartesian Pose with constraints...")
	# Get actual pose
	actual_pose = example.get_cartesian_pose()
	actual_pose.position.x += 0
	actual_pose.position.y += 0
	actual_pose.position.z -= 0.2

	# Send the goal
	success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
	#time.sleep(2)
	example.ValidateMatrix(6)

	rospy.loginfo("Reach Cartesian Pose with constraints...")
	# Get actual pose
	actual_pose = example.get_cartesian_pose()
	actual_pose.position.x += 0
	actual_pose.position.y += 0.2
	actual_pose.position.z += 0

	# Send the goal
	success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
	time.sleep(2)
	example.ValidateMatrix(6)

	rospy.loginfo("Reach Cartesian Pose with constraints...")
	# Get actual pose
	actual_pose = example.get_cartesian_pose()
	actual_pose.position.x += 0 
	actual_pose.position.y += 0
	actual_pose.position.z += 0.2

	# Send the goal
	success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
	#time.sleep(2)
	example.ValidateMatrix(6)
	
	if example.is_gripper_present:
		rospy.loginfo("Opening the gripper...")
		success &= example.reach_gripper_position(0.3)
		print (success)

		rospy.loginfo("Closing the gripper 50%...")
		success &= example.reach_gripper_position(0.5)
		print (success)
    
if __name__ == '__main__':
  main()
