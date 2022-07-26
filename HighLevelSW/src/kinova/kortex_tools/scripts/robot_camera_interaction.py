#!/usr/bin/env python

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import time
import numpy as np
from math import pi
from moveit_commander.conversions import pose_to_list
# from kortex_planner.srv import *
from kortex_tools.srv import *
import pyrealsense2 as rs
import cv2
import matlab.engine
import rospkg

# from std_srvs.srv import Empty
# from std_msgs.msg import String
# import argparse
# import copy
# from numpy.core.defchararray import join
# from numpy.core.records import fromrecords
# from threading import active_count
# from numpy.lib.npyio import save
# from numpy.linalg.linalg import _raise_linalgerror_eigenvalues_nonconvergence


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
	# Denavit-Hartenberg Modified parameters
	alpha = [0, pi/2, pi, pi/2, pi/2, -pi/2] # rad
	a = [0, 0, 0.280, 0, 0, 0] # meter
	d = [0.2433, 0, -0.01, 0.245, 0.057, 0.235] # meter
	theta = [joint_values[0], pi/2+joint_values[1], pi/2+joint_values[2], pi/2+joint_values[3], joint_values[4], joint_values[5]] #rad
	
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
		rospy.init_node('robot_camera_interaction', )

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
		
		return pose.pose

	def get_joint(self):
		
		joint = self.arm_group.get_current_joint_values()
		joint = np.array(joint)

		return joint

	def ValidateMatrix(self,number_joint):

		# get joint
		joint = self.arm_group.get_current_joint_values()
		joint = np.array(joint)
		# get trasform matrix
		T = TransformMatrix(number_joint,joint)
		T = np.asanyarray(T)

		return T

	def rest_pos(self):
		self.arm_group.set_goal_joint_tolerance(0.01)
		joint_goal = self.arm_group.get_current_joint_values()
		joint_goal[0] = -3.0*pi/180
		joint_goal[1] = 21*pi/180
		joint_goal[2] = 145*pi/180
		joint_goal[3] = 92*pi/180 #-88+180
		joint_goal[4] = -33*pi/180
		joint_goal[5] = 90*pi/180

		self.arm_group.set_joint_value_target(joint_goal)
		return self.arm_group.go(wait=True)

class Quaternion():
	def __init__(self,w,x,y,z):
		self.w = w
		self.x = x
		self.y = y
		self.z = z

def QuaternionToMatrix(q,x,y,z):
	
	r11 = 1 - 2*q.y**2 - 2*q.z**2
	r12 = 2*(q.x*q.y-q.z*q.w)
	r13 = 2*(q.x*q.z+q.y*q.w)
	r14 = x

	r21 = 2*(q.x*q.y+q.z*q.w)
	r22 = 1 - 2*q.x**2 -2*q.z**2
	r23 = 2*(q.y*q.z-q.x*q.w)
	r24 = y

	r31 = 2*(q.x*q.z-q.y*q.w)
	r32 = 2*(q.y*q.z+q.x*q.w)
	r33 = 1-2*q.x**2-2*q.y**2
	r34 = z

	Mtx = np.asanyarray([[r11, r12, r13, r14],[r21, r22, r23, r24],[r31, r32, r33, r34],[0, 0, 0, 1]])

	return Mtx
	
def MatrixToQuaternion(Mtx):
	r11 = Mtx[0][0]
	r22 = Mtx[1][1]
	r33 = Mtx[2][2]

	r32 = Mtx[2][1]
	r23 = Mtx[1][2]

	r13 = Mtx[0][2]
	r31 = Mtx[2][0]

	r21 = Mtx[1][0]
	r12 = Mtx[0][1]

	if 1+r11+r22+r33<10**(-32):
		w = 0
	else:
		w = 0.5*math.sqrt(1+r11+r22+r33)
	
	x = 0.5*np.sign(r32-r23)*math.sqrt(1+r11-r22-r33)

	y = 0.5*np.sign(r13-r31)*math.sqrt(1-r11+r22-r33)
	
	z = 0.5*np.sign(r21-r12)*math.sqrt(1-r11-r22+r33)

	return Quaternion(w,x,y,z)
"""
def YPRToMatrix(yaw,pitch,roll,actual_pose): #yaw (Z), pitch (Y), roll (X)
		
	x = float(actual_pose.position.x)
	y = float(actual_pose.position.y)
	z = float(actual_pose.position.z)

	Yaw = np.asanyarray([[math.cos(yaw),-math.sin(yaw),0,0],[math.sin(yaw),math.cos(yaw),0,0],[0,0,1,0],[0,0,0,1]])
	#print(Yaw)

	Pitch = np.asanyarray([[math.cos(pitch),0,math.sin(pitch),0],[0,1,0,0],[-math.sin(pitch),0,math.cos(pitch),0],[0,0,0,1]])
	#print(Pitch)
	
	Roll = np.asanyarray([[1,0,0,0],[0, math.cos(roll),-math.sin(roll),0],[0,math.sin(roll),math.cos(roll),0],[0,0,0,1]])
	
	Pos = [[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]]
	
	Eye = np.identity(4)
	rot = np.matmul(Eye,Yaw)
	rot = np.matmul(Pitch,rot)
	rot = np.matmul(Roll,rot)
	T = np.asanyarray(rot)

	return T
"""
def stampa():
	rospy.wait_for_service('camera_server')
	try:
		camera_service = rospy.ServiceProxy('camera_server', SaveData)
		print("\nPose Matrix of link 6 respect link 0")
		print(feedback)
		save_file(feedback)
		resp1 = camera_service(True)

		return resp1.Success
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
	
def save_file(Matrix):
	Row = Matrix.shape[0]
	Col = Matrix.shape[1]
	
	i = 0
	j = 0
	
	while i<Row:
		line = ""
		while j<Col:
			line = line+str(Matrix[i][j])+" "
			j = j+1
		file_path = rospkg.RosPack().get_path('kortex_tools')+"/calibration_tools/Robot_fb.txt"
		f = open(file_path, "a")
		f.write(line)
		f.write("\n")
		i = i+1
		j = 0

def main():

	example = MoveGroupPythonIntefaceTutorial()
	# For testing purposes
	rospy.loginfo(example.is_init_success)
	try:
		rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
	except:
		pass

	x = rospy.get_param('~x')
	y = rospy.get_param('~y')
	z = rospy.get_param('~z')
	sign_inv = rospy.get_param('~sign_inversion')
	num_theta = rospy.get_param('~cone_step')
	num_step = rospy.get_param('~rotation_step')

	## Angles of the cone seleceted for calibration
	possible_theta = np.linspace(5,30,num=num_theta) # deg
	step_pos = np.linspace(0,180,num=num_step) # deg
	step_pos = step_pos*pi/180

	if sign_inv == 1:
		step_pos = step_pos+ pi*np.ones(step_pos.shape) # deg
	
	global f
	file_path = rospkg.RosPack().get_path('kortex_tools')+"/calibration_tools/Robot_fb.txt"
	f = open(file_path, "w")
	actual_pose = example.get_cartesian_pose()

	# close gripper to measure better
	print(example.reach_gripper_position(0))

	# initial pose
	yaw = (270)*pi/180
	pitch = 180*pi/180
	roll = (0)*pi/180
	q = ToQuaternion(yaw,  pitch,  roll) #yaw (Z), pitch (Y), roll (X)

	actual_pose.orientation.w = q.w
	actual_pose.orientation.x = q.x 
	actual_pose.orientation.y = q.y
	actual_pose.orientation.z = q.z

	actual_pose.position.z = z # m
	actual_pose.position.x = x # m
	actual_pose.position.y = y - z*math.tan(roll) # m

	if sign_inv == 1:
		actual_pose.position.x = -actual_pose.position.x 
		actual_pose.position.y = -actual_pose.position.y 

	example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None) 
	time.sleep(1)
	global feedback
	feedback = example.ValidateMatrix(6)
	ok = stampa()
	if not ok:
		print("Error on saving")
	
	
	i=0 # iteration counter
	for z_rot in step_pos:
		
		i = i+1
		print("\n#######################")
		print("##### ITERATION " +str(i)+"######" )	
		
		for theta in possible_theta:

			yaw = (180)*pi/180
			pitch = 180*pi/180
			roll = (theta)*pi/180
			q = ToQuaternion(yaw,  pitch,  roll) #yaw (Z), pitch (Y), roll (X)
			actual_pose.orientation.w = q.w
			actual_pose.orientation.x = q.x 
			actual_pose.orientation.y = q.y
			actual_pose.orientation.z = q.z
			
			actual_pose.position.z = z # m
			actual_pose.position.x = x # m
			actual_pose.position.y = y - z*math.tan(roll) # m
			if sign_inv == 1:
				actual_pose.position.x = -actual_pose.position.x #m
				actual_pose.position.y = -y - z*math.tan(roll) # m
			
			# from pose is calculated the matrix pose for one position and then it will be rotated
			cal = QuaternionToMatrix(q,actual_pose.position.x,actual_pose.position.y,actual_pose.position.z)
			# rotation matrix
			Rot_matrix = np.asanyarray([[math.cos(z_rot),-math.sin(z_rot),0],[math.sin(z_rot),math.cos(z_rot),0],[0,0,1]])
			
			# rotate matrix of pose
			rot_input = cal[[0,1,2],:]
			rot_input = rot_input[:,[0,1,2]]
			rot_input = np.matmul(Rot_matrix,rot_input)
			rot_input = np.asanyarray(rot_input)

			# get a quaternion from rotated pose matrix
			qin = MatrixToQuaternion(rot_input)
			
			actual_pose.orientation.w = qin.w
			actual_pose.orientation.x = qin.x 
			actual_pose.orientation.y = qin.y
			actual_pose.orientation.z = qin.z
			
			actual_pose.position.z = z # m
			actual_pose.position.x = x + z*math.tan(roll)*math.sin(z_rot) # m
			actual_pose.position.y = y - z*math.tan(roll)*math.cos(z_rot) # m
			if sign_inv == 1:
				actual_pose.position.x = -x + z*math.tan(roll)*math.sin(z_rot) # m
				actual_pose.position.y = -y - z*math.tan(roll)*math.cos(z_rot) # m

			example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
			time.sleep(1)
			feedback = example.ValidateMatrix(6)
			ok = stampa()

			if not ok:
				print("Error on saving")
		
	example.rest_pos()

	# Running matlab calibration code
	print("\nMatlab searching for calibration matrix with acquired data...")
	eng = matlab.engine.start_matlab()
	s = rospkg.RosPack().get_path('kortex_tools')+"/calibration_tools"
	# s = '../cw_kinova/src/kortex_tools/calibration_tools'
	eng.cd(s, nargout=0)
	eng.Calibration_matirx_finder()
	eng.quit

if __name__ == '__main__':
    main()
