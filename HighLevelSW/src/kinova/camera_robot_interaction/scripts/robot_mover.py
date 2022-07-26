#!/usr/bin/env python

import sys
# from threading import active_count, get_ident
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
from camera_robot_interaction.srv import *
# import pyrealsense2 as rs
# import cv2
# import matlab.engine
# from rospy.client import spin
# import copy
import rospkg

# from std_srvs.srv import Empty
# from std_msgs.msg import String
# import argparse
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
		rospy.init_node('robot_mover', )

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
			self.arm_group.set_max_velocity_scaling_factor(0.9)
			self.arm_group.set_max_acceleration_scaling_factor(0.9)
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

	def plan_execute_cartesian_path(self, waypoints, eef = 0.01,threshold = 0.0):
		
		plan, fraction = self.arm_group.compute_cartesian_path(waypoints,eef,threshold)
		self.arm_group.execute(plan,wait=True)

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
	
	if 1+r11-r22-r33>0:
		x = 0.5*np.sign(r32-r23)*math.sqrt(1+r11-r22-r33)
	elif w != 0:
		x = (r32-r23)/(4*w)

	if 1-r11+r22-r33>=0:
		y = 0.5*np.sign(r13-r31)*math.sqrt(1-r11+r22-r33)
	elif w != 0:
		y = (r13-r31)/(4*w)

	if 1-r11-r22+r33>=0:
		z = 0.5*np.sign(r21-r12)*math.sqrt(1-r11-r22+r33)
	elif w != 0:
		z = (r21-r12)/(4*w)

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

def stampa():
	rospy.wait_for_service('camera_server')
	try:
		camera_service = rospy.ServiceProxy('camera_server', SaveData)
		print("\nPose Matrix of link 5 respect link 0")
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
		f = open("../cw_kinova/calibration_tools/Robot_fb.txt", "a")
		f.write(line)
		f.write("\n")
		i = i+1
		j = 0
"""

def wait_for_state_update(scene, box_name, box_is_known=False, box_is_attached=False, timeout=4):
	start = rospy.get_time()
	seconds = rospy.get_time()
	while (seconds - start < timeout) and not rospy.is_shutdown():
		# Test if the box is in attached objects
		attached_objects = scene.get_attached_objects([box_name])
		is_attached = len(attached_objects.keys()) > 0

		# Test if the box is in the scene.
		# Note that attaching the box will remove it from known_objects
		is_known = box_name in scene.get_known_object_names()

	# Test if we are in the expected state
	if (box_is_attached == is_attached) and (box_is_known == is_known):
		return True

	# Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	seconds = rospy.get_time()

	# If we exited the while loop without returning then we timed out
	return False

def mover_robot(req):

	# global value
	global Cam_matr
	global move_robot
	global grip_pos
	global rotate
	global rest_pos

	# info from camera	
	Cam_matr = [[req.r11, req.r12, req.r13, req.r14],[req.r21, req.r22, req.r23, req.r24],[req.r31, req.r32, req.r33, req.r34],[0,0,0,1]]
	Cam_matr = np.array(Cam_matr)
	grip_pos = req.gripper 
	rotate = req.rotate
	rest_pos = req.rest_pos
	move_robot = True
	print("Cam matrix:")
	print(Cam_matr)

	return MovementResponse(1,grip_pos)
	


def main():

	example = MoveGroupPythonIntefaceTutorial()
	# For testing purposes
	rospy.loginfo(example.is_init_success)
	global Cam_matr
	global move_robot
	global grip_pos
	global rotate
	global rest_pos
	move_robot = False
	rotate = 0
	rest_pos = 0
	last_gripper = 0 
	
	try:
		rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
	except:
		pass

	# open the gripper
	example.reach_gripper_position(1)

	# adding box that is present in real world
	"""
	is_known = False
	while not is_known:
		scene = example.scene
		box_name = ['box','wall']
		box_size = (0.2, 0.5,0.39)
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = "base_link"
		box_pose.pose.position.x = 0.48+0.2*0.5
		box_pose.pose.position.y = 0
		box_pose.pose.position.z = 0.35/2
		box_pose.pose.orientation.w = 1
		box_pose.pose.orientation.x = 0
		box_pose.pose.orientation.y = 0
		box_pose.pose.orientation.z = 0
		scene.add_box(box_name[0], box_pose, box_size)
		# wait_for_state_update(scene, box_name[0], box_is_known=True)
		time.sleep(3)

		box_size = (0.02, 1, 1)
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = "base_link"
		box_pose.pose.position.x = 0.50+0.15
		box_pose.pose.position.y = 0
		box_pose.pose.position.z = 0.5
		box_pose.pose.orientation.w = 1
		box_pose.pose.orientation.x = 0
		box_pose.pose.orientation.y = 0
		box_pose.pose.orientation.z = 0
		scene.add_box(box_name[1], box_pose, box_size)
		# wait_for_state_update(scene, box_name[1], box_is_known=True)
		time.sleep(2.5)
		is_known = box_name[0] in scene.get_known_object_names()
	"""
	far_pose = example.get_cartesian_pose()
	# starting server
	rospy.Service("robot_mover", Movement, mover_robot)
	rospy.loginfo("Enviroment loaded, ready to recieve input")

	try:
		while not rospy.is_shutdown():
			
			if move_robot:
				# Empty_matrix = False
			
				if float(Cam_matr[0,3]) == 0.0 and float(Cam_matr[1,3]) == 0.0 and float(Cam_matr[2,3]) == 0.0:
					print("Only gripper movement or standard movement is possible")
					# print("\n## Choose a Movement ##")
					# Empty_matrix = True

					if (rotate == 1):
					
						pose = example.get_cartesian_pose()
						
						quat = Quaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z)
						Pose_Matrix = QuaternionToMatrix(quat,pose.position.x,pose.position.y,pose.position.z)
						rot = -pi/6
						Rot_matrix = np.array([[math.cos(rot),-math.sin(rot),0,0],[math.sin(rot),math.cos(rot),0,0],[0,0,1,0],[0,0,0,1]])
						Pose_Matrix = np.matmul(Rot_matrix,Pose_Matrix)
						print("Rotation around Z axis of the fixed frame")
						print(Pose_Matrix)
						q = MatrixToQuaternion(Pose_Matrix)
						pose.orientation.w = q.w
						pose.orientation.x = q.x
						pose.orientation.y = q.y
						pose.orientation.z = q.z
						pose.position.x = Pose_Matrix[0,3]
						pose.position.y = Pose_Matrix[1,3]
						pose.position.z = Pose_Matrix[2,3]
						example.reach_cartesian_pose(pose=pose, tolerance=0.01, constraints=None)
						far_pose = example.get_cartesian_pose()

					elif (rest_pos == 1):
												
						print("Return in far position")
						iter = 0
						success = example.reach_cartesian_pose(pose=far_pose, tolerance=0.01, constraints=None)
						while not success and iter < 6:
							success = example.reach_cartesian_pose(pose=far_pose, tolerance=0.01, constraints=None)
							iter = iter +1
						print("Return in rest position")

						

						iter = 0
						success = example.rest_pos()
						while not success and iter < 6:
							example.rest_pos()
							iter = iter +1
						far_pose = example.get_cartesian_pose()

					
				else:
					file = rospkg.RosPack().get_path('kortex_tools')+"/calibration_tools/Calib_Matrix.txt"
					Cal_matr = np.loadtxt(file, delimiter=",")
					feed_matr = example.ValidateMatrix(6) #metti 5 o 6 a seconda del giunto in cui e' posta la camera
					
					Robot_matr = np.matmul(Cal_matr,Cam_matr)
					Robot_matr = np.matmul(feed_matr,Robot_matr)
					print("Robot Matrix:")
					print(Robot_matr)

					q = MatrixToQuaternion(Robot_matr)
					far = 0.1
					far = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-far],[0,0,0,1]])
					Robot_matr_far = np.matmul(Robot_matr,far)
					x = Robot_matr_far[0,3]
					y = Robot_matr_far[1,3]
					z = Robot_matr_far[2,3]

					# orientation of gripper in the right position
					actual_pose = example.get_cartesian_pose()
					actual_pose.orientation.w = q.w
					actual_pose.orientation.x = q.x 
					actual_pose.orientation.y = q.y
					actual_pose.orientation.z = q.z
					actual_pose.position.x = x # m
					actual_pose.position.y = y # m
					actual_pose.position.z = z # m

					attempt = 0
					reach = example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
					while not reach and attempt <6:
						reach = example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
						attempt = attempt+1

					if reach:
						# waypoints = []
						far_pose = example.get_cartesian_pose()
						"""
						far_poseM = QuaternionToMatrix(Quaternion(far_pose.orientation.w,far_pose.orientation.x,far_pose.orientation.y,far_pose.orientation.z),far_pose.position.x,far_pose.position.y,far_pose.position.z)
						h = 0.05 # m
						h = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,h],[0,0,0,1]])
						far_poseM = np.matmul(h,far_poseM)
						far_pose.position.x = far_poseM[0,3]
						far_pose.position.y = far_poseM[1,3]
						far_pose.position.z = far_poseM[2,3]
						"""
						far_pose.position.z = far_pose.position.z + 0.05 #+ 0.05

						# far = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-distance],[0,0,0,1]])
						# Robot_matr_far = np.matmul(Robot_matr,far)
						x = Robot_matr[0,3]
						y = Robot_matr[1,3]
						z = Robot_matr[2,3]

						# gripper approaching
						constraints = moveit_msgs.msg.Constraints()
						orientation_constraint = moveit_msgs.msg.OrientationConstraint()
						orientation_constraint.orientation = actual_pose.orientation
						constraints.orientation_constraints.append(orientation_constraint)
						actual_pose.position.x = x # float(x[0,0]) # m
						actual_pose.position.y = y # float(y[0,0]) # m
						actual_pose.position.z = z # float(z[0,0]) # m
							
						number = 6
						attempt = 0
						done = example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
						while attempt<number and not done:
							
							if attempt == number-1:
								example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
							else:
								done = example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)
							attempt = attempt +1
							"""
							wpose = example.arm_group.get_current_pose().pose
							wpose.position.x = float(x[0,0]) # m
							wpose.position.y = float(y[0,0]) # m
							wpose.position.z = float(z[0,0]) # m
							waypoints.append(copy.deepcopy(wpose))
							"""
						# example.plan_execute_cartesian_path(waypoints,eef=0.1,threshold=0.1)

					# print("\n## Choose a Movement ##")

				# TODO mettere un controllo sul gripper per evitare l'errore
				# if grip_pos != last_gripper:
				iter = 0
				i = 0
				open_command = False
				
				# last_gripper = grip_pos
				if grip_pos == 1:
					open_command = True

				while i < 2:
					i = i+1
					success = example.reach_gripper_position(grip_pos)
					"""
					while not success and iter < 8:
						success = example.reach_gripper_position(grip_pos)
						iter = iter +1
						i

						if open_command:
							grip_pos = grip_pos-0.05*iter
						
						else:
							grip_pos = grip_pos + 0.05*iter
						
						print(success)
					"""

				
				print("Command terminated")
				
				# redefine al variables
				move_robot = False
				rotate = 0
				rest_pos = 0
		"""
		if rotate == 0.0 and rest_pos != 1:
			print("Return in far position")
			iter = 0
			success = False
			while not success and iter < 4:
				success = example.reach_cartesian_pose(pose=far_pose, tolerance=0.01, constraints=None)
				iter = iter +1	

		print("Return in rest position")
		example.rest_pos()	
		"""
		

	finally:
		"""
		if rotate == 0.0 and rest_pos != 1:
			print("Return in far position")
			example.reach_cartesian_pose(pose=far_pose, tolerance=0.01, constraints=None)
		"""	

		# for i in box_name:
		# 	scene.remove_world_object(i)

	

if __name__ == '__main__':
    main()
