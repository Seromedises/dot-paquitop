#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class position():
	def __init__(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z

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
	
	return q;

class MoveGroupPythonIntefaceTutorial(object):

	def __init__(self):
		super(MoveGroupPythonIntefaceTutorial, self).__init__()

		## BEGIN_SUB_TUTORIAL setup
		##
		## First initialize `moveit_commander`_ and a `rospy`_ node:
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('rviz_object_spawn', anonymous=True)
		
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
			self.arm_group.set_num_planning_attempts(20)
			self.box_name = ''
			self.table_name = ''
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
		
		'''
		## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
		## kinematic model and the robot's current joint states
		robot = moveit_commander.RobotCommander("robot_description")

		## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
		## for getting, setting, and updating the robot's internal understanding of the
		## surrounding world:
		scene = moveit_commander.PlanningSceneInterface()

		## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		## to a planning group (group of joints).  In this tutorial the group is the primary
		## arm joints in the Panda robot, so we set the group's name to "panda_arm".
		## If you are using a different robot, change this value to the name of your robot
		## arm planning group.
		## This interface can be used to plan and execute motions:
		group_name = "arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)

		## Create a `DisplayTrajectory`_ ROS publisher which is used to display
		## trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
				                                           moveit_msgs.msg.DisplayTrajectory,
				                                           queue_size=20)	
		'''
	
	def wait_for_state_update(self, name, box_is_known=False, box_is_attached=False, timeout=4):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		box_name = name
		scene = self.scene

		## BEGIN_SUB_TUTORIAL wait_for_scene_update
		##
		## Ensuring Collision Updates Are Receieved
		## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		## If the Python node dies before publishing a collision object update message, the message
		## could get lost and the box will not appear. To ensure that the updates are
		## made, we wait until we see the changes reflected in the
		## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
		## For the purpose of this tutorial, we call this function after adding,
		## removing, attaching or detaching an object in the planning scene. We then wait
		## until the updates have been made or ``timeout`` seconds have passed
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
	
		
	def attach_box(self, timeout=4):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		box_name = self.box_name
		robot = self.robot
		scene = self.scene
		eef_link = self.eef_link
		group_names = self.group_names

		## BEGIN_SUB_TUTORIAL attach_object
		##
		## Attaching Objects to the Robot
		## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
		## robot be able to touch them without the planning scene reporting the contact as a
		## collision. By adding link names to the ``touch_links`` array, we are telling the
		## planning scene to ignore collisions between those links and the box. For the Panda
		## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
		## you should change this value to the name of your end effector group name.
		grasping_group = 'end_effector'
		touch_links = robot.get_link_names(group=grasping_group)
		scene.attach_box(eef_link, box_name, touch_links=touch_links)
		## END_SUB_TUTORIAL

		# We wait for the planning scene to update.
		return self.wait_for_state_update(name, box_is_attached=True, box_is_known=False, timeout=timeout)


	def detach_box(self, timeout=4):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		box_name = self.box_name
		scene = self.scene
		eef_link = self.eef_link

		## BEGIN_SUB_TUTORIAL detach_object
		##
		## Detaching Objects from the Robot
		## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		## We can also detach and remove the object from the planning scene:
		scene.remove_attached_object(eef_link, name=box_name)
		## END_SUB_TUTORIAL

		# We wait for the planning scene to update.
		return self.wait_for_state_update(name, box_is_known=True, box_is_attached=False, timeout=timeout)


	def remove_box(self, timeout=20):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		box_name = self.box_name
		scene = self.scene

		## BEGIN_SUB_TUTORIAL remove_object
		##
		## Removing Objects from the Planning Scene
		## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		## We can remove the box from the world.
		scene.remove_world_object(box_name)

		## **Note:** The object must be detached before we can remove it from the world
		## END_SUB_TUTORIAL

		# We wait for the planning scene to update.
		return self.wait_for_state_update(name, box_is_attached=False, box_is_known=False, timeout=timeout)
		
	def add_box(self, name, position, quaternion, size, timeout=4):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		box_name = self.box_name
		scene = self.scene

		## BEGIN_SUB_TUTORIAL add_box
		##
		## Adding Objects to the Planning Scene
		## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		## First, we will create a box in the planning scene at the location of the left finger:
		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = "base_link"
		
		box_pose.pose.orientation.w = quaternion.w
		box_pose.pose.orientation.x = quaternion.x
		box_pose.pose.orientation.y = quaternion.y
		box_pose.pose.orientation.z = quaternion.z
		box_pose.pose.position.x = position.x 
		box_pose.pose.position.y = position.y
		box_pose.pose.position.z = position.z
		
		box_name = name
		scene.add_box(box_name, box_pose, size)

		## END_SUB_TUTORIAL
		# Copy local variables back to class variables. In practice, you should use the class
		# variables directly unless you have a good reason not to.
		self.box_name=box_name
		return self.wait_for_state_update(name, box_is_known=True, timeout=20)
		
	def add_table(self, name, position, quaternion, timeout=4):
		# Copy class variables to local variables to make the web tutorials more clear.
		# In practice, you should use the class variables directly unless you have a good
		# reason not to.
		table_name = self.table_name
		scene = self.scene
		
		## BEGIN_SUB_TUTORIAL add_box
		##
		## Adding Objects to the Planning Scene
		## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		## First, we will create a box in the planning scene at the location of the left finger:
		table_pose = geometry_msgs.msg.PoseStamped()
		table_pose.header.frame_id = "base_link"
		
		table_pose.pose.orientation.w = quaternion.w
		table_pose.pose.orientation.x = quaternion.x
		table_pose.pose.orientation.y = quaternion.y
		table_pose.pose.orientation.z = quaternion.z
		table_pose.pose.position.x = position.x 
		table_pose.pose.position.y = position.y
		table_pose.pose.position.z = position.z
		
		filename = "~/cw_kinova/src/kortex_planner/stl/table.stl"#"~/cw_kinova/src/kortex_planner/stl/Table.STL"
		
		table_name = name
		scene.add_mesh(table_name, table_pose, filename, size=(1,1,1))
		
		self.table_name=table_name
		return self.wait_for_state_update(name,box_is_known=True, timeout=20)

    
def main():
	tutorial = MoveGroupPythonIntefaceTutorial()
	
	# spawning floor
	q = ToQuaternion(0,0,0)
	p = position(0,0,-0.2)
	size = (2,2,0.1)
	name = 'floor'
	tutorial.add_box(name, position = p,quaternion = q,size = size)
	
	q = ToQuaternion(0,0,0)
	p = position(0,0,-0.15)
	size = (0.5,0.3,0.3)
	name = 'paquito'
	tutorial.add_box(name, position = p,quaternion = q,size = size)
	
	q = ToQuaternion(0,0,0)
	p = position(1,1,-0.15)
	name = 'desk'
	#tutorial.add_table(name,p,q)
	
	print("READY")
	raw_input()
	tutorial.remove_box()

if __name__ == '__main__':
  main()
