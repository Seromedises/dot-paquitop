#!/usr/bin/env python

from geometry_msgs.msg import Point
from pilz_robot_programming import *
import math
import rospy

__REQUIRED_API_VERSION__ = "1"

class MoveGroupPythonIntefaceTutorial(object):

	def __init__(self):

		# Initialize the node
		super(MoveGroupPythonIntefaceTutorial, self).__init__()
		# moveit_commander.roscpp_initialize(sys.argv)
		# rospy.init_node('constrain_path', anonymous=True)
		
		rospy.init_node('pliz_motion_planner', anonymous=True)
		self.r = Robot(__REQUIRED_API_VERSION__)
		
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

		

def main():
	
	 	 
	rospy.init_node('pliz_motion_planner', anonymous=True)
	example = MoveGroupPythonIntefaceTutorial()
	# r = Robot(__REQUIRED_API_VERSION__)
	 	 
	 # absolute Lin movement
	 # pose1 = Pose(position=Point(0.3, -0.25, 0.45), orientation=from_euler(90*pi/180, 0, 90*pi/180))
	example.r.move(Lin(goal=Pose(position=Point(0.2, -0.25, 0.45), orientation=from_euler(math.radians(90), 0, math.radians(90))), vel_scale=1, acc_scale=1))



if __name__ == '__main__':
  main()
