#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

#  TO DO:
# - read the torque from the joint motors from the driver
# - elabarotion of the signal
# - send the joint velocity to the robotic arm driver 

def joint_state_callback(msg):
    torque_values= msg.effort

    print("Torque_values:" , torque_values)


def main():
    rospy.init_node("torque_reader")
    rospy.subscriber("joint_state", JointState, joint_state_callback)
    rospy.spin()
if __name__ ==  "main":
    main() 
