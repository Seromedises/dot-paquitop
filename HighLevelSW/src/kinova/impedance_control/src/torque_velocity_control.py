#!/usr/bin/env python3

import numpy as np
import scipy.fftpack as fftpack
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from paquitop.msg import Joint_position


# Constant for translate input force in velocity output
OUT_max = 0.4 # m/s
OUT_lim = 0.1 # m/s
IN_max_T3, IN_max_T4, IN_max_T6 = 3, 2, 2# N, N and Nm
IN_min_T3, IN_min_T4, IN_min_T6= 1.0, 0.5, 0.5 # N, N and Nm
filter_span = 50

def length_control(variable,span=100):

  if len(variable) > span:
    variable = variable[-span:]
  
  return variable

def offset(variable, offset):

  
  if len(variable) < 10:
   offset = sum(variable)/len(variable)
  
  return offset

def filter(variable,span,offset):

  variable = length_control(variable,span)
  dct = fftpack.dct(variable, norm="ortho")
  dct[8:] = 0
  filtred = fftpack.idct(dct, norm="ortho")
  for i in range(len(filtred)):
    filtred[i] = float(filtred[i]- offset)
  
  return list(filtred)

def to_velocity(IN, IN_lim = 1, IN_max = 5):
  
  # IN_lim and IN_max are N or Nm 

  velocity = 0
  
  b1 = (OUT_max-OUT_lim)/(IN_max-IN_lim)
  b0 = OUT_max - b1*IN_max
  a3 = 2*(5*OUT_max - 5*b1*IN_max + 3*b1*IN_lim)/(IN_lim**3)
  a4 = -(15*OUT_max - 15*b1*IN_max + 8*b1*IN_lim)/(IN_lim**4)
  a5 = 3*(2*OUT_max - 2*b1*IN_max + b1*IN_lim)/(IN_lim**5)

  # input control
  if IN <= -IN_lim:
    velocity = b1 * IN - b0
  elif IN > -IN_lim and IN <= 0:
    IN = -IN
    velocity = -(a5*(IN**5) + a4*(IN**4) + a3*(IN**3))
  elif IN > 0 and IN <= IN_lim:
    velocity = a5*(IN**5) + a4*(IN**4) + a3*(IN**3)
  elif IN > IN_lim:
    velocity = b1 * IN + b0

  # saturation control
  if velocity <-OUT_max:
    velocity = -OUT_max 
  elif velocity > OUT_max:
    velocity = OUT_max

  return velocity

def main():
  rospy.init_node('torque_velocity_control')
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  rest_position_cmd = rospy.Publisher('/joint_angles', Joint_position, queue_size=1)
  plot_fb = rospy.Publisher('/torque',Joint_position,queue_size=10)
  vel_msg = Twist()
  start_position = Joint_position()
  rest_position_cmd.publish(start_position)
  rospy.sleep(1)
  
  T1_ofs, T2_ofs, T3_ofs, T4_ofs, T5_ofs, T6_ofs = 0, 0, 0, 0, 0, 0
  vx, vy,wz = [], [], []
  T1, T2, T3, T4, T5, T6 = [], [], [], [], [], []
  T1_mean, T2_mean, T3_mean, T4_mean, T5_mean, T6_mean = [], [], [], [], [], []
  
  start_position.value = [270, 0, 0, 0, 0, 0]#[0, 340, 0, 90, 25, 0]#[90, -30, -60, 10, -60, -90] #[40, 330, 300, 40, 295, 300]
  for i in range(3):
    rest_position_cmd.publish(start_position)
  rospy.sleep(10)
  
  while not rospy.is_shutdown():
    data = rospy.wait_for_message("/my_gen3_lite/base_feedback/joint_state", JointState)

    T1.append(data.effort[0])
    T2.append(data.effort[1])
    T3.append(data.effort[2])
    T4.append(data.effort[3])
    T5.append(data.effort[4])
    T6.append(data.effort[5])

    T1 = length_control(T1,span=50)
    T2 = length_control(T2,span=50)
    T3 = length_control(T3,span=50)
    T4 = length_control(T4,span=50)
    T5 = length_control(T5,span=50)
    T6 = length_control(T6,span=50)
    
    T1_ofs = offset(T1,T1_ofs)
    T2_ofs = offset(T2,T2_ofs)
    T3_ofs = offset(T3,T3_ofs)
    T4_ofs = offset(T4,T4_ofs)
    T5_ofs = offset(T5,T5_ofs)
    T6_ofs = offset(T6,T6_ofs)

    T1_mean = filter(T1,filter_span,T1_ofs)
    T2_mean = filter(T2,filter_span,T2_ofs)
    T3_mean = filter(T3,filter_span,T3_ofs)
    T4_mean = filter(T4,filter_span,T4_ofs)
    T5_mean = filter(T5,filter_span,T5_ofs)
    T6_mean = filter(T6,filter_span,T6_ofs)

    vx.append(to_velocity(T5_mean[-1],IN_lim=IN_min_T3,IN_max=IN_max_T3))
    vy.append(to_velocity(T6_mean[-1],IN_lim=IN_min_T6,IN_max=IN_max_T6))
    wz.append(to_velocity(T4_mean[-1],IN_lim=IN_min_T4,IN_max=IN_max_T4))
    vx = length_control(vx, span=50)
    vy = length_control(vy, span=50)
    wz = length_control(wz, span=50)
    

    # controll to disacopiate vx and vy
    if abs(vy[-1])<0.02: #abs(vx[-1])> abs(vy[-1]) or abs(wz[-1])> abs(vy[-1]):
      vel_msg.linear.x = vx[-1]
      vel_msg.linear.y = 0
      vel_msg.linear.z = 0
      vel_msg.angular.x = 0
      vel_msg.angular.y = 0
      vel_msg.angular.z = wz[-1]

    else:
      vel_msg.linear.x = 0
      vel_msg.linear.y = vy[-1]
      vel_msg.linear.z = 0
      vel_msg.angular.x = 0
      vel_msg.angular.y = 0
      vel_msg.angular.z = 0

    torque = Joint_position()
    torque.value = [T1_mean[-1], T2_mean[-1], T3_mean[-1], T4_mean[-1], T5_mean[-1], T6_mean[-1]]
    
    plot_fb.publish(torque)
    cmd_vel.publish(vel_msg)
    
if __name__ == "__main__":
  main()