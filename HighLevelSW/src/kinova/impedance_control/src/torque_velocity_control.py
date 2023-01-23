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
IN_max_Fx, IN_max_Fy, IN_max_Tz = 5, 5, 4# N, N and Nm
IN_min_Fx, IN_min_Fy, IN_min_Tz= 1.5, 1.5, 1 # N, N and Nm
filter_span = 50

def plot_fct(filtred, velocity, title1, filtred2, velocity2, title2, filtred3, velocity3, title3):
  
  plt.subplot(2, 3, 1)
  plt.plot(filtred)
  plt.title(title1)
  #plt.xlabel("numbers of acquisitions")
  plt.ylabel("Force [N]")
  plt.grid()
  #plt.ylim(ymax = 1.1*IN_max_Fx, ymin = -(1.1*IN_max_Fx))
  plt.ylim(ymax = 1.1*IN_max_Tz, ymin = -(1.1*IN_max_Tz))

  plt.subplot(2, 3, 4)
  plt.plot(velocity)
  plt.xlabel("numbers of acquisitions")
  plt.ylabel("velocity [m/s]")
  plt.grid()
  #plt.ylim(ymax = 0.5 + OUT_max, ymin = -(0.5 + OUT_max))
  plt.ylim(ymax = 1.1*IN_max_Tz, ymin = -(1.1*IN_max_Tz))

  plt.subplot(2, 3, 2)
  plt.plot(filtred2)
  plt.title(title2)
  #plt.xlabel("numbers of acquisitions")
  plt.ylabel("Force [N]")
  plt.grid()
  #plt.ylim(ymax = 1.1*IN_max_Fy, ymin = -(1.1*IN_max_Fy))
  plt.ylim(ymax = 1.1*IN_max_Tz, ymin = -(1.1*IN_max_Tz))

  plt.subplot(2, 3, 5)
  plt.plot(velocity2)
  plt.xlabel("numbers of acquisitions")
  plt.ylabel("velocity [m/s]")
  plt.grid()
  #plt.ylim(ymax = 0.5 + OUT_max, ymin = -(0.5 + OUT_max))
  plt.ylim(ymax = 1.1*IN_max_Tz, ymin = -(1.1*IN_max_Tz))

  plt.subplot(2, 3, 3)
  plt.plot(filtred3)
  plt.title(title3)
  #plt.xlabel("numbers of acquisitions")
  plt.ylabel("Torque [Nm]")
  plt.grid()
  plt.ylim(ymax = 1.1*IN_max_Tz, ymin = -(1.1*IN_max_Tz))

  plt.subplot(2, 3, 6)
  plt.plot(velocity3)
  plt.xlabel("numbers of acquisitions")
  plt.ylabel("angular velocity [rad/s]")
  plt.grid()
  #plt.ylim(ymax = 0.5 + OUT_max, ymin = -(0.5 + OUT_max))
  plt.ylim(ymax = 1.1*IN_max_Tz, ymin = -(1.1*IN_max_Tz))

  plt.draw
  plt.pause(0.0000001)
  plt.clf()

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
  
  start_position.value = [0, 340, 0, 90, 70, 0]#[90, -30, -60, 10, -60, -90] #[40, 330, 300, 40, 295, 300]
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

    """
    T1_mean.append(i for i in (filter(T1,filter_span,T1_ofs)))
    T2_mean.append(i for i in (filter(T2,filter_span,T2_ofs)))
    T3_mean.append(i for i in (filter(T3,filter_span,T3_ofs)))
    T4_mean.append(i for i in (filter(T4,filter_span,T4_ofs)))
    T5_mean.append(i for i in (filter(T5,filter_span,T5_ofs)))
    T6_mean.append(i for i in (filter(T6,filter_span,T6_ofs)))
    
    T1_mean = length_control(T1_mean,span=50)
    T2_mean = length_control(T2_mean,span=50)
    T3_mean = length_control(T3_mean,span=50)
    T4_mean = length_control(T4_mean,span=50)
    T5_mean = length_control(T5_mean,span=50)
    T6_mean = length_control(T6_mean,span=50)
    
    
    Fx_mean, Fx, Fx_ofs = variable_control(Fx, Fx_ofs, span=50)
    Fy_mean, Fy, Fy_ofs = variable_control(Fy, Fy_ofs, span=50)
    #Fz_mean, Fz, Fz_ofs = variable_control(Fz, Fz_ofs, span=50)
    #Tx_mean, Tx, Tx_ofs = variable_control(Tx, Tx_ofs, span=50)
    #Ty_mean, Ty, Ty_ofs = variable_control(Ty, Ty_ofs, span=50)
    Tz_mean, Tz, Tz_ofs = variable_control(Tz, Tz_ofs, span=50)

    vx.append(force_to_velocity(Fx_mean[-1],IN_lim=IN_min_Fx,IN_max=IN_max_Fx))
    vy.append(force_to_velocity(Fy_mean[-1],IN_lim=IN_min_Fy,IN_max=IN_max_Fy))
    wz.append(force_to_velocity(Tz_mean[-1],IN_lim=IN_min_Tz,IN_max=IN_max_Tz))
    vx = length_control(vx, span=50)
    vy = length_control(vy, span=50)
    wz = length_control(wz, span=50)
    
    vel_msg.linear.x = vx[-1]
    vel_msg.linear.y = vy[-1]
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = wz[-1]

    cmd_vel.publish(vel_msg)"""

    torque = Joint_position()
    torque.value = [T1_mean[-1], T2_mean[-1], T3_mean[-1], T4_mean[-1], T5_mean[-1], T6_mean[-1]]
    plot_fb.publish(torque)
      

      # plot_fct(T1, T2, "T1 and T2", T3, T4, "T3 and T4", T5, T6, "T5 and T6")
      # plot_fct(T1_mean, T2_mean , "T1 and T2", T3_mean, T4_mean, "T3 and T4", T5_mean, T6_mean , "T5 and T6")
      

if __name__ == "__main__":
  main()