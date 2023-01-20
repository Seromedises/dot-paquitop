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

def plot_fct(filtred, velocity, title1, filtred2, velocity2, title2, filtred3, velocity3, title3):
  
  plt.subplot(2, 3, 1)
  plt.plot(filtred)
  plt.title(title1)
  #plt.xlabel("numbers of acquisitions")
  plt.ylabel("Force [N]")
  plt.grid()
  plt.ylim(ymax = 1.1*IN_max_Fx, ymin = -(1.1*IN_max_Fx))

  plt.subplot(2, 3, 4)
  plt.plot(velocity)
  plt.xlabel("numbers of acquisitions")
  plt.ylabel("velocity [m/s]")
  plt.grid()
  plt.ylim(ymax = 0.5 + OUT_max, ymin = -(0.5 + OUT_max))

  plt.subplot(2, 3, 2)
  plt.plot(filtred2)
  plt.title(title2)
  #plt.xlabel("numbers of acquisitions")
  plt.ylabel("Force [N]")
  plt.grid()
  plt.ylim(ymax = 1.1*IN_max_Fy, ymin = -(1.1*IN_max_Fy))

  plt.subplot(2, 3, 5)
  plt.plot(velocity2)
  plt.xlabel("numbers of acquisitions")
  plt.ylabel("velocity [m/s]")
  plt.grid()
  plt.ylim(ymax = 0.5 + OUT_max, ymin = -(0.5 + OUT_max))

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
  plt.ylim(ymax = 0.5 + OUT_max, ymin = -(0.5 + OUT_max))

  plt.draw()
  plt.pause(0.000001)
  plt.clf()

def length_control(variable,span=100):

  if len(variable) > span:
    variable = variable[-span:]
  
  return variable

def variable_control(variable, offset=0, span=100):

  variable = length_control(variable,50)

  if len(variable) < 10:
   offset = sum(variable)/len(variable)
  """
  dct = fftpack.dct(variable, norm="ortho")
  dct[8:] = 0
  filtred = fftpack.idct(dct, norm="ortho")
  """
  filtred = variable
  """
  for i in range(len(filtred)):
    filtred[i] = filtred[i]- offset 
  """
  return filtred, variable, offset

def force_to_velocity(IN, IN_lim = 1, IN_max = 5):
  
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
  vel_msg = Twist()
  start_position = Joint_position()
  rest_position_cmd.publish(start_position)
  rospy.sleep(1)
  
  Fx, Fy, Fz, Tx, Ty, Tz = [], [], [], [], [], []
  Fx_ofs, Fy_ofs, Fz_ofs, Tx_ofs, Ty_ofs, Tz_ofs = 0, 0, 0, 0, 0, 0
  vx, vy,wz = [], [], []
  Torque = np.array()

  start_position.value = [0, 340, 0, 90, 70, 0]#[90, -30, -60, 10, -60, -90] #[40, 330, 300, 40, 295, 300]
  for i in range(3):
    rest_position_cmd.publish(start_position)
  rospy.sleep(10)
  
  
  while not rospy.is_shutdown():
    data = rospy.wait_for_message("/my_gen3_lite/base_feedback/joint_state", JointState)
    
    Torque.append(list(data.effort))
    print(Torque)

    """
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

    cmd_vel.publish(vel_msg)
    title1 = "$F_x$ mean value and $v_x$ output value"
    title2 = "$F_y$ mean value and $v_y$ output value"
    title3 = "$T_z$ mean value and $\omega_z$ output value"
    """

    plot_fct(Fx_mean, vx , title1, Fy_mean, vy, title2, Tz_mean, wz , title3)
    

if __name__ == "__main__":
  main()