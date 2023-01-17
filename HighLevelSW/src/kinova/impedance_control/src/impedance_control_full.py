#!/usr/bin/env python

import scipy.fftpack as fftpack
import matplotlib.pyplot as plt
import rospy
from kortex_driver.msg import BaseCyclic_Feedback
from geometry_msgs.msg import Twist

# Constant for translate input force in velocity output
OUT_max = 0.4 # m/s
OUT_lim = 0.1 # m/s

IN_lim = 1 # N
IN_max = 5 # N

b1 = (OUT_max-OUT_lim)/(IN_max-IN_lim)
b0 = OUT_max - b1*IN_max
a3 = 2*(5*OUT_max - 5*b1*IN_max + 3*b1*IN_lim)/(IN_lim**3)
a4 = -(15*OUT_max - 15*b1*IN_max + 8*b1*IN_lim)/(IN_lim**4)
a5 = 3*(2*OUT_max - 2*b1*IN_max + b1*IN_lim)/(IN_lim**5)

def plot_fct(filterd, velocity, title):
  
  plt.subplot(2, 1, 1)
  plt.plot(filterd)
  plt.title(title)
  plt.xlabel("numbers of acquisitions")
  plt.ylabel("Force [N]")
  plt.grid()
  plt.ylim(ymax = 1/2 + IN_max, ymin = -(1/2 + IN_max))

  plt.subplot(2, 1, 2)
  plt.plot(velocity)
  plt.xlabel("numbers of acquisitions")
  plt.ylabel("velocity [m/s]")
  plt.grid()
  plt.ylim(ymax = 0.5 + OUT_max, ymin = -(0.5 + OUT_max))

  plt.draw()
  plt.pause(0.0001)
  plt.clf()

def length_control(variable,span=100):

  if len(variable) > span:
    variable = variable[-span:]
  
  return variable

def variable_control(variable, offset=0, span=100):

  variable = length_control(variable,span)

  if len(variable) < span:
   offset = sum(variable)/len(variable)

  dct = fftpack.dct(variable, norm="ortho")
  dct[8:] = 0
  filtred = fftpack.idct(dct, norm="ortho")

  for i in range(len(filtred)):
    filtred[i] = filtred[i]- offset 

  return filtred, variable, offset

def force_to_velocity(IN):

  velocity = 0

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
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  vel_msg = Twist()
  rospy.init_node("impedance_control")
  Fx, Fy, Fz, Tx, Ty, Tz = [], [], [], [], [], []
  Fx_ofs, Fy_ofs, Fz_ofs, Tx_ofs, Ty_ofs, Tz_ofs = 0, 0, 0, 0, 0, 0
  vx = []

  while not rospy.is_shutdown():
    data = rospy.wait_for_message("/my_gen3_lite/base_feedback",BaseCyclic_Feedback)

    Fx.append(data.base.tool_external_wrench_force_x)
    Fy.append(data.base.tool_external_wrench_force_y)
    Fz.append(data.base.tool_external_wrench_force_z)
    Tx.append(data.base.tool_external_wrench_torque_x)
    Ty.append(data.base.tool_external_wrench_torque_y)
    Tz.append(data.base.tool_external_wrench_torque_z)

    Fx_mean, Fx, Fx_ofs = variable_control(Fx, Fx_ofs, span=50)
    Fy_mean, Fy, Fy_ofs = variable_control(Fy, Fy_ofs, span=50)
    Fz_mean, Fz, Fz_ofs = variable_control(Fz, Fz_ofs, span=50)
    Tx_mean, Tx, Tx_ofs = variable_control(Tx, Tx_ofs, span=50)
    Ty_mean, Ty, Ty_ofs = variable_control(Ty, Ty_ofs, span=50)
    Tz_mean, Tz, Tz_ofs = variable_control(Tz, Tz_ofs, span=50)

    vx.append(force_to_velocity(Fx_mean[-1]))
    vx = length_control(vx, span = 50)

    vel_msg.linear.x = vx[-1]
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    cmd_vel.publish(vel_msg)

    plot_fct(Fx_mean, vx , title="$F_x$ mean value and $v_x$ output value")
    

if __name__ == "__main__":
  main()