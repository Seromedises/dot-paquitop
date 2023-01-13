#!/usr/bin/env python

import numpy as np
import scipy.fftpack as fftpack
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64

def filtering_fct(raw_signal):
  dct = fftpack.dct(raw_signal, norm="ortho")
  dct[8:] = 0
  return fftpack.idct(dct, norm="ortho")

def plot_fct_raw(raw,filterd,time):

  i = 0
  span = 40
  rawi = raw[0]

  while i <= len(raw):
      i = i + 1

      if i > span:
          rawi = raw[i-span:i]
          xi = time[i-span:i]
      else:
          rawi = raw[0:i]
          xi = time[0:i]

      plt.plot(xi, rawi, ".")
      plt.plot(xi, filterd)
      plt.draw()
      plt.pause(0.00001)
      if i != len(raw) + 1:
          plt.clf()
      else:
          plt.show()

def plot_fdt(filterd):

  
  plt.plot(range(len(filterd)-1), filterd)
  plt.draw()
  plt.pause(0.00001)
  plt.clf()
  """else:
      plt.show()"""


def lenght_ctrl(variable, span = 100):
  
  if len(variable) > span:
    variable = variable[-span]
  
  return variable


def main():
  rospy.init_node("impedance_control")
  Fx, Fy, Fz, Tx, Ty, Tz = []

  rospy.Subscriber("/my_gen3_lite/base_feedback/base/tool_external_wrench_force_x", Float64, Fx.append())
  rospy.Subscriber("/my_gen3_lite/base_feedback/base/tool_external_wrench_force_y", Float64, Fy.append())
  rospy.Subscriber("/my_gen3_lite/base_feedback/base/tool_external_wrench_force_z", Float64, Fz.append())
  rospy.Subscriber("/my_gen3_lite/base_feedback/base/tool_external_wrench_torque_x", Float64, Tx.append())
  rospy.Subscriber("/my_gen3_lite/base_feedback/base/tool_external_wrench_torque_y", Float64, Ty.append())
  rospy.Subscriber("/my_gen3_lite/base_feedback/base/tool_external_wrench_torque_z", Float64, Tz.append())
  
  while not rospy.is_shutdown():
    Fx = filtering_fct(lenght_ctrl(Fx))
    Fy = filtering_fct(lenght_ctrl(Fy))
    Fz = filtering_fct(lenght_ctrl(Fz))
    Tx = filtering_fct(lenght_ctrl(Tx))
    Ty = filtering_fct(lenght_ctrl(Ty))
    Tz = filtering_fct(lenght_ctrl(Tz))



if __name__ == "__main__":
  main()