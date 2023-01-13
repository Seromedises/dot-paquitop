#!/usr/bin/env python

import numpy as np
import scipy.fftpack as fftpack
import matplotlib.pyplot as plt
import rospy
from kortex_driver.msg import BaseCyclic_Feedback

def filtering_fct(raw_signal):
  
  """span = 150
  raw_signal = raw_signal[-span:]"""

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

def plot_fct(filterd,title):

  
  plt.plot(filterd)
  plt.title(title)
  plt.xlabel("numbers of acquisitions")
  plt.ylabel("Force [N]")
  plt.grid()
  plt.ylim(ymax = 3, ymin = -3)
  plt.draw()
  plt.pause(0.00001)
  plt.clf()
  
def lenght_ctrl(variable, span = 100):
  
  if len(variable) > span:
    variable = variable[-span:]
  
  return variable


def main():
  rospy.init_node("impedance_control")
  Fx, Fy, Fz, Tx, Ty, Tz = [], [], [], [], [], []
    
  while not rospy.is_shutdown():
    data = rospy.wait_for_message("/my_gen3_lite/base_feedback",BaseCyclic_Feedback)

    Fx.append(data.base.tool_external_wrench_force_x)
    Fy.append(data.base.tool_external_wrench_force_y)
    Fz.append(data.base.tool_external_wrench_force_z)
    Tx.append(data.base.tool_external_wrench_torque_x)
    Ty.append(data.base.tool_external_wrench_torque_y)
    Tz.append(data.base.tool_external_wrench_torque_z)

    Fx = lenght_ctrl(Fx,span=250)
    Fy = lenght_ctrl(Fx,span=250)
    Fz = lenght_ctrl(Fx,span=250)
    Tx = lenght_ctrl(Tx,span=250)
    Ty = lenght_ctrl(Ty,span=250)
    Tz = lenght_ctrl(Tz,span=250)

    Fx_mean = filtering_fct(Fx)
    Fy_mean = filtering_fct(Fy)
    Fz_mean = filtering_fct(Fz)
    Tx_mean = filtering_fct(Tx)
    Ty_mean = filtering_fct(Ty)
    Tz_mean = filtering_fct(Tz)

    # plot_fct(Fx_mean,title="$F_x$ mean value")
    

if __name__ == "__main__":
  main()