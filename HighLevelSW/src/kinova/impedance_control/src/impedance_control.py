#!/usr/bin/env python

import numpy as np
import scipy.fftpack as fftpack
import matplotlib.pyplot as plt

def filtering_fct(raw_signal):
  dct = fftpack.dct(raw_signal, norm="ortho")
  dct[8:] = 0
  return fftpack.idct(dct, norm="ortho")

def plot_fct(raw,filterd,time):

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

