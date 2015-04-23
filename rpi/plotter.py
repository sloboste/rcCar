#!/usr/bin/python

import numpy
from matplotlib import pyplot as plt
import sys

# Take in log filename
if (sys.argv[1] == None):
    raise ValueError("log filename must not be blank")
filename = sys.argv[1]

# data [timestamp, steerPercent, throttlePercent]
data = numpy.genfromtxt(filename, skiprows=0, delimiter=',')
timestamp = data[:,0]
mode = data[:,1]
steerPercent = data[:,2]
throttlePercent = data[:,3]
if (len(timestamp) > 0):
    timeoffset = timestamp[0]
for i in range(0, len(timestamp)):
    timestamp[i] -= timeoffset

# Plot of steerPercent as fxn of timestamp
plt.title('Steering Percent as fxn of Time')
plt.xlabel('Time (ms)')
plt.ylabel('Steering Percent (negative==left)')
plt.plot(timestamp, steerPercent)
plt.xlim([timestamp[0], timestamp[len(timestamp)-1]])
plt.ylim([-100, 100])
plt.savefig('steer.png', dpi=100)
plt.clf()

# Plot of throttlePercent as fxn of timestamp
plt.title('Throttle Percent as fxn of Time')
plt.xlabel('Time (ms)')
plt.ylabel('Throttle Percent (negative==backward)')
plt.plot(timestamp, throttlePercent)
plt.xlim([timestamp[0], timestamp[len(timestamp)-1]])
plt.ylim([-100, 100])
plt.savefig('throttle.png', dpi=100)
plt.clf()

