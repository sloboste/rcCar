#!/usr/bin/python

import numpy
from matplotlib import pyplot as plt

# data [timestamp, steerPercent, throttlePercent]
data = numpy.genfromtxt('templog.csv', skiprows=0, delimiter=',')
timestamp = data[:,0]
steerPercent = data[:,1]
throttlePercent = data[:,2]

# Plot of steerPercent as fxn of timestamp
plt.title('Steering Percent as fxn of Timestamp')
plt.xlabel('Timestamp (ms)')
plt.ylabel('Steering Percent (negative==left)')
plt.plot(timestamp, steerPercent)
plt.savefig('steer.png', dpi=100)
plt.clf()

# Plot of throttlePercent as fxn of timestamp
plt.title('Throttle Percent as fxn of Timestamp')
plt.xlabel('Timestamp (ms)')
plt.ylabel('Throttle Percent (negative==backward)')
plt.plot(timestamp, throttlePercent)
plt.savefig('throttle.png', dpi=100)
plt.clf()

