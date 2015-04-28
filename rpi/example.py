#!/usr/bin/python

from time import sleep

# Import the rc car module
from car import Car

# Instantiate the car object
car = Car("/dev/ttyUSB0")

# Begin logging data
car.beginLog('examplelog.csv', 0.5)

# Test the system
car.testSteer()
car.testThrottle()
print("Done testing")

# Begin logging data
car.endLog()

# Drive in a figure-8
#car.setMode("rpi")
#car.setMotor("forward", 5)
#while True:
    # FIXME this might not work......
    #car.setSteer(50)
    #sleep(5)
    #car.setSteer(-50)
    #sleep(5)

