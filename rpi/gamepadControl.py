#!/usr/bin/python

# Gamepad control of the car
#

import time
import sys
from controller import Controller
from car import Car

# Take in type of controller (gamepad or wheel)
if (len(sys.argv) != 3):
    print("Error: usage ./gamepadExample <controllerType> <logfile>")
    sys.exit(1)

# Controller instance
ctrl = Controller(sys.argv[1])

# Car instance
car = Car("/dev/ttyUSB0")
car.testSteer()
#car.testThrottle()
car.setMode("rpi")
car.beginLog(sys.argv[2], 0.25)

# Loop to read data and send to car
try:
    while (True):
        #print("loop")
        # send controller data to car
        data = ctrl.getData()
        #print("data = %s" %str(data))
        car.setSteer(data[0])
        car.setThrottle(data[1])
        time.sleep(0.15)
except KeyboardInterrupt:
    #car.endLog()
    pass

