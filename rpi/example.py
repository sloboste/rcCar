#!/usr/bin/python

import time

# Import the rc car module
from car import Car

# Instantiate the car object
car = Car()

# Test the system
car.testSteer()
car.testMotor()
print("Done testing")

# Drive in a figure-8
car.setMode("rpi")
car.setMotor("forward", 15)
while True:
    # FIXME this might not work......
    car.setSteer("right", 30)
    sleep(5)
    car.setMotor("left", 30)
    sleep(5)

