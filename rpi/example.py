#!/usr/bin/python

from time import sleep

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
car.setMotor("forward", 5)
while True:
    # FIXME this might not work......
    car.setSteer("right", 50)
    sleep(5)
    car.setSteer("left", 50)
    sleep(5)

