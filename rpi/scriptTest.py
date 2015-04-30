#!/usr/bin/python
from car import Car
car = Car("/dev/ttyUSB0")
car.testSteer()
print("done")

