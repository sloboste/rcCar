#! /usr/bin/python
# Test for the RPi <--I2C--> Arduino communication

import rcCar

car = RcCar("SC10")
verified = car.verifyID()
print 'car id verified = ' + verified

