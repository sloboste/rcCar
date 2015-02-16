#! /usr/bin/python

# Test for communicating between RPi and Arduino. RPi is the master.

import smbus

bus = smbus.SMBus(1) # /dev/i2c-1

while (True)
    print 'Begin communication loop'
