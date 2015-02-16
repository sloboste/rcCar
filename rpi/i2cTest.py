#! /usr/bin/python

# Test for the RPi I2C
#
#

import smbus
import time

bus = smbus.SMBus(1) # /dev/i2c-1

ADDR = 0x77
REG_R = 0xD0
REG_W = 0x00 

# Read
value = bus.read_byte_data(ADDR, REG_R)
print 'read {data} from device {dev} register {reg}'.format(data=format(value, '2x'), dev=format(ADDR, '2x'), reg=format(REG_R, '2x'))

# Write
value = 0x00
# bus.write_byte_data(ADDR, REG_READ, value)


