# rc-car.py
# Microcomputer-Controlled Car Project
# University of Michigan - Tilbury Research Group 
# Author: Steven Sloboda
# Version: 0.1
#

import smbus

# RcCar: 
# Module for control of an RC car with a RPi. The car has an Arduino Pro Mini 
# on board to handle PWM which communicates with the RPi via I2C.
#
class RcCar:
    # Public: Create and initialize an ArduinoI2C object
    # 
    # rcCarType - A string that indicates the type of RC car that the Arduino
    #             is mounted on. The only supported type right now is "SC10".
    #
    def __init__(self, carType):
        # Check type and assign appropriate values
        if (carType == "SC10"):
            self.ADDR = 0x10          # Arduino 7-bit I2C address
            self.ID = 0xAD            # Arduino 8-bit id number (arbitrary)
        else:
            raise ValueError('rcCarType is not supported')
        # Setup RPi I2C bus
        self.bus = smbus.SMBus(1) # /dev/i2c-1 
        # "Registers" in Arduino
        self.REG_ID        = 0x00 
        self.REG_MODE      = 0x01
        self.REG_STEER     = 0x02
        self.REG_SPEED     = 0x03
        self.REG_NEXT_READ = 0x04
        # Values to write to Arduino mode "register"
        self.MODE_IDLE   = 0x00
        self.MODE_RC     = 0x01
        self.MODE_RPI    = 0x02
        self.MODE_SLEEP  = 0x03    

    # Public: Verify the id number of the Arduino module
    #             "idle"  - do nothing
    #             "rc"    - respond to commands from the radio transmitter
    #             "rpi"   - respond to commands from the RPi
    #             "sleep" - low power mode; can't do anything until woken up
    #
    def verifyID(self):
        # Tell Arduino to send id on next read
        self.bus.write_byte_data(self.ADDR, self.REG_NEXT_READ, self.REG_ID)
        # Read id number from Arduino
        # number = self.bus.read_byte_data(self.ADDR, self.REG_ID) 
        number = self.bus.read_byte(self.ADD) # Arduino doesn't accpet read regs 
        # Check if response was correct
        if (number == self.ID):
            return True
        else:
            return False


    # Public: Put the Arduino into one of the following modes:
    #             "idle"  - do nothing
    #             "rc"    - respond to commands from the radio transmitter
    #             "rpi"   - respond to commands from the RPi
    #             "sleep" - low power mode; can't do anything until woken up
    #
    def setMode(self, mode):
        # Validate mode
        m = 0xFF
        if (mode == "idle"):
            cmd = self.MODE_IDLE
        elif (mode == "rc"):
            cmd = self.MODE_RC
        elif (mode == "rpi"):
            cmd = self.MODE_RPI
        elif (mode == "sleep"):
            cmd = self.MODE_SLEEP
        else:
            raise ValueError('mode not recognized')
        # Write to Arduino
        self.bus.write_byte_data(self.ADDR, self.REG_MODE, m)


    # Public: Set the motor speed of the RC car as a percentage of the maximum
    #
    # Note: The Arduino will not respond unless it is in RPi mode
    #
    # direction - go "forward" = 0 or "reverse" = 1
    #
    # percentage - range 0 to 100 percent
    #
    def setSpeed(self, direction, percentage):
        # Validate direction
        if ( (direction == 0) and (direction != 1) ):
            raise ValueError('directon must be either 0 = forward or 1 = reverse')
        # Validate percentage
        if ( (percentage < 0) or (percentage > 100) ):
            raise ValueError('percentage must be between 0 and 100')
        # Write to Arduino
        self.bus.write_byte_data(self.ADDR, self.REG_SPEED, direction, percentage)


    # Public: Set the direction for the RC car to turn as a percentage of the
    #         maximum
    #
    # Note: The Arduino will not respond unless it is in RPi mode
    #
    # direction - turn "left" = 0 or "right" = 1
    #
    # percentage - range 0 to 100 percent
    #
    def setSteer(self, direction, percentage):
        # Validate direction
        if ( (direction == 0) and (direction != 1) ):
            raise ValueError('directon must be either 0 = left or 1 = right')
        # Validate percentage
        if ( (percentage < 0) or (percentage > 100) ):
            raise ValueError('percentage must be between 0 and 100')
        # Write to Arduino
        self.bus.write_byte_data(self.ADDR, self.REG_STEER, direction, percentage)

