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
        self.REG_ID   = 0x00 
        self.REG_MODE = 0x01
        # Commands to send to Arduino FIXME
        self.ENTER_IDLE_MODE   = 0x00
        self.ENTER_RC_MODE     = 0x01
        self.ENTER_RPI_MODE    = 0x02
        self.ENTER_SLEEP_MODE  = 0x03
        self.RETURN_STEER_DC   = 0x04
        self.RETURN_MOTOR_DC   = 0x05
        self.SET_STEER_DC      = 0x06
        self.SET_MOTOR_DC      = 0x07
    

    # Public: Verify the id number of the Arduino module
    #             "idle"  - do nothing
    #             "rc"    - respond to commands from the radio transmitter
    #             "rpi"   - respond to commands from the RPi
    #             "sleep" - low power mode; can't do anything until woken up
    #
    def verifyID():
        # Read id number from Arduino
        number = bus.read_byte_data(self.ADDR, self.REG_ID) 
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
    def setMode(mode):
        # Validate mode
        cmd = 0xFF
        if (mode == "idle"):
            cmd = self.ENTER_IDLE_MODE
        else if (mode == "rc"):
            cmd = self.ENTER_RC_MODE
        else if (mode == "rpi"):
            cmd = self.ENTER_RPI_MODE
        else if (mode == "sleep"):
            cmd = self.ENTER_SLEEP_MODE
        else:
            raise ValueError('mode not recognized')
        # Write to Arduino
        bus.write_byte_data(self.ADDR, self.REG_MODE, cmd)


    # Public: Set the motor speed of the RC car as a percentage of the maximum
    #
    # Note: The Arduino will not respond unless it is in RPi mode
    #
    # percentage - range 0 to 100 percent
    #
    def setSpeed(percentage):
        # Validate percentage
        if ( (percentage < 0) or (percentage > 100) ):
            raise ValueError('percentage must be between 0 and 100')
        # Write to Arduino
        bus.write_byte_data(self.ADDR, self.REG_SPEED, percentage)


    # Public: Set the direction for the RC car to turn as a percentage of the
    #         maximum
    #
    # Note: The Arduino will not respond unless it is in RPi mode
    #
    # direction - turn "left" or "right"
    #
    # percentage - range 0 to 100 percent
    #
    def setSteer(direction, percentage):
        # Validate direction
        if ( (direction != "right") and (direction != "left") ):
            raise ValueError('directon must be either right or left')
        # Validate percentage
        if ( (percentage < 0) or (percentage > 100) ):
            raise ValueError('percentage must be between 0 and 100')
        # Write to Arduino
        bus.write_byte_data(self.ADDR, self.REG_STEER, percentage)

