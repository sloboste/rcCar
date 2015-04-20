# car.py
# module for usb serial communication with the Arduino on the rc car
#
# Microcomputer-Controlled Car Project
# University of Michigan - Tilbury Research Group
# Version 1.0
#

import serial
from time import sleep

# Represents the rc car
class Car:
    def __init__(self):
        # USB serial port on the RPI that is connected to the Arduino
        #FIXME This will need to be adjusted for the usb port on the RPi
        self.serialport = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)

    # Set the control mode of the car on the Arduino
    #
    # mode - one of the following strings:
    #   "idle" => Car remains motionless, waiting for another setMode command
    #   "rc"   => Puts car into radio control mode. Steering and throttle are
    #             set by the radio transmitter.
    #   "rpi"  => Puts car into Raspberry Pi control mode. Steering and throttle
    #             are set by serial commands sent to the Arduino.
    #
    # The Arduino will respond to a setMode comand sent to it at any time. 
    #
    def setMode(self, mode):
        if (mode == "idle"):
            self.serialport.write('A I\0')
        elif (mode == "rc"):
            self.serialport.write('A R\0')
        elif (mode == "rpi"):
            self.serialport.write('A P\0')
        else:
            raise ValueError("bad mode\n")
    # Set the steering servo on the car to a specified direction and percentage
    # of the maximum turn in that direction.
    #
    # direction - one of the following strings: "left" or "right"
    #
    # percent - an integer between 0 and 100 inclusive that indicates the
    #           percentage of the maximum turn in the above direction
    #
    def setSteer(self, direction, percent):
        pValid = True
        percent = int(percent)
        if ( (percent < 0) or (percent > 100) ): 
            pValid = False
        if (pValid):
            if (direction == "right"):
                self.serialport.write('B R '+str(percent)+'\0') 
            elif (direction == "left"):
                self.serialport.write('B L '+str(percent)+'\0') 
            else:
                raise ValueError("bad direction\n")
        else:
            raise ValueError("bad percent\n")

    # Set the motor controller on the car to a specified direction and
    # percentage of the maximum throttle in that direction.
    #
    # direction - one of the following strings: "forward" or "backward"
    #
    # percent - an integer between 0 and 100 inclusive that indicates the
    #           percentage of the maximum throttle in the above direction
    #
    def setMotor(self, direction, percent):
        pValid = True
        percent = int(percent)
        if ( (percent < 0) or (percent > 100) ): 
            pValid = False
        if (pValid):
            if (direction == "forward"):
                self.serialport.write('C F '+str(percent)+'\0') 
            elif (direction == "backward"):
                self.serialport.write('C B '+str(percent)+'\0') 
            else:
                raise ValueError("bad direction\n")
        else:
            raise ValueError("bad percent\n")

    # Run a test of the steering mechanism by scanning from 
    # center-->left-->right-->center one time
    #
    def testSteer(self):
        self.setMode("rpi")
        for x in range(0, 100):
            self.setSteer("left", x) 
            sleep(0.01)
        for x in reversed(range(0, 100)):
            self.setSteer("left", x) 
            sleep(0.01)
        for x in range(0, 100):
            self.setSteer("right", x) 
            sleep(0.01)
        for x in reversed(range(0, 100)):
            self.setSteer("right", x) 
            sleep(0.01)
        self.setMode("idle")
    
    # Run a test of the throttle by scanning from 
    # stop-->30% forward-->stop -->100% reverse-->stop
    # Note: the motor may cut out in reverse or not move in reverse...
    #
    def testMotor(self):
        self.setMode("rpi")
        for x in range(0, 30):
            self.setMotor("forward", x) 
            sleep(0.1)
        for x in reversed(range(0, 30)):
            self.setMotor("forward",x) 
            sleep(0.1)
        sleep(3)
        for x in range(0, 100):
            self.setMotor("backward", x) 
            sleep(0.01)
        for x in reversed(range(0, 100)):
            self.setMotor("backward", x) 
            sleep(0.01)
        self.setMode("idle")

