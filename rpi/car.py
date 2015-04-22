# car.py
# module for usb serial communication with the Arduino on the rc car
#
# Microcomputer-Controlled Car Project
# University of Michigan - Tilbury Research Group
# Version 1.0
#

import threading
import csv
import serial
from time import sleep
from mutex import Mutex

# Represents the rc car
class Car:
    # Construct the Car object
    #
    # doLogging - True --> log data to csvLogFilename at 1Hz, False --> no log
    #
    # csvLogFilename - full path to the csv file to which to write the log
    #
    def __init__(self, doLogging, csvLogFilename):
        # USB serial port on the RPI that is connected to the Arduino
        # Note: may need to be adjusted for the usb port on the RPi
        self.serialport = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)
        # Local data
        self.dataLock = threading.Lock()
        self.localDataValid = False
        self.mode = "idle"
        self.localSteerPerc = 0
        self.localThrottlePerc = 0
        # Log parameters
        self.doLog = doLogging
        self.logName = csvLogFilename
        self.lobBufMax = 30
        # FIXME

    
    # FIXME 
    #
    def logData(self):
        filehandle = None
        csvwriter = None
        # Write to buffer to log if full
        if (len(self.logBuf) == self.logBufMax): 
            filehandle = open('templog.csv', 'a')
            csvwriter = csv.writer(self.filehandle)
        # Write new data to log
        if (self.mode == "idle"):
            m = 0
        elif (self.mode == "rc"):
            m = 1
        elif (self.mode == "rpi"):
            m = 2
        tup = self.getData()
        row = [m, tup[0], tup[1]]    
        self.logBuf.append(row)

    # FIXME
    def close(self):
        self.filehandle.close()

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
            self.mode = mode
        elif (mode == "rc"):
            self.serialport.write('A R\0')
            self.mode = mode
        elif (mode == "rpi"):
            self.serialport.write('A P\0')
            self.mode = mode
        else:
            raise ValueError("bad mode\n")

    # Set the steering servo angle on the car.
    #
    # percent - an integer between -100 and 100 inclusive that indicates the
    #           percentage of the maximum turn. 
    #           [-100, 100] --> [full left, full right]
    #
    def setSteer(self, percent):
        if (self.mode != "rpi"):
            raise ValueError("bad mode\n")
        percent = int(percent)
        if ( (percent < -100) or (percent > 100) ): 
            raise ValueError("bad percent\n")
        self.serialport.write('B '+str(percent)+'\0') 
        return True

    # Set the throttle on the car.
    #
    # percent - an integer between -100 and 100 inclusive that indicates the
    #           percentage of the maximum throttle. 
    #           [-100, 100] --> [full reverse, full forward]
    #
    def setThrottle(self, percent):
        if (self.mode != "rpi"):
            raise ValueError("bad mode\n")
        percent = int(percent)
        if ( (percent < -100) or (percent > 100) ): 
            raise ValueError("bad percent\n")
        self.serialport.write('C '+str(percent)+'\0') 
        return True

    # Get the current steering servo direction and percent.
    # If the car is in idle mode, use local copy of data.
    # If the car is in rc mode, poll the Arduino for data.
    # If the car is in rpi mode, use local copy of data.
    #
    # Returns a tuple containing (steerPercent, throttlePercent)
    #
    def getData(self):
        if ( (self.mode == "rc") or (not self.localDataValid) ):
            self.serialport.write('D\0')
            resp = self.serialport.readline()
            nullchar = self.serialport.read()
            tokens = resp.split()
            s = int(tokens[1])
            t = int(tokens[2])
        else:
            s = self.localSteerPerc
            t = self.localThrottlePerc
        return (s, t)

    # Run a test of the steering mechanism by scanning from 
    # center-->right-->left-->center, one time
    #
    def testSteer(self):
        self.setMode("rpi")
        for x in range(0, 100):
            self.setSteer(x) 
            sleep(0.01)
        for x in reversed(range(-100, 100)):
            self.setSteer(x) 
            sleep(0.01)
        for x in range(-100, 0):
            self.setSteer(x) 
            sleep(0.01)
        self.setMode("idle")
    
    # Run a test of the throttle by scanning from 
    # stop-->30% forward-->stop -->100% reverse-->stop
    # Note: the motor may cut out in reverse or not move in reverse...
    #
    def testMotor(self):
        self.setMode("rpi")
        for x in range(0, 30):
            self.setMotor(x) 
            sleep(0.1)
        for x in reversed(range(0, 30)):
            self.setMotor(x) 
            sleep(0.1)
        sleep(3)
        for x in reversed(range(-100, 0)):
            self.setMotor(x) 
            sleep(0.01)
        for x in range(-100, 0):
            self.setMotor(x) 
            sleep(0.01)
        self.setMode("idle")

