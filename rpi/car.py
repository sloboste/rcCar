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
    def __init__(self):
        # USB serial port on the RPI that is connected to the Arduino
        # Note: may need to be adjusted for the usb port on the RPi
        self.__serialport = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)
        # Local data
        self.__dataLock = threading.Lock()
        self.__localSteerValid = False
        self.__localThrottleValid = False
        self.__mode = "idle"
        self.__localSteerPerc = 0
        self.__localThrottlePerc = 0
        # Log parameters
        self.__logCond = threading.Condition()
        self.__doLog = doLogging
        self.__logName = ""
        self.__approxPeriod = 0
        self.__filehandle = None 
        self.__csvwriter = None
        # Put the car into idle mode at start
        self.setMode(self, "idle")

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
            self.__serialport.write('A I\0')
            self.__mode = mode
            self.__localSteerPerc = 0
            self.__localThrottlePerc = 0
            self.__localSteerValid = True
            self.__localThrottleValid = True
        elif (mode == "rc"):
            self.__serialport.write('A R\0')
            self.__mode = mode
            self.__localSteerValid = False
            self.__localThrottleValid = False
        elif (mode == "rpi"):
            self.__serialport.write('A P\0')
            self.__mode = mode
            self.__localSteerValid = False
            self.__localThrottleValid = False
        else:
            raise ValueError("bad mode\n")

    # Set the steering servo angle on the car.
    #
    # percent - an integer between -100 and 100 inclusive that indicates the
    #           percentage of the maximum turn. 
    #           [-100, 100] --> [full left, full right]
    #
    def setSteer(self, percent):
        if (self.__mode != "rpi"):
            raise ValueError("bad mode\n")
        percent = int(percent)
        if ( (percent < -100) or (percent > 100) ): 
            raise ValueError("bad percent\n")
        self.__serialport.write('B '+str(percent)+'\0') 
        self.__localSteerValid = True
        return True

    # Set the throttle on the car.
    #
    # percent - an integer between -100 and 100 inclusive that indicates the
    #           percentage of the maximum throttle. 
    #           [-100, 100] --> [full reverse, full forward]
    #
    def setThrottle(self, percent):
        if (self._mode != "rpi"):
            raise ValueError("bad mode\n")
        percent = int(percent)
        if ( (percent < -100) or (percent > 100) ): 
            raise ValueError("bad percent\n")
        self.__serialport.write('C '+str(percent)+'\0') 
        self.__localThrottleValid = True
        return True

    # Get the current steering servo direction and percent.
    # If the car is in idle mode, use local copy of data.
    # If the car is in rc mode, poll the Arduino for data.
    # If the car is in rpi mode, use local copy of data.
    #
    # Returns a tuple containing (steerPercent, throttlePercent)
    #
    def getData(self):
        if ( (self.__mode == "rc")  or \
             (not(self.__localSteerValid and self.__localThrottleValid)) ):
            self.__serialport.write('D\0')
            resp = self.__serialport.readline()
            nullchar = self.__serialport.read()
            tokens = resp.split()
            s = int(tokens[1])
            t = int(tokens[2])
            self.__dataLock.acquire()
            self.__localSteerPerc = s
            self.__localThrottlePerc = t
            self.__dataLock.release()
        else:
            self.__dataLock.acquire()
            s = self.__localSteerPerc
            t = self.__localThrottlePerc
            self.__dataLock.release()
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

    # FIXME
    # Begin logging data in a csv file. The logging code runs in another thread
    #
    # logFilename - the full path the csv file in which to log data. May or
    #               may not already exist
    #
    # approxPeriod - The approximate period to take measurements. This is an
    #                approximation because python thread creation takes time
    #                and we aren't using an RTOS.
    #                Noe: if this is too small, i.e., << 1s,
    #
    def beginLog(self, logFilename, approxPeriod):
        # FIXME vaildate args
        self.__logCond.acquire()
        self.__doLog = True
        self.__logCond.release()
        self.__approxPeriod = approxPeriod
        self.__filehandle = open(logFilename, 'a') 
        self.__csvwriter = csv.writer(self.filehandle) 
        # Start periodic logging
        self.__logData()


    # FIXME 
    # Write the timestamp (in miliseconds limited to a 1 day timespan), 
    # current mode, steering percent, and motor throttle to the log 
    #
    def __logData(self):
        # Write new data to log
        if (self.__mode == "idle"):
            m = 0
        elif (self.__mode == "rc"):
            m = 1
        elif (self.__mode == "rpi"):
            m = 2
        tup = self.getData()
        t = time.datetime.now()
        tms = t.microseconds/1e3 + t.seconds*1e3 + t.minutes*60*1e3
        tms += t.hours*3600*1e3
        row = [tms, m, tup[0], tup[1]]    
        self.__csvwriter.writerow(row)
        # Re-call the method with another thread if logging not ended
        self.__logCond.acquire()
        while (not self.__doLog):
            self.__logCond.wait()
        self.__logCond.release()
        threading.Timer(self.__approxPeriod, self.__logData)

    # FIXME
    # Terminate logging and close the logfile
    #
    def endLog(self):
        self.__filehandle.close()
        self.__logCond.acquire()
        self.__doLogging = False
        self.__logCond.release()

