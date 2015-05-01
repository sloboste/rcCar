# car.py
# Module for usb serial communication with the Arduino on the rc car
# Microcomputer-Controlled Car Project
# University of Michigan - Tilbury Research Group
# Version 1.0
#

import threading
import csv
import serial
import time

# Represents the rc car
class Car:
    # Construct the Car object
    #
    # usbPortName - the full path to the file that acts as the usb port. For
    #               example, "/dev/ttyUSB0"
    #
    def __init__(self, usbPortName):
        # USB serial port on the RPI that is connected to the Arduino
        self.__usbPort = usbPortName
        self.__serialport = serial.Serial(usbPortName, 9600, timeout=0.5)
        # Must delay for 3 seconds because Arduino resets on serial connection   
        time.sleep(3)
        # Local data
        self.__dataLock = threading.Lock()
        self.__localSteerValid = False
        self.__localThrottleValid = False
        self.__mode = "idle"
        self.__localSteerPerc = 0
        self.__localThrottlePerc = 0
        # Log parameters
        self.__logCond = threading.Condition()
        self.__doLog = False
        self.__logName = ""
        self.__approxPeriod = 0
        self.__filehandle = None 
        self.__csvwriter = None
        # Put the car into idle mode at start
        self.setMode("idle")

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
            self.__dataLock.acquire()
            self.__mode = mode
            self.__localSteerPerc = 0
            self.__localThrottlePerc = 0
            self.__localSteerValid = True
            self.__localThrottleValid = True
            self.__dataLock.release()
        elif (mode == "rc"):
            self.__serialport.write('A R\0')
            self.__dataLock.acquire()
            self.__mode = mode
            self.__localSteerValid = False
            self.__localThrottleValid = False
            self.__dataLock.release()
        elif (mode == "rpi"):
            self.__serialport.write('A P\0')
            self.__dataLock.acquire()
            self.__mode = mode
            self.__localSteerValid = False
            self.__localThrottleValid = False
            self.__dataLock.release()
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
        self.__dataLock.acquire()
        self.__localSteerPerc = int(percent)
        self.__localSteerValid = True
        self.__dataLock.release()

    # Set the throttle on the car.
    #
    # percent - an integer between -100 and 100 inclusive that indicates the
    #           percentage of the maximum throttle. 
    #           [-100, 100] --> [full reverse, full forward]
    #
    def setThrottle(self, percent):
        if (self.__mode != "rpi"):
            raise ValueError("bad mode\n")
        percent = int(percent)
        if ( (percent < -100) or (percent > 100) ): 
            raise ValueError("bad percent\n")
        self.__serialport.write('C '+str(percent)+'\0') 
        self.__dataLock.acquire()
        self.__localThrottlePerc = int(percent)
        self.__localThrottleValid = True
        self.__dataLock.release()

    # Get the current steering servo direction and percent.
    # If the car is in idle mode, use local copy of data.
    # If the car is in rc mode, poll the Arduino for data.
    # If the car is in rpi mode, use local copy of data.
    #
    # Returns a tuple containing (steerPercent, throttlePercent)
    #
    def getData(self):
        self.__dataLock.acquire()
        if ( (self.__mode == "rc")  or \
             (not(self.__localSteerValid and self.__localThrottleValid)) ):
            self.__dataLock.release()
            self.__serialport.write('D\0')
            resp = self.__serialport.readline()
            nullchar = self.__serialport.read()
            tokens = resp.split()
            s = int(tokens[1])
            t = int(tokens[2])
            #self.__dataLock.acquire()
            #self.__localSteerPerc = s
            #self.__localThrottlePerc = t
            #self.__dataLock.release()
        else:
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
            time.sleep(0.01)
        for x in reversed(range(-100, 100)):
            self.setSteer(x) 
            time.sleep(0.01)
        for x in range(-100, 0):
            self.setSteer(x) 
            time.sleep(0.01)
        self.setMode("idle")
    
    # Run a test of the throttle by scanning from 
    # stop-->30% forward-->stop -->100% reverse-->stop
    # Note: the motor may cut out in reverse or not move in reverse...
    #
    def testThrottle(self):
        self.setMode("rpi")
        for x in range(0, 30):
            self.setThrottle(x) 
            time.sleep(0.1)
        for x in reversed(range(0, 30)):
            self.setThrottle(x) 
            time.sleep(0.1)
        time.sleep(3)
        for x in reversed(range(-100, 0)):
            self.setThrottle(x) 
            time.sleep(0.01)
        for x in range(-100, 0):
            self.setThrottle(x) 
            time.sleep(0.01)
        self.setMode("idle")

    # Begin logging data in a csv file. The logging code runs in another thread
    #
    # logFilename - the full path the csv file in which to log data. May or
    #               may not already exist
    #
    # approxPeriod - The approximate period (in seconds) to take measurements.
    #                This is an approximation because python thread creation 
    #                takes time and we aren't using an RTOS.
    #                Note: if this is too small, i.e., << 1s,
    #
    def beginLog(self, logFilename, approxPeriod):
        # FIXME vaildate args
        self.__logCond.acquire()
        self.__logName = logFilename
        self.__doLog = True
        self.__approxPeriod = approxPeriod
        self.__filehandle = open(logFilename, 'a') 
        self.__csvwriter = csv.writer(self.__filehandle) 
        # Start periodic logging
        thrd = threading.Timer(self.__approxPeriod, self.__logData)
        self.__curLogThread = thrd 
        thrd.daemon = True
        thrd.start()
        self.__logCond.release()
        # FIXME need some delay to allow the logging thread to start
        time.sleep(0.5)

    # Write the timestamp (in miliseconds), current mode, steering percent, and
    # motor throttle to the log 
    #
    def __logData(self):
        # Write new data to log
        self.__dataLock.acquire()
        if (self.__mode == "idle"):
            m = 0
        elif (self.__mode == "rc"):
            m = 1
        elif (self.__mode == "rpi"):
            m = 2
        self.__dataLock.release()
        tup = self.getData()
        tms = time.time() * 1e3 
        row = [tms, m, tup[0], tup[1]]    
        self.__logCond.acquire()
        self.__csvwriter.writerow(row)
        # Re-call the method with another thread if logging not ended
        while (not self.__doLog):
            self.__filehandle.close()
            self.__logCond.wait()
        self.__filehandle = open(self.__logName, 'a') 
        self.__csvwriter = csv.writer(self.__filehandle) 
        thrd = threading.Timer(self.__approxPeriod, self.__logData)
        self.__curLogThread = thrd 
        thrd.daemon = True
        thrd.start()
        self.__logCond.release()

    # Terminate logging
    #
    def endLog(self):
        self.__curLogThread.join() 
        self.__logCond.acquire()
        self.__doLogging = False
        self.__logCond.release()

