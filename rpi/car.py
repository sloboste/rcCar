# python module for usb serial communication with the rc car

import serial
from time import sleep

class Car:
    def __init__(self):
        # USB serial port on the RPI that is connected to the Arduino
        #FIXME rpi port
        self.serialport = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)

    def setMode(self, mode):
        if (mode == "idle"):
            self.serialport.write('A I\0')
        elif (mode == "rc"):
            self.serialport.write('A R\0')
        elif (mode == "rpi"):
            self.serialport.write('A P\0')
        else:
            raise ValueError("bad mode\n")

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
            self.setMotor("backward", x*0.3) 
            sleep(0.01)
        for x in reversed(range(0, 100)):
            self.setMotor("backward", x*0.3) 
            sleep(0.01)
        self.setMode("idle")

