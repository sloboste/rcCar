# python module for usb serial communication with the rc car

import serial

class RcCarSerial:
    def __init__(self):
        # USB serial port on the RPI that is connected to the Arduino
        #FIXME rpi port
        self.serialport = serial.Serial("/dev/ttyUSB0", 9600, timeout=0.5)
        # Modes that the Arduino can be in 
        self.MODES.IDLE = 'I'
        self.MODES.RC = 'R'
        self.MODES.RPI = 'P'
    def setMode(self, mode):
        m = '\0'
        if (mode == "idle"):
            m = 'I' 
        elif (mode == "rc"):
            m = 'R'
        elif (mode == "rpi"):
            m = 'P'
        else:
            raise ValueError("bad mode\n")
        self.serialport.write
