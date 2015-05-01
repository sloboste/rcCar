# controller.py
# Module for gamepad and wheel videogame controller
# Microcomputer-Controlled Car Project
# University of Michigan - Tilbury Research Group
# Version 1.0
#

import threading
from gamepad import gamepad_t
import lcm

# Represents a gamepad controller or a steering wheel controller
#
class Controller:
    # Instantiate the controller
    #
    def __init__(self, cType):
        self.__dataLock = threading.Lock()
        self.__type = cType
        self.__s = 0
        self.__t = 0
        # LCM instance
        self.__lc = lcm.LCM()
        self.__sub = None
        if (self.__type == "gamepad" or self.__type == "wheel"):
            self.__sub = self.__lc.subscribe("GAMEPAD", self.__dataHandler)
        else:
            raise ValueError("controller type not supported")
        # Spawn handler thread
        thrd = threading.Thread(target=self.__loopLCMreads)
        thrd.daemon = True
        thrd.start()

    # Handles gamepad / steering wheel data packet by converting the axis data
    # to steering and throttle percentage
    #
    def __dataHandler(self, channel, data):
        msg = gamepad_t.decode(data)
        #print ("axis 0 = %s" %str(msg.axes[0]))
        #print ("axis 1 = %s" %str(msg.axes[1]))
        #print ("axis 2 = %s" %str(msg.axes[2]))
        #print ("axis 5 = %s" %str(msg.axes[5]))
        self.__dataLock.acquire()
        self.__s = 100 * msg.axes[0] # steering 
        if (self.__type == "gamepad"):
            self.__t = 50*((msg.axes[5]+1) - (msg.axes[2]+1))
        elif (self.__type == "wheel"):
            self.t = msg.axes[0]
        #print ("s = %s" %str(self.__s))
        #print ("t = %s" %str(self.__t))
        self.__dataLock.release()

    # Continuously reads lcm messages
    #
    def __loopLCMreads(self):
        while (True):
            self.__lc.handle()

    # Get the steering and throttle percentage that the state of the controller
    # is currently at
    #
    # Returns at tuple containing the steer percentage then throttle percentage 
    #
    def getData(self):
        self.__dataLock.acquire()
        s = self.__s
        t = self.__t
        self.__dataLock.release()
        return (s, t)

