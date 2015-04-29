#


import threading
from gamepad import gamepad_t
import lcm

# Controller
class Controller:
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

    # Handles gamepad / steering wheel data packet
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
    def __loopLCMreads(self):
        while (True):
            self.__lc.handle()

    # Returns steering and throttle data to the caller 
    def getData(self):
        self.__dataLock.acquire()
        s = self.__s
        t = self.__t
        self.__dataLock.release()
        return (s, t)

