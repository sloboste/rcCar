import lcm
from exlcm import gamepad_t
import time

def my_handler(channel, data):
    msg = gamepad_t.decode(data)
    print("   timestamp   = %s" % str(msg.utime))
    print("   axis 1 = %s" % str(msg.axes[0]))
    print("   axis 2 = %s" % str(msg.axes[2]))
    print("   axis 5 = %s" % str(msg.axes[5]))
    print("   buttons = %s" % str(msg.buttons))
    print("")

lc = lcm.LCM()
subscription = lc.subscribe("GAMEPAD", my_handler)

try:
    while True:
        time.sleep(0.5)
        lc.handle()
except KeyboardInterrupt:
    pass
