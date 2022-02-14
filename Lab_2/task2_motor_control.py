from utils import *
import time


initEncoders()
pwm=initMotors()

turnRV(4,2)
time.sleep(5)
print("Speed: {}".format(getSpeeds()))
print("Counts: {}".format(getCounts()))
resetCounts()
setSpeedsPWM (1.5,1.495)
input("Press Enter to continue...\n")

turnRV(4,-2)
time.sleep(5)
print("Speed: {}".format(getSpeeds()))
print("Counts: {}".format(getCounts()))
resetCounts()
setSpeedsPWM (1.5,1.495)
input("Press Enter to continue...\n")



exit()
