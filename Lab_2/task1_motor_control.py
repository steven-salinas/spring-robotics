from utils import *
import time

initEncoders()
pwm=initMotors()


moveXV(5,1)
input("Press Enter to continue...")

setSpeedsPWM (1.7,1.7)
time.sleep(5)
setSpeedsPWM (1.5,1.495)
input("Press Enter to continue...")


setSpeedsRPS (0.4,0.4)
time.sleep(5)
setSpeedsPWM (1.5,1.495)
input("Press Enter to continue...")

setSpeedsIPS (6,6)
time.sleep(5)
setSpeedsPWM (1.5,1.495)
input("Press Enter to continue...")
exit()
