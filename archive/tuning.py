from utils import *
import time
import math

initEncoders()
pwm=initMotors()

# l=1.5
# r=1.468
# setSpeedsPWM(l,r)
setSpeedsIPS(-2,2)

start_time=time.time()
timer=time.time()+11
second_timer=time.time()+1
totalL=0
totalR=0
while time.time()<timer:
    ticksL=getCounts()[0]
    ticksR=getCounts()[1]
    if time.time()>second_timer:
        second_timer=time.time()+0.1
        time_elapsed=round(time.time()-start_time,3)
        dL=round(ticksL*0.255254,3)
        dR=round(ticksR*0.255254,3)
        totalL=round(totalL+dL,6)
        totalR=round(totalR+dR,6)

        s=getSpeeds()
        sL=s[0]*2*math.pi*1.3
        sR=s[1]*2*math.pi*1.3
        speed=(sL,sR)
        print("S:{} \tDL:{} \tDR:{} \tT:{} \ttDL:{} \ttDR:{}".format(speed,dL,dR,time_elapsed,totalL,totalR))
        resetCounts()
# time.sleep(5)
# input("Press Enter to continue...")
#
# setSpeedsPWM (1.7,1.7)
# time.sleep(5)
# setSpeedsPWM (1.5,1.495)
# input("Press Enter to continue...")
#
#
# setSpeedsRPS (0.4,0.4)
# time.sleep(5)
# setSpeedsPWM (1.5,1.495)
# input("Press Enter to continue...")
#
# setSpeedsIPS (6,6)
# time.sleep(5)
setSpeedsPWM (1.5,1.495)
# input("Press Enter to continue...")

exit()
