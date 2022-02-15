from utils import *
import time
import math
import time

initEncoders()
initMotors()
initPID()
#
# P=0.02
# I=0.01
# D=0

# pidL = PID.PID(P,I,D)
# pidR = PID.PID(P,I,D)
# pidL.setSampleTime(0.01)
# pidR.setSampleTime(0.01)
speedL=1
speedR=1
print("hi")
setSpeedsIPS(1,1)

second_timer=time.time()+1
while True:
    if time.time()>second_timer:
        second_timer=time.time()+1
        print(getSpeeds())
time.sleep(10000)
# current_speedL=speedL
# current_speedR=speedR
# pidL.SetPoint=speedL
# pidR.SetPoint=speedR
# while True:
#
#     x=getSpeeds()[0]
#     y=getSpeeds()[1]
#     pidL.update(x)
#     pidR.update(y)
#     print(x,y)
#     current_speedL=pidL.output+current_speedL
#     current_speedR=pidR.output+current_speedR
#     #print(speed,y,current_speedR,pidR.output)
#     setSpeedsIPS(-current_speedL,-current_speedR)
#

exit()
