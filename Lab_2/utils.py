import time
import Adafruit_PCA9685
import signal
import math
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/
from itertools import cycle
from decimal import Decimal
from matplotlib import pyplot as plt
import numpy as np
import PID
import threading

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 1
RSERVO = 0

LENCODER = 17
RENCODER = 18



global pidL, pidR
def initPID(P,I,D):
    global pidL, pidR
    pidL = PID.PID(P,I,D)
    pidR = PID.PID(P,I,D)
    pidL.SetPoint=0
    pidR.SetPoint=0
    # return pidL, pidR

def initMotors():
    global pwm
    # Initialize the servo hat library.
    pwm = Adafruit_PCA9685.PCA9685()

    # 50Hz is used for the frequency of the servos.
    pwm.set_pwm_freq(50)
    setSpeedsPWM (1.5,1.495)
    return pwm

def remap(value, fromLow, fromHigh, toLow, toHigh):
    fromRange = (fromHigh - fromLow)
    toRange= (toHigh - toLow)
    newValue = (((value - fromLow) * toRange) / fromRange) + toLow
    newValue=round(newValue,3)
    return newValue

def clamp(value, low, high):
    return max(min(high, value), low)


def closest(lst, K):

     lst = np.asarray(lst)
     idx = (np.abs(lst - K)).argmin()
     return idx

def moveXV(X,V):
    travel_time=abs(X/V)
    print("Moving a distance of {} inches at {} linear inches per second for {} seconds".format(X,V,travel_time))

    num_ticks= X/0.255254

    second_timer=time.time()+1
    start_time=time.time()
    status=setSpeedsIPS(V, V)
    if status:
        # while time.time()<end_time:
        ticks_elapsed=(getCounts()[1]+getCounts()[0])/2
        while ticks_elapsed <num_ticks:

            if time.time()>second_timer:
                second_timer=time.time()+1
                time_elapsed=round(time.time()-start_time,3)
                distance_elapsed=round(ticks_elapsed*0.255254,3)
                print("{} seconds pased, moved {} inches".format(time_elapsed,distance_elapsed))
                x=getSpeeds()
                print(round(x[0]*2*math.pi*1.3,2))
                print(round(x[1]*2*math.pi*1.3,2))

            ticks_elapsed=(getCounts()[1]+getCounts()[0])/2

        time_elapsed=round(time.time()-start_time,3)
        print("Finished moving in {} seconds\n".format(time_elapsed))
        setSpeedsPWM (1.5,1.495)
    else:
        print("Requested velocity is not able to be set, movement cancelled\n")
        setSpeedsPWM (1.5,1.495)

wheel_diam=1.3 #Wheel diameter in inches

maxIPS=5
maxRPS=maxIPS/(2*math.pi*wheel_diam)


pwmMapL=[1.448,1.453,1.458,1.463,1.468,1.473,1.478,1.483,1.488,1.493,1.498,1.503,1.508,1.513,1.518,1.523,1.528,1.533,1.538,1.543,1.548,1.553,1.558,1.563]
ipsMapL=[-4.65,-4.2,-3.75,-3.3,-2.85,-2.4,-1.94,-1.583,-1.123,-0.638,-0.153,0,0,0.1786,0.63,1.123,1.601,2.075,2.45,2.9,3.35,3.8,4.25,4.7]

pwmMapR=[1.443,1.448,1.453,1.458,1.463,1.468,1.473,1.478,1.483,1.488,1.493,1.498,1.503,1.508,1.513,1.518,1.523,1.528,1.533,1.538,1.543,1.548,1.553,1.558,1.563]
ipsMapR=[-4.65,-4.2,-3.75,-3.3,-2.85,-2.4,-1.94,-1.48,-1.05,-0.613,-0.077,0,0,0.08,0.48,0.89,1.4,1.813,2.25,2.7,3.15,3.6,4.05,4.5,4.95]


def setSpeedsPWM (pwmLeft, pwmRight):
    global pwmLeftCurrent, pwmRightCurrent


    Lcal=0
    Rcal=0

    if abs(pwmLeft-1.5)>0.2 or abs(pwmRight-1.5)>0.2:
        print("Not able to set motor speed to PWM, left: {} right: {}".format(pwmLeft,pwmRight))
    # minPWM=1.3
    # maxPWM=1.7
    # pwmLeft=clamp(pwmLeft,minPWM,maxPWM)
    # pwmRight=clamp(pwmRight,minPWM,maxPWM)
        status=False
    else:
        pwmLeftCurrent=pwmLeft
        pwmRightCurrent=pwmRight
        #print("Setting motor speed in PWM, left: {} right: {}".format(pwmLeft,pwmRight))
        pwm.set_pwm(LSERVO, 0, math.floor(pwmLeft / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(pwmRight / 20 * 4096));
        status=True
    return status



def setSpeedsRPS (rpsLeft, rpsRight):
    global maxRPS
    global wheel_diam

    if abs(rpsLeft)>maxRPS or abs(rpsRight)>maxRPS:
        print("Not able to set motor speed to RPS, left: {} right: {}".format(rpsLeft,rpsRight))
        status=False
    else:
        ipsLeft=(2*math.pi*wheel_diam)*rpsLeft
        ipsRight=(2*math.pi*wheel_diam)*rpsRight
        setSpeedsIPS(ipsLeft,ipsRight)
    return status


def setSpeedsIPS(ipsLeft, ipsRight,setPID=True):
    global maxIPS

    global pwmMapL
    global ipsMapL

    global pwmMapR
    global ipsMapR

    global pidL
    global pidR

    # print(setPID,ipsLeft,ipsRight)
    # print(pidL.SetPoint)
    if setPID is True:
        pidL.SetPoint=ipsLeft
        pidR.SetPoint=ipsRight
        # print(pidL.SetPoint)

    if abs(ipsLeft)>maxIPS or abs(ipsRight)>maxIPS:
        print("Not able to set motor speed to IPS, left: {} right: {}".format(ipsLeft,ipsRight))
        status=False

    else:
        print("Setting motor speed in IPS, L: {} R: {}".format(ipsLeft,ipsRight))
        idxL=closest(ipsMapL, ipsLeft)
        remL= ipsLeft-ipsMapL[idxL]

        if int(remL) is not 0:
            interIdxL=int(remL/abs(remL))
            interL=abs(pwmMapL[idxL]-pwmMapL[idxL+interIdxL])*remL
        else:
            interL=0

        pwmL=pwmMapL[idxL]+interL
        pwmL=round(pwmL,4)

        #Right
        ipsRight=ipsRight*-1
        idxR=closest(ipsMapR, ipsRight)
        remR= ipsRight-ipsMapR[idxR]

        if int(remL) is not 0:
            interIdxR=int(remR/abs(remR))
            interR=abs(pwmMapR[idxR]-pwmMapR[idxR+interIdxR])*remR
        else:
            interR=0

        pwmR=pwmMapR[idxR]+interR
        pwmR=round(pwmR,4)


        setSpeedsPWM(pwmL,pwmR)
        status=False
    return status

def setSpeedsVW(V, W):
    global maxIPS
    print("Setting motor speed in VW, V: {} W: {}".format(V,W))

    if W == 0:
        Vl=V
        Vr=V
    else:
        R=abs(V/W) #inches

        dmid=2 #half of wheel axis distance in inches
        if W>0:
            Vl=abs(W)*(R-dmid)#V of outer wheel
            Vr=abs(W)*(R+dmid)#V of innner wheel
        else:
            Vl=abs(W)*(R+dmid)#V of outer wheel
            Vr=abs(W)*(R-dmid)#V of innner wheel

    if abs(Vl) <=maxIPS and abs(Vr) <=maxIPS:

        setSpeedsIPS(Vl,Vr)
    else:
        print("Motors are not capable of completing maneuver, outer {} inner {}".format(Vl,Vr))




left_count=0
right_count=0

window_size=5
left_window=[]
right_window=[]


left_start=time.time()
right_start=time.time()

def onLeftEncode(pin):
    global left_count
    global left_window
    global left_start
    global window_size

    left_count=left_count+1 #Increment tick count

    left_end=time.time()
    time_elapsed=round(left_end-left_start,4)
    left_window.append(time_elapsed) #append elapsed time between ticks to window


    if len(left_window) == window_size:

        left_window.pop(0) #removes oldest entry, required for moving average

    left_start=time.time()
    #print("Left encoder ticked!")

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global right_count
    global right_window
    global right_start
    global window_size

    right_count=right_count+1 #Increment tick count

    right_end=time.time()
    time_elapsed=right_end-right_start
    right_window.append(time_elapsed) #append elapsed time between ticks to window

    if len(right_window) == window_size:
        right_window.pop(0) #removes oldest entry, required for moving average

    right_start=time.time()
    #print("Right encoder ticked!")

def initEncoders():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
    GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

def getCounts():
    global left_count, right_count
    return (left_count,right_count)

def resetCounts():
    global left_count
    global right_count
    left_count=0
    right_count=0

def getSpeeds():
    global left_window
    global right_window
    global window_size
    global pwmLeftCurrent, pwmRightCurrent

    if sum(left_window)==0 or len(left_window)<1:
        left_speed=0
    else:
        left_speed= 2*math.pi*1.3*(max(len(left_window), 1)/32)/float(sum(left_window)) #calculate speed using formula using total_revolutions/total_time
        if pwmLeftCurrent<1.5:
            left_speed=-left_speed
    if sum(right_window)==0 or len(right_window)<1:
        right_speed=0
    else:
        right_speed= 2*math.pi*1.3*(max(len(right_window), 1)/32)/float(sum(right_window)) #calculate speed using formula using total_revolutions/total_time
        if pwmRightCurrent>1.5:
            right_speed=-right_speed

    return (round(left_speed,4),round(right_speed,4))

def turnRV(R, V):
    W=V/R
    setSpeedsVW(V,W)
