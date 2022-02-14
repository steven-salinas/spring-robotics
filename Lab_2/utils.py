import time
import Adafruit_PCA9685
import signal
import math
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/
from itertools import cycle
import PID


# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 1
RSERVO = 0

LENCODER = 17
RENCODER = 18

ipsLeftCurrent=0
ipsRightCurrent=0


def initPID():
    global pidL, pidR
    P=0.1
    I=0.001
    D=0
    pidL = PID.PID(P,I,D)
    pidR = PID.PID(P,I,D)
    pidL.SetPoint=0
    pidR.SetPoint=0


# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")

    # Stop the servos


    GPIO.cleanup()
    setSpeedsPWM (1.5,1.495)
    exit()

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)

def initMotors():
    global pwm
    # Initialize the servo hat library.
    pwm = Adafruit_PCA9685.PCA9685()

    # 50Hz is used for the frequency of the servos.
    pwm.set_pwm_freq(50)
    return pwm

def remap(value, fromLow, fromHigh, toLow, toHigh):
    fromRange = (fromHigh - fromLow)
    toRange= (toHigh - toLow)
    newValue = (((value - fromLow) * toRange) / fromRange) + toLow
    # newValue=round(newValue,3)
    return newValue

def clamp(value, low, high):
    return max(min(high, value), low)


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

            ticks_elapsed=(getCounts()[1]+getCounts()[0])/2

        time_elapsed=round(time.time()-start_time,3)
        print("Finished moving in {} seconds\n".format(time_elapsed))
        setSpeedsPWM (1.5,1.495)
    else:
        print("Requested velocity is not able to be set, movement cancelled\n")
        setSpeedsPWM (1.5,1.495)




def setSpeedsPWM (pwmLeft, pwmRight):
    global pidL, pidR
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
        #print(pidL.SetPoint,getSpeeds()[0],"\t",pidR.SetPoint,getSpeeds()[1])
        pwmLeft=round(pwmLeft,5)
        pwmRight=round(pwmRight,5)
        #print("Setting motor speed in PWM, left: {} right: {}".format(pwmLeft,pwmRight))
        pwm.set_pwm(LSERVO, 0, math.floor(pwmLeft / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(pwmRight / 20 * 4096));
        status=True
    return status



def setSpeedsRPS (rpsLeft, rpsRight):
    #print("Setting motor speed in RPS, left: {} right: {}".format(rpsLeft,rpsRight))
    minRPS=-0.8
    maxRPS=0.8
    # rpsLeft=clamp(rpsLeft,minRPS,maxRPS)
    # rpsRight=clamp(rpsRight,minRPS,maxRPS)

    rpsLeft=clamp(rpsLeft,minRPS,maxRPS)
    rpsRight=clamp(rpsRight,minRPS,maxRPS)

    ipsLeft=rpsLeft*2*math.pi*1.3
    ipsRight=rpsRight*2*math.pi*1.3

    setSpeedsIPS(ipsLeft,ipsRight)



def setSpeedsIPS(ipsLeft, ipsRight):
    global ipsLeftCurrent, ipsRightCurrent
    global pidL,pidR
    #print("Setting motor speed in IPS, left: {} right: {}".format(ipsLeft,ipsRight))
    minIPS=-6.8
    maxIPS=6.8
    ipsLeft=clamp(ipsLeft,minIPS,maxIPS)
    ipsRight=clamp(ipsRight,minIPS,maxIPS)

    ipsLeftCurrent=ipsLeft
    ipsRightCurrent=ipsRight


    pidL.SetPoint=ipsLeftCurrent
    pidR.SetPoint=ipsRightCurrent
    # print(pidL.SetPoint)


    pwmL=remap(ipsLeft,minIPS,maxIPS,1.3,1.7)
    pwmR=remap(ipsRight,minIPS,maxIPS,1.7,1.3)
    status=setSpeedsPWM(pwmL,pwmR)
    return status

def setSpeedsVW(V, W):
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

    if abs(Vl) <=6.8 and abs(Vr) <=6.8:

        setSpeedsIPS(Vl,Vr)
    else:
        print("Motors are not capable of completing maneuver, outer {} inner {}".format(Vl,Vr))






countL=0
countR=0

speedL=0
speedR=0

def onLeftEncode(pin):
    global ipsLeftCurrent, ipsRightCurrent
    global pidL
    global countL
    global prevEncL
    global speedL
    minIPS=-6.8
    maxIPS=6.8

    countL=countL+1 #Increment tick count

    now=time.time()
    speedL=2*math.pi*1.3*(1/32)/(now-prevEncL)

    if ipsLeftCurrent<0:
        speedL=-speedL
    print(speedL)

    pidL.update(speedL)
    ipsLeftCurrent=pidL.output+ipsLeftCurrent


    pwmL=remap(ipsLeftCurrent,minIPS,maxIPS,1.3,1.7)
    pwmR=remap(ipsRightCurrent,minIPS,maxIPS,1.7,1.3)

    setSpeedsPWM(pwmL,pwmR)



    prevEncL=now


    #print("Left encoder ticked!")

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global ipsLeftCurrent, ipsRightCurrent
    global pidR
    global countR
    global prevEncR
    global speedR
    minIPS=-6.8
    maxIPS=6.8


    countR=countR+1 #Increment tick count

    now=time.time()



    speedR=2*math.pi*1.3*(1/32)/(now-prevEncR)

    if ipsRightCurrent<0:
        speedR=-speedR

    pidR.update(speedR)
    ipsRightCurrent=pidR.output+ipsRightCurrent



    pwmL=remap(ipsLeftCurrent,minIPS,maxIPS,1.3,1.7)
    pwmR=remap(ipsRightCurrent,minIPS,maxIPS,1.7,1.3)

    setSpeedsPWM(pwmL,pwmR)


    prevEncR=now

def initEncoders():
    global prevEncL
    global prevEncR

    prevEncL=time.time()
    prevEncR=time.time()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)


    GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
    GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

def getCounts():
    global countL, countR
    return (countL,countR)

def resetCounts():
    global countL, countR
    countL=0
    countR=0

def getSpeeds():
    global speedL, speedR
    retL=speedL
    retR=speedR
    # speedL=0
    # speedR=0
    return (retL,retR)

def turnRV(R, V):
    W=V/R
    setSpeedsVW(V,W)
