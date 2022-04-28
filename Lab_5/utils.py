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
import board
import adafruit_bno055
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import cv2
import cv2 as cv

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 1
RSERVO = 0

LENCODER = 17
RENCODER = 18

setSpeedL=0
setSpeedR=0

global pidL, pidR

def moveCell(X,V):
    global maxIPS
    global lSensor, rSensor, fSensor

    mmToInch=0.0393701

    travel_time=abs(X/V)

    num_ticks= X/0.255254
    second_timer=time.time()+0.1
    start_time=time.time()
    resetCounts()
    updateRate=0.01
    lTrigger=8.7
    rTrigger=7
    adjustVal=1.5
    if abs(V)<maxIPS :
        vL=V
        vR=V
        setSpeedsIPS(vL,vR)
        ticks_elapsed=(getCounts()[1]+getCounts()[0])/2
        while ticks_elapsed <num_ticks:
            lDistance = lSensor.get_distance()*mmToInch
            rDistance = rSensor.get_distance()*mmToInch
            fDistance = fSensor.get_distance()*mmToInch
            ticks_elapsed=(getCounts()[1]+getCounts()[0])/2
            # print(lDistance,rDistance)
            if fDistance<7:
                print("front is too close, stopping")
                break
            elif rDistance<rTrigger:
                vR=V+adjustVal
                vL=V
                # vR=vR+adjustVal
                # vL=vL-adjustVal
                setSpeedsIPS(vL,vR)
                # print("right too close")
                # print(vR,vL)

            elif lDistance<lTrigger:
                vL=V+adjustVal
                vR=V
                # vR=vR-adjustVal
                # vL=vL+adjustVal
                setSpeedsIPS(vL,vR)
                # print("left too close")
                # print(vR,vL)

            else:

                vR=V
                vL=V
                setSpeedsIPS(vL,vR)
                # print("back to normal")
                # print(vR,vL)

                time.sleep(updateRate)

        setSpeedsPWM (1.5,1.495)
    else:
        print("Requested velocity is not able to be set, movement cancelled\n")
        setSpeedsPWM (1.5,1.495)

def initBlob():
    global blob
    blob=ThreadedBlob()
    blob.start()

def cleanupBlob():
    global blob
    blob.stop()

def getBlobKP():
    global blob
    return blob.read()

def locateLandmarks():
    global fSensor
    mmToInch=0.0393701
    findGoal(1,'pink')

    time.sleep(1)
    pinkDistance=fSensor.get_distance()*mmToInch

    findGoal(1,'blue')
    time.sleep(1)
    blueDistance=fSensor.get_distance()*mmToInch

    findGoal(1,'green')
    time.sleep(1)
    greenDistance=fSensor.get_distance()*mmToInch

    findGoal(1,'yellow')
    time.sleep(1)
    yellowDistance=fSensor.get_distance()*mmToInch

    landmarkColors=['pink','blue','green','yellow']
    landmarkDistances=[pinkDistance,blueDistance,greenDistance,yellowDistance]

    return landmarkDistances,landmarkColors

def trilateration(landmarkDistances,landmarkColors):
    maxDistance=max(landmarkDistances)
    maxIndex=landmarkDistances.index(maxDistance)
    del landmarkDistances[maxIndex]
    del landmarkColors[maxIndex]

    mapLength=48
    #in inches

    circleValues=[]
    for i in range(len(landmarkDistances)):
        if landmarkColors[i]=='pink':
            circleValues.append([mapLength/2,mapLength/2,landmarkDistances[i]])
        if landmarkColors[i]=='yellow':
            circleValues.append([-mapLength/2,mapLength/2,landmarkDistances[i]])
        if landmarkColors[i]=='blue':
            circleValues.append([mapLength/2,-mapLength/2,landmarkDistances[i]])
        if landmarkColors[i]=='green':
            circleValues.append([-mapLength/2,-mapLength/2,landmarkDistances[i]])

    x1,y1,r1=circleValues[0]
    x2,y2,r2=circleValues[1]
    x3,y3,r3=circleValues[2]

    A=-2*x1+2*x2
    B=-2*y1+2*y2
    C=(r1**2)- (r2**2)-(x1**2)+(x2**2)-(y1**2)+(y2**2)
    D=-2*x2+2*x3
    E=-2*y2+2*y3
    F=(r2**2)- (r3**2)-(x2**2)+(x3**2)-(y2**2)+(y3**2)

    x=(C*E-F*B)/(E*A-B*D)
    y=(C*D-A*F)/(B*D-A*E)


    return x,y



def centerSquare():
    # print("Starting centering")
    mmToInch=0.0393701
    fDistance = fSensor.get_distance()*mmToInch
    fDistanceLast=fDistance
    fMin=0
    fMinCount=0
    fDeltaNegCount=0
    fDeltaPosCount=0
    minMet=False
    setSpeedsIPS(1,-1)
    time.sleep(0.1)
    while minMet is not True:
        fDistance = fSensor.get_distance()*mmToInch
        fDelta=fDistance-fDistanceLast
        if fDistance >15:
            print("Passing because fdistance is big")
            continue
        elif fDelta<=0:
            print("fdelta is negative, incrementing neg count {} ".format(fDeltaNegCount))
            fDeltaNegCount=fDeltaNegCount+1
        elif fDelta>0 and fDeltaNegCount>2:
            print("fdelta is positive, incrementing pos count {} ".format(fDeltaPosCount))
            fDeltaPosCount=fDeltaPosCount+1

        if fDeltaPosCount>5:
            print("Finished")
            setSpeedsIPS(0,0)
            minMet=True
        fDistanceLast=fDistance

    setSpeedsIPS(1,1)
    fDistance = fSensor.get_distance()*mmToInch
    while fDistance>7:
        fDistance = fSensor.get_distance()*mmToInch
    setSpeedsIPS(0,0)

def motionGoal(startSpeed,direction,fSensor=None,sSensor=None,color='pink'):
    global blob

    if color=='pink':
        blob.minH =  158; blob.minS = 78; blob.minV =  98;
        blob.maxH = 180; blob.maxS = 255; blob.maxV = 255;
    elif color=='green':
        blob.minH =  45; blob.minS = 74; blob.minV =  48;
        blob.maxH = 86; blob.maxS = 255; blob.maxV = 255;
    elif color=='blue':
        blob.minH =  92; blob.minS = 144; blob.minV =  63;
        blob.maxH = 118; blob.maxS = 229; blob.maxV = 165;
    elif color=='yellow':
        blob.minH =  22; blob.minS = 47; blob.minV =  154;
        blob.maxH = 52; blob.maxS = 88; blob.maxV = 255;

    #fSensor, uses fsensor as stopping condition if given
    #start speed is the base speed to move to goal to
    print("Starting motion to {} goal".format(color))

    pidLBlob=PID.PID(0.01,0,0)
    pidLBlob.SetPoint=320
    updateRate=0.1

    maxBlobSize=300

    kps=getBlobKP()
    while len(kps)==0:
        print("Lost goal, finding again")
        findGoal(direction)
        kps=getBlobKP()

    blobSize= kps[0].size
    blobPosition = kps[0].pt[0]
    arrived=True
    if fSensor is not None:
        mmToInch=0.0393701
        fDistance = fSensor.get_distance()*mmToInch
    while fDistance>7:

        fDistance = fSensor.get_distance()*mmToInch


        pidLBlob.update(blobPosition)
        setSpeedL=-pidLBlob.output+startSpeed

        setSpeedL=clamp(setSpeedL,-3,3)
        setSpeedsIPS(setSpeedL,startSpeed,False)


        kps=getBlobKP()
        while len(kps)==0:
            print("Lost goal, finding again")
            findGoal(direction)
            kps=getBlobKP()

        blobSize= kps[0].size
        blobPosition = kps[0].pt[0]



        time.sleep(updateRate)

    if arrived==True:
        print("Arrived at goal\n")
    return arrived

def findGoal(direction,color='pink'):
    #direction is 1 or 0, 1 clockwise, 0 anti-clockwise
    global pidL,pidR
    global blob

    if color=='pink':
        blob.minH =  158; blob.minS = 78; blob.minV =  98;
        blob.maxH = 180; blob.maxS = 255; blob.maxV = 255;
    elif color=='green':
        blob.minH =  45; blob.minS = 74; blob.minV =  48;
        blob.maxH = 86; blob.maxS = 255; blob.maxV = 255;
    elif color=='blue':
        blob.minH =  92; blob.minS = 144; blob.minV =  63;
        blob.maxH = 118; blob.maxS = 229; blob.maxV = 165;
    elif color=='yellow':
        blob.minH =  22; blob.minS = 47; blob.minV =  154;
        blob.maxH = 52; blob.maxS = 88; blob.maxV = 255;


    print("Finding {} goal in direction {}".format(color,direction))
    setSpeedsIPS(0,0)


    pidL.enable=False
    pidR.enable=False

    pidBlob=PID.PID(0.01,0,0)

    speed=2

    blobResolution=5

    blobGoal=320

    kps=getBlobKP()

    if len(kps)>0:
        blobPosition = kps[0].pt[0]
    else:
        blobPosition=direction*640


    blobError= blobPosition - blobGoal

    initUpdateTime=(2*math.pi*2*3/360)/speed
    while abs(blobError)>blobResolution:

        pidBlob.update(blobPosition)
        output=abs(pidBlob.output)

        updateTime=(2*math.pi*2*output/360)/speed
        if blobError>0: #if error is positive turn right

            setSpeedsIPS(speed,-speed,False)

        else: #if negative turn left
            setSpeedsIPS(-speed,speed,False)

        time.sleep(updateTime)
        setSpeedsIPS(0,0,False)
        time.sleep(0.1)

        kps=getBlobKP()
        if len(kps)>0:
            blobPosition = kps[0].pt[0]
        else:
            blobPosition=direction*640

        blobError= blobPosition - blobGoal


    print("Finished finding goal\n")
    pidL.enable=True
    pidR.enable=True

def initPID(P,I,D):
    global pidL, pidR

    pidL = PID.PID(P,I,D)
    pidR = PID.PID(P,I,D)
    pidL.SetPoint=0
    pidR.SetPoint=0
    # return pidL, pidR

def initTOF():
    global lSensor, fSensor, rSensor
    LSHDN = 27
    FSHDN = 22
    RSHDN = 23

    DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
    LADDR = 0x2a
    RADDR = 0x2b

    # Set the pin numbering scheme to the numbering shown on the robot itself.
    GPIO.setmode(GPIO.BCM)

    # Setup pins
    GPIO.setup(LSHDN, GPIO.OUT)
    GPIO.setup(FSHDN, GPIO.OUT)
    GPIO.setup(RSHDN, GPIO.OUT)

    # Shutdown all sensors
    GPIO.output(LSHDN, GPIO.LOW)
    GPIO.output(FSHDN, GPIO.LOW)
    GPIO.output(RSHDN, GPIO.LOW)

    time.sleep(0.01)

    # Initialize all sensors
    lSensor = VL53L0X.VL53L0X(address=LADDR)
    fSensor = VL53L0X.VL53L0X(address=DEFAULTADDR)
    rSensor = VL53L0X.VL53L0X(address=RADDR)

    # Connect the left sensor and start measurement
    GPIO.output(LSHDN, GPIO.HIGH)
    time.sleep(0.01)
    lSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

    # Connect the right sensor and start measurement
    GPIO.output(RSHDN, GPIO.HIGH)
    time.sleep(0.01)
    rSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

    # Connect the front sensor and start measurement
    GPIO.output(FSHDN, GPIO.HIGH)
    time.sleep(0.01)
    fSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

def initIMU():
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    return sensor

def initMotors():
    global pwm
    # Initialize the servo hat library.
    pwm = Adafruit_PCA9685.PCA9685()

    # 50Hz is used for the frequency of the servos.
    pwm.set_pwm_freq(50)
    #setSpeedsPWM (1.5,1.495)
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
    global maxIPS

    travel_time=abs(X/V)
    # print("Moving a distance of {} inches at {} linear inches per second for {} seconds".format(X,V,travel_time))

    num_ticks= X/0.255254

    second_timer=time.time()+1
    start_time=time.time()
    resetCounts()

    if abs(V)<maxIPS :
        setSpeedsIPS(V,V)
        # while time.time()<end_time:
        ticks_elapsed=(getCounts()[1]+getCounts()[0])/2
        while ticks_elapsed <num_ticks:

            if time.time()>second_timer:
                second_timer=time.time()+1
                time_elapsed=round(time.time()-start_time,3)
                distance_elapsed=round(ticks_elapsed*0.255254,3)
#                print("{} seconds pased, moved {} inches".format(time_elapsed,distance_elapsed))


            ticks_elapsed=(getCounts()[1]+getCounts()[0])/2

        time_elapsed=round(time.time()-start_time,3)
        #print("Finished moving in {} seconds\n".format(time_elapsed))
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

    else:
        pwmLeftCurrent=pwmLeft
        pwmRightCurrent=pwmRight
        #print("Setting motor speed in PWM, left: {} right: {}".format(pwmLeft,pwmRight))
        pwm.set_pwm(LSERVO, 0, math.floor(pwmLeft / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(pwmRight / 20 * 4096));




def setSpeedsRPS (rpsLeft, rpsRight):
    global maxRPS
    global wheel_diam

    if abs(rpsLeft)>maxRPS or abs(rpsRight)>maxRPS:
        print("Not able to set motor speed to RPS, left: {} right: {}".format(rpsLeft,rpsRight))
    else:
        ipsLeft=(2*math.pi*wheel_diam)*rpsLeft
        ipsRight=(2*math.pi*wheel_diam)*rpsRight
        setSpeedsIPS(ipsLeft,ipsRight)



def setSpeedsIPS(ipsLeft, ipsRight,setPID=True):
    global maxIPS

    global pwmMapL
    global ipsMapL

    global pwmMapR
    global ipsMapR

    global pidL
    global pidR

    global setSpeedL, setSpeedR

    # print(setPID,ipsLeft,ipsRight)
    # print(pidL.SetPoint)

    pidL.clear()
    pidR.clear()
    setSpeedL=ipsLeft
    setSpeedR=ipsRight

    if setPID is True:
        pidL.SetPoint=ipsLeft
        pidR.SetPoint=ipsRight
        # print(pidL.SetPoint)



    if abs(ipsLeft)>maxIPS or abs(ipsRight)>maxIPS:
        print("Not able to set motor speed to IPS, left: {} right: {}".format(ipsLeft,ipsRight))
        status=False

    else:
        #print("Setting motor speed in IPS, L: {} R: {}".format(ipsLeft,ipsRight))
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


        #print(pwmL,pwmR)
        setSpeedsPWM(pwmL,pwmR)


def setSpeedsVW(V, W):
    global maxIPS
    #print("Setting motor speed in VW, V: {} W: {}".format(V,W))

    if W == 0:
        Vl=V
        Vr=V
    else:
        R=abs(V/W) #inches

        dmid=2 #half of wheel axis distance in inches
        if W>0:
            Vl=abs(W)*(R+dmid)#V of outer wheel
            Vr=abs(W)*(R-dmid)#V of innner wheel
        else:
            Vl=abs(W)*(R-dmid)#V of outer wheel
            Vr=abs(W)*(R+dmid)#V of innner wheel

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

def resetSpeeds():
    global left_window, right_window

    left_window=[]
    right_window=[]
def turnRV(R, V):
    W=V/R
    setSpeedsVW(V,W)



'''
    Setting up IMU sensor settings
'''

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF

'''
    IMU functions
'''
def temperature():
    global last_val
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
        last_val = result
    return result

# Function to turn robot 90 degrees using IMU euler angle
# Function should stop turning robot after turing 90 degrees clockwise
#
# Note - However there is a small amount of inconsistency
#        due to not being able to call euler position frequently
#        enough without erroneous response
def rotateA(angle):
    global pidL,pidR
    print("Rotating {} degrees".format(angle))
    setSpeedsIPS(0,0)


    pidL.enable=False
    pidR.enable=False

    pidIMU=PID.imuPID(0.4,0,0)

    speed=2

    angleResolution=2

    # angle=angle % 360
    angleCurrent = sensor.euler[0]
    angleGoal=(angleCurrent+angle)%360

    angleError= (angleCurrent - angleGoal + 540)%360 - 180

    initUpdateTime=(2*math.pi*2*5/360)/speed
    while abs(angleError)>angleResolution:

        pidIMU.update(angleError)
        output=abs(pidIMU.output)

        updateTime=(2*math.pi*2*output/360)/speed
        if angleError>0: #if error is positive turn left

            setSpeedsIPS(-speed,speed,False)

        else: #if negative turn right
            setSpeedsIPS(speed,-speed,False)

        time.sleep(updateTime)
        setSpeedsIPS(0,0,False)
        time.sleep(0.001)
        angleCurrent = sensor.euler[0] if sensor.euler[0] is not None else angleCurrent
        angleError= (angleCurrent - angleGoal + 540)%360 - 180
        # time.sleep(.1)

            # angleErrorWindow.append(abs(angleError)) #append elapsed time between ticks to window
            # angleErrorWindow.pop(0) #removes oldest entry, required for moving average
            # angleErrorAvg=sum(angleErrorWindow)/len(angleErrorWindow)
            # print(int(angleErrorAvg))

    setSpeedsIPS(0,0)
    time.sleep(0.1)
    angleCurrent = sensor.euler[0] if sensor.euler[0] is not None else angleCurrent
    angleError= (angleCurrent - angleGoal + 540)%360 - 180
    print("Finished rotating {} deg with {} deg of error".format(angle,angleError))
    pidL.enable=True
    pidR.enable=True

lastIMU=0
def getIMUDegrees():
    global lastIMU

    val=sensor.euler[0]
    if val== None or val>360 or val<0:
        deg=lastIMU
    else:
        deg=val
        lastIMU=val

    return deg



def initCellUpdate(color):
    global currentX, currentY,currentGrid,currentTheta
    if color=='pink':
        currentX=13
        currentY=13
        currentGrid=4
        currentTheta=270

    if color=='blue':
        currentX=13
        currentY=-13
        currentGrid=16
        currentTheta=180

    if color=='green':
        currentX=-13
        currentY=-13
        currentGrid=13
        currentTheta=90

    if color=='yellow':
        currentX=-13
        currentY=-13
        currentGrid=1
        currentTheta=0


def thetaCellUpdate(angle,rotate_count):
    global currentX, currentY,currentGrid,currentTheta
    print("Updating theta estimate")
    print("Previous theta is {}".format(currentTheta))
    if rotate_count>=4:
        if currentTheta==270:
            currentTheta=180
            currentX=1
            currentY=-1
            currentGrid=11
        elif currentTheta==180:
            currentTheta=90
            currentX=-1
            currentY=-1
            currentGrid=10

        elif currentTheta==90:
            currentTheta=0
            currentX=-1
            currentY=1
            currentGrid=6
        elif currentTheta==0:
            currentTheta=270
            currentX=1
            currentY=1
            currentGrid=7

    else:
        if currentTheta==270:
            currentTheta=180
            currentX=13
            currentY=-13
            currentGrid=16
        elif currentTheta==180:
            currentTheta=90
            currentX=-13
            currentY=-13
            currentGrid=13

        elif currentTheta==90:
            currentTheta=0
            currentX=-13
            currentY=13
            currentGrid=1
        elif currentTheta==0:
            currentTheta=270
            currentX=13
            currentY=13
            currentGrid=4
    print("Current theta is {}".format(currentTheta))

def positionCellUpdate():
    print("Updating position estimate")

    global currentX, currentY,currentGrid,currentTheta


    if currentTheta==270:
        xMult=0
        yMult=-1
    if currentTheta==180:
        xMult=-1
        yMult=0
    if currentTheta==90:
        xMult=0
        yMult=1
    if currentTheta==0:
        xMult=1
        yMult=0

    print("Went from {}, {}".format(currentX,currentY))

    currentX=(xMult*12)+currentX
    currentY=(yMult*12)+currentY
    print("To {}, {}".format(currentX,currentY))



visitedCells=[['.','.','.','.'],['.','.','.','.'],['.','.','.','.'],['.','.','.','.']]
def printCellUpdate():
    global currentX, currentY,currentGrid,currentTheta,visitedCells
    # gridMap=[[13,14,15,16],[9,10,11,12],[5,6,7,8],[1,2,3,4]]
    gridMap=[[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]]

    gridMapX=abs(int((currentX+24)/12))
    gridMapY=abs(int((currentY-24)/12))

    currentGrid=gridMap[gridMapY][gridMapX]

    visitedCells[gridMapY][gridMapX]='X'
    for i in visitedCells:
        print(i)

    print("Pose is ({},{},{},{})".format(currentX,currentY,currentGrid,currentTheta))


    #
    # while abs(angleError)>angleResolution:
    #
    #
    #     if angleError>0: #if error is positive turn left
    #         setSpeedsIPS(-speed/4,speed/4)
    #     else: #if negative turn right
    #         setSpeedsIPS(speed/4,-speed/4)
    #
    #
    #     angleCurrent = sensor.euler[0] if sensor.euler[0] is not None else angleCurrent
    #     angleError= (angleCurrent - angleGoal + 540)%360 - 180
