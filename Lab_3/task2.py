import utils
import time

import signal
import math
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/

import PID
import threading


def ctrlC(signum, frame):
    global breakFlag
    breakFlag=True
    print("Ctrl-C caught")

    # threadPID.join()
    print("Exiting")

    lSensor.stop_ranging()
    fSensor.stop_ranging()
    rSensor.stop_ranging()
    utils.setSpeedsPWM (1.508,1.5)
    GPIO.cleanup()
    exit()






if __name__ == "__main__":
    global breakFlag
    global pidL, pidR
    global lSensor, fSensor, rSensor
    global startSpeedL, startSpeedR




    breakFlag=False
    signal.signal(signal.SIGINT, ctrlC)

    #initialize motors/encoders, PID is initialized with values inputted
    utils.initEncoders()
    utils.initMotors()
    utils.initPID(0,0,0)
    # imu=utils.initIMU()
    utils.initIMU()
    utils.initTOF()



    #Grab initialized objects from utils
    pidLWall=PID.PID(1,0,0.4)
    pidFWall=PID.PID(1,0,0.4)

    lSensor=utils.lSensor
    fSensor=utils.fSensor
    rSensor=utils.rSensor

    #Getting PID thread ready




    print("Starting")




    #Body code here-start



    fWallDistance=10 #Desired distance from front wall
    lWallDistance=2 #Desired distance from left wall

    startSpeedL=3 #Desired speed left (left and right should be same)
    startSpeedR=3 #Desired speed right

    maxStartSpeedL=startSpeedL
    maxStartSpeedR=startSpeedR

    pidLWall.SetPoint=lWallDistance
    pidFWall.SetPoint=fWallDistance


    updateRate=0.1
    mmToInch=0.0393701

    fCount=0
    while breakFlag is not True:

        fDistance = fSensor.get_distance()*mmToInch


        lDistance = lSensor.get_distance()*mmToInch
        pidLWall.update(lDistance)
        setSpeedL=pidLWall.output+startSpeedL
        #print(pidLWall.SetPoint,lDistance,pidLWall.output,setSpeedL)
        setSpeedL=utils.clamp(setSpeedL,-5,5)
        utils.setSpeedsIPS(setSpeedL,startSpeedR,False)

        time.sleep(updateRate)


    #Body code here -end

    #Cleanup

    print("Exiting")
    utils.setSpeedsPWM (1.508,1.5)
    GPIO.cleanup()
    exit()
