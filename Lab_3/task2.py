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
    pidWall=PID.imuPID(1,0,0)



    lSensor=utils.lSensor
    fSensor=utils.fSensor
    rSensor=utils.rSensor

    #Getting PID thread ready




    print("Starting")




    #Body code here-start



    startSpeedL=2
    startSpeedR=2


    pidWall.SetPoint=0




    updateRate=0.1
    mmToInch=0.0393701

    fCount=0
    startTime=time.time()
    while time.time()-startTime<500:




        fDistance = fSensor.get_distance()*mmToInch
        lDistance = lSensor.get_distance()*mmToInch
        rDistance = rSensor.get_distance()*mmToInch

        timeElapsed=time.time()-startTime
        timeElapsed=round(timeElapsed,2)
        print("Left: {} Right: {} Front: {} Time Elapsed: {}".format(round(lDistance,2),round(rDistance,2),round(fDistance,2),timeElapsed))

        #print(fDistance,lDistance)


        error=lDistance-rDistance
        pidWall.update(error)

        setSpeedL=-pidWall.output+startSpeedL
        setSpeedR=pidWall.output+startSpeedR
        #print(pidLWall.SetPoint,lDistance,pidLWall.output,setSpeedL)

        setSpeedL=utils.clamp(setSpeedL,-3,3)
        setSpeedR=utils.clamp(setSpeedR,-3,3)
        print(setSpeedL,setSpeedR)
        utils.setSpeedsIPS(setSpeedL,setSpeedR,False)
        time.sleep(updateRate)


    #Body code here -end

    #Cleanup

    print("Exiting")
    utils.setSpeedsPWM (1.508,1.5)
    GPIO.cleanup()
    exit()
