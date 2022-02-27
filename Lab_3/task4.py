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


    lSensor=utils.lSensor
    fSensor=utils.fSensor
    rSensor=utils.rSensor

    #Getting PID thread ready




    print("Starting")




    #Body code here-start



    startSpeedL=3
    startSpeedR=3


    pidLWall.SetPoint=5



    updateRate=0.1
    mmToInch=0.0393701

    fCount=0
    while breakFlag is not True:



        fDistance = fSensor.get_distance()*mmToInch
        lDistance = lSensor.get_distance()*mmToInch


        #print(fDistance,lDistance)
        if fDistance<5:
            fCount=fCount+1
            if fCount>1:
                fCount=0
                utils.rotateA(90)

                # lCount=0
                # while lCount<3:
                #
                #     lDistance = lSensor.get_distance()*mmToInch
                #     if lDistance<7:
                #         lCount=lCount+1
                #     else:
                #         lCount=0
                #     pass
        else:
            fCount=0
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
