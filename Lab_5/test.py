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

    utils.setSpeedsPWM (1.508,1.5)
    lSensor.stop_ranging()
    fSensor.stop_ranging()
    rSensor.stop_ranging()

    GPIO.cleanup()
    utils.cleanupBlob()
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
    utils.initIMU()
    utils.initBlob()
    utils.initTOF()


    lSensor=utils.lSensor
    fSensor=utils.fSensor
    rSensor=utils.rSensor

    pidLWall=PID.PID(0,0,0)
    pidLWall.SetPoint=7


    print("Starting ")

    #Body code here-start
    speed=2
    startSpeedL=2
    startSpeedR=2




    updateRate=0.1
    mmToInch=0.0393701
    fSensor=utils.fSensor
    mmToInch=0.0393701
    while True:
        fDistance = fSensor.get_distance()*mmToInch
        if fDistance<18:
            utils.rotateA(90)

        else:
            utils.moveCell(18,2)
        # time.sleep(1)
        # print("Moved 1 cell")


        #arrived at specified color landmark


    #Body code here -end

    #Cleanup

    print("Exiting")
    utils.setSpeedsPWM (1.508,1.5)
    GPIO.cleanup()
    exit()
