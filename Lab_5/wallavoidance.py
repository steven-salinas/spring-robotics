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
    pidLWall=PID.PID(1,0,0)


    lSensor=utils.lSensor
    fSensor=utils.fSensor
    rSensor=utils.rSensor

    #Getting PID thread ready




    print("Starting")




    #Body code here-start
    direction=1
    color='green'
    utils.findGoal(direction,color)
    arrived=utils.motionGoal(speed,1,fSensor,lSensor,color)
    while arrived is not True:
        fDistance = fSensor.get_distance()*mmToInch
        lDistance = lSensor.get_distance()*mmToInch
        rDistance = rSensor.get_distance()*mmToInch

        pidLWall.update(rDistance)
        setSpeedR=pidLWall.output+startSpeedR

        setSpeedR=utils.clamp(setSpeedR,-3,3)
        utils.setSpeedsIPS(startSpeedL,setSpeedR,False)
        time.sleep(updateRate)

        kps=utils.getBlobKP()
        if len(kps)>0:
            arrived=utils.motionGoal(speed,1,fSensor,lSensor)

    utils.rotateA(135)


    startSpeedL=2
    startSpeedR=2

    f_setpoint=11
    l_setpoint=10
    pidLWall.SetPoint=l_setpoint

    updateRate=0.1
    mmToInch=0.0393701

    fCount=0
    visited_all_cells=False
    rotate_count=0

    while visited_all_cells == False:


        fDistance = fSensor.get_distance()*mmToInch
        lDistance = lSensor.get_distance()*mmToInch
        rDistance = rSensor.get_distance()*mmToInch

        timeElapsed=time.time()-startTime
        timeElapsed=round(timeElapsed,2)
        print("Left: {} Right: {} Front: {} Time Elapsed: {}".format(round(lDistance,2),round(rDistance,2),round(fDistance,2),timeElapsed))

        #print(fDistance,lDistance)
        if fDistance<f_setpoint:
            if rotate_count==3:
                f_setpoint=17

            elif rotate_count==4:
                l_setpoint=16

            elif rotate_count==7:
                visited_all_cells==True

            utils.rotateA(90)
            rotate_count=rotate_count+1


        else:


            pidLWall.update(lDistance)
            setSpeedL=pidLWall.output+startSpeedL
            #print(pidLWall.SetPoint,lDistance,pidLWall.output,setSpeedL)
            setSpeedL=utils.clamp(setSpeedL,-3,3)
            utils.setSpeedsIPS(setSpeedL,startSpeedR,False)
            time.sleep(updateRate)


    #Body code here -end

    #Cleanup

    print("Exiting")
    utils.setSpeedsPWM (1.508,1.5)
    GPIO.cleanup()
    exit()
