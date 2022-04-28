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

    distances,colors=utils.locateLandmarks()
    startX,startY=utils.trilateration(distances,colors)


    print("Initial position found using trilateration is {},{}".format(startX,startY))


    #
    # if startX>0 and startY>0:
    #     color='pink'
    #     direction=1
    # elif startX>0 and startY<0:
    #     color='blue'
    #     direction=1
    # elif startX<0 and startY>0:
    #     color='yellow'
    #     direction=1
    # elif startX<0 and startY<0:
    #     color='green'
    #     direction=1

    if startX>0:
        color='pink'
        direction=1

    elif startX<0:
        color='green'
        direction=1


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



    utils.initCellUpdate(color)



    f_setpoint=8
    l_setpoint=8

    startSpeedL=2
    startSpeedR=2


    pidLWall=PID.PID(1,0,0)
    pidLWall.SetPoint=l_setpoint
    rotate_count=0
    visited_all_cells=False


    cellSize=10
    utils.resetCounts()
    utils.printCellUpdate()
    while visited_all_cells == False:


        fDistance = fSensor.get_distance()*mmToInch
        lDistance = lSensor.get_distance()*mmToInch
        rDistance = rSensor.get_distance()*mmToInch


        encoder_counts=utils.getCounts()
        lEncoder=encoder_counts[0]
        rEncoder=encoder_counts[1]
        rDistance=rEncoder*0.255254

        if rDistance>cellSize:
            utils.resetCounts()
            utils.positionCellUpdate()
            utils.printCellUpdate()




        #print(fDistance,lDistance)
        if fDistance<f_setpoint:



            utils.rotateA(90)
            rotate_count=rotate_count+1
            utils.resetCounts()
            utils.thetaCellUpdate(90,rotate_count)
            utils.printCellUpdate()


        else:
            if rotate_count==3:
                f_setpoint=17

            elif rotate_count==4:
                l_setpoint=18
                utils.moveXV(2,2)
                pidLWall=PID.PID(1,0,0)
                pidLWall.SetPoint=l_setpoint

            elif rotate_count==7:
                visited_all_cells=True


            pidLWall.update(lDistance)
            setSpeedL=pidLWall.output+startSpeedL
            #print(pidLWall.SetPoint,lDistance,pidLWall.output,setSpeedL)
            setSpeedL=utils.clamp(setSpeedL,-3,3)
            utils.setSpeedsIPS(setSpeedL,startSpeedR,False)
            time.sleep(updateRate)


        #arrived at specified color landmark


    #Body code here -end

    #Cleanup

    print("Exiting")
    utils.setSpeedsPWM (1.508,1.5)
    GPIO.cleanup()
    exit()
