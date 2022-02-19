import utils
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

def ctrlC(signum, frame):
    global breakFlag
    breakFlag=True
    print("Ctrl-C caught")

    threadPID.join()
    print("Exiting")
    utils.setSpeedsPWM(1.5,1.5)
    GPIO.cleanup()
    exit()




def loopPID(name):
    global breakFlag
    global enablePID
    global pidL, pidR

    global setSpeedL, setSpeedR
    print("PID thread started")


    #How fast PID updates, when it's too fast PID updates faster than speed measurements can update leading to crazy high overshoot, too slow it takes a while to steady state
    #0.4 is not bad
    updateRate=0.3

    #Initialize to 0 for now, gets updated when any set speed command is called
    setSpeedL=0
    setSpeedR=0

    if enablePID:
        while breakFlag is not True:

            time.sleep(updateRate)

            speedL, speedR=utils.getSpeeds()

            pidL.update(speedL)
            pidR.update(speedR)

            # print(speedL,speedR)

            # print(pidL.output,'\t',pidR.output)
            setSpeedL=pidL.output+setSpeedL
            setSpeedR=pidR.output+setSpeedR
            #print(speed,y,current_speedR,pidR.output)
            utils.setSpeedsIPS(setSpeedL,setSpeedR,False)

    print("PID thread ended")


if __name__ == "__main__":
    global enablePID
    global breakFlag
    global pidL, pidR

    enablePID=True


    breakFlag=False
    signal.signal(signal.SIGINT, ctrlC)

    #initialize motors/encoders, PID is initialized with values inputted
    utils.initEncoders()
    utils.initMotors()
    utils.initPID(0.12,0.03,0.01)
    imu=utils.initIMU()

    #Grab initialized pid objects
    pidL=utils.pidL
    pidR=utils.pidR

    #Getting PID thread ready
    threadPID = threading.Thread(target=loopPID, args=(1,))

    print("Starting")
    threadPID.start()


    #Body code here-start


    while breakFlag is not True:
        print(imu.euler)


    #Body code here -end

    #Cleanup
    breakFlag=True
    threadPID.join()
    print("Exiting")
    utils.setSpeedsPWM(1.5,1.5)
    GPIO.cleanup()
    exit()
