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

    exit()



signal.signal(signal.SIGINT, ctrlC)
utils.initTOF()
lSensor=utils.lSensor
fSensor=utils.fSensor
rSensor=utils.rSensor

mmToInch=0.0393701
updateRate=0.1
while True:
    fDistance = fSensor.get_distance()*mmToInch
    lDistance = lSensor.get_distance()*mmToInch
    rDistance = rSensor.get_distance()*mmToInch
    fDistance = round(fDistance,2)
    lDistance = round(lDistance,2)
    rDistance = round(rDistance,2)
    print("Front: {} Left: {} Right: {}".format(fDistance,lDistance,rDistance))
    time.sleep(updateRate)
