# This program demonstrates the usage of the time of flight sensors.
# After running the program, move your hand in front of each sensor to verify that it's working.
# See https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/overview for more details.

import time
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import utils

utils.initTOF()

lSensor=utils.lSensor
fSensor=utils.fSensor
rSensor=utils.rSensor


for count in range(1, 100):
    # Get a measurement from each sensor
    mmToInch=0.0393701
    lDistance = lSensor.get_distance()*mmToInch
    fDistance = fSensor.get_distance()*mmToInch
    rDistance = rSensor.get_distance()*mmToInch

    # Print each measurement
    print(fDistance)
    #print("Left: {}\tFront: {}\tRight: {}".format(lDistance, fDistance, rDistance))

# Stop measurement for all sensors
lSensor.stop_ranging()
fSensor.stop_ranging()
rSensor.stop_ranging()
