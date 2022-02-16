'''
	task1.py completes maneuver of rectangle of height H and width W.
	The completed manuever should result in a outline of a rectangle H*W.
'''
import utils
import time
import Adafruit_PCA9685
import signal
import math
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/
import numpy as np
import PID
import threading
import adafruit_bno055

# Stop robot by pressing Ctrl + C
def ctrlC(signum, frame):
	global breakFlag
	breakFlag=True
	print("Ctrl-C caught")

	threadPID.join()
	print("Exiting")
	utils.setSpeedsPWM (1.5,1.495)
	GPIO.cleanup()
	exit()

# Loop for robot PID thread
def loopPID(name):
	global breakFlag
	global pidL, pidR
	global setSpeedL, setSpeedR

	print("PID thread started")

	#How fast PID updates, when it's too fast PID updates faster than speed measurements can update leading to crazy high overshoot, too slow it takes a while to steady state
	#0.4 is not bad
	updateRate=0.3

	#Initialize to 0 for now, gets updated when any set speed command is called
	setSpeedL=0
	setSpeedR=0

	if True:
		while breakFlag is not True:

			time.sleep(updateRate)

			if pidL.SetPoint==0 and pidR.SetPoint==0:
				utils.setSpeedsPWM (1.5,1.495)
			else:
				speedL, speedR=utils.getSpeeds()
				if pidL.enable==True and pidR.enable==True:
					pidL.update(speedL)
					setSpeedL=pidL.output+utils.setSpeedL

					pidR.update(speedR)
					setSpeedR=pidR.output+utils.setSpeedR

					#print(pidL.output,pidR.output)
					#print(utils.getSpeeds(),setSpeedL,setSpeedR)
					utils.setSpeedsIPS(setSpeedL,setSpeedR,False)
	print("PID thread ended")

# Beginning of main function for task1.py
if __name__ == "__main__":
	global breakFlag
	global pidL, pidR

	breakFlag=False
	signal.signal(signal.SIGINT, ctrlC)

	#initialize motors/encoders, PID is initialized with values inputted
	utils.initEncoders()
	utils.initMotors()
	utils.initPID(0.6,0.1,0.00)
	utils.setSpeedsPWM (1.5,1.495)
	imu=utils.initIMU()

	#Grab initialized pid objects
	pidL=utils.pidL
	pidR=utils.pidR

	#Getting PID thread ready
	threadPID = threading.Thread(target=loopPID, args=(1,))

	print("Starting")
	threadPID.start()

	#Body code here-start
	H=25
	W=25
	V=3
	Y=42

	distance=2*(H+W)
	distance=distance+(8*math.pi)
	if V<(distance/Y):
		print("Can not perform rectangle maneuver at {} speed in {} seconds".format(V,Y))
	else:
		print("Performing rectangle maneuver at {} speed in {} seconds".format(V,Y))
		start=time.time()

		utils.moveXV(H/2,V)
		utils.rotateA(90)


		utils.moveXV(W,V)
		utils.rotateA(90)


		utils.moveXV(H,V)
		utils.rotateA(90)


		utils.moveXV(W,V)
		utils.rotateA(90)


		utils.moveXV(H/2,V)

		print("Finished rectangle maneuver in {} seconds".format(time.time()-start))
	#Body code here -end

	#Cleanup
	breakFlag=True
	threadPID.join()
	print("Exiting")
	utils.setSpeedsPWM (1.5,1.495)
	GPIO.cleanup()
	exit()
