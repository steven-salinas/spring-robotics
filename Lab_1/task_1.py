# This program demonstrates usage of the servos.
# Keep the robot in a safe location before running this program,
# as it will immediately begin moving.
# See https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/ for more details.

import time
import Adafruit_PCA9685
import signal
import math

MIN_PWM = 1.3
MAX_PWM = 1.7
MAX_VELOCITY = (13/6)*math.pi
MAX_RADIANS = (5/3)*math.pi
MAX_RPS = 5/6
D_MID = 2
r = 1.3

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    
    # Stop the servos
    pwm.set_pwm(LSERVO, 0, 0)
    pwm.set_pwm(RSERVO, 0, 0)
    
    exit()

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Write an initial value of 1.5, which keeps the servos stopped.
# Due to how servos work, and the design of the Adafruit library, 
# the value must be divided by 20 and multiplied by 4096.
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

#Set the speeds of the servo motor using given pulse width
#Formula for wheel angular velocity given pwm is 'w = 26.18p - 39.27' 1.3 <= p <= 1.7
def setSpeedsPWM(pwm_left, pwm_right):
	pwm.set_pwm(LSERVO, 0, math.floor(pwm_left/20 * 4096))
	pwm.set_pwm(RSERVO, 0, math.floor(pwm_right/20 * 4096))

#Set the speed of servo motors in rotation per second of robotbull wheels
#Formula for rps given pwm is 'rps = 4.167p - 6.25'
def setSpeedsRPS(rpsLeft,rpsRight):
	#first have to calculate pwm for given rps
	pwm_left = (rpsLeft + 6.25) / 4.167
	pwm_right = (rpsRight + 6.25) / 4.167
	setSpeedsPWM(pwm_left,pwm_right)

#Set speed of servo motors in inches per second
#Formula to convert linear velocity given pwm is 'v = 34.04p - 51.05'
#ipsLeft - inches per second of left servo motor where left is looking from above while robot is moving forward
#ipsRight - inches per second of right servo motor where right is looking from above while robot is moving forward
def setSpeedsIPS(ipsLeft,ipsRight):
	#Conversion from linear velocity to pwm
	pwm_left = (ipsLeft + 51.05) / 34.04
	pwm_right = (ipsRight + 51.05) / 34.04
	setSpeedsPWM(pwm_left,pwm_right)

#Set the spped of the robot given linear speed and angular velocity 
def setSpeedsVW(linear_velocity, angular_velocity, moving_time = 5):
	if angular_velocity == 0: 
		moveXV(12,linear_velocity)
		exit()

	#Solve for radius of circle with [v=wR] = [R=vw]
	radius = abs(linear_velocity/angular_velocity)

	print(radius)
	if(angular_velocity < 0): 	#Turning counter-clockwise
		#Calculate vl and vr with [v = w(R +/- dmid)]
		v_left = angular_velocity * (radius + D_MID)
		v_right = angular_velocity * (radius - D_MID)
	else:	#Turning clockwise
		#Calculate vl and vr with [v = w(R +/- dmid)]
		v_left = angular_velocity * (radius - D_MID)
		v_right = angular_velocity * (radius + D_MID)

	#check if passing max velocity
	if v_left > MAX_VELOCITY or v_right > MAX_VELOCITY :
		print("Robot can not go that fast!")
		exit()
	
	if(angular_velocity > 0):
		setSpeedsIPS(-v_left,v_right)
		print("Left Wheel Linear: ", v_left, "Right Wheel Linear: ", v_right)
		time.sleep(moving_time)
		pwm.set_pwm(LSERVO, 0, 0)
		pwm.set_pwm(RSERVO, 0, 0)
	else:
		setSpeedsIPS(v_left, -v_right)
		print("Left Wheel Linear: ", v_left, "Right Wheel Linear: ", v_right)
		time.sleep(moving_time)
		pwm.set_pwm(LSERVO, 0, 0)
		pwm.set_pwm(RSERVO, 0, 0)		

#Move robot X inches at a linear velocity of V inches per second
def moveXV(X,V):
	if(V > MAX_VELOCITY):
		print("Robot can not go that fast!")
		exit()
	if X < 0:
		V = 0 - V

	#Calculate time using [v = d/t] = [t = d/v]
	time_on = X/V
	print("Running Time: ", time_on)
	setSpeedsIPS(-V,V)
	time.sleep(abs(time_on))
	pwm.set_pwm(LSERVO, 0, 0)
	pwm.set_pwm(RSERVO, 0, 0)

#moveXV(5,2)
#setSpeedsVW(3,1)
#setSpeedsVW(.5,-3)
#exit()
