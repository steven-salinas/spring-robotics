# See https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/ for more details.
import time
import math
from tkinter import W
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/ 
import Adafruit_PCA9685
import signal # README: https://raspberry-projects.com/pi/programming-in-c/signal-handling/signal-function
import task_1
from task_1 import MAX_PWM,MIN_PWM,MAX_VELOCITY,MAX_RADIANS,MAX_RPS,D_MID

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    GPIO.cleanup()
    exit()

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)

######################################################################
# Functions for Encoder on robot
#####################################################################

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

# Class to create object to easily interact with encoder
class Encoders():
    def __init__(self) -> None:
        self.left_ticks = 0
        self.right_ticks = 0
    def reset(self):
        self.left_ticks = 0
        self.right_ticks = 0
    def increase_left(self):
        self.left_ticks = self.left_ticks + 1
    def increase_right(self):
        self.right_ticks = self.right_ticks + 1
    def get_left(self):
        return self.left_ticks
    def get_right(self):
        return self.right_ticks

# Initialize encoder with 0 for tick values
encoder = Encoders()

# Called when left encoder encounters a rising edge signal
def onLeftEncode(pin):
    encoder.increase_left()

# Called when right encoder encounters a rising edge signal
def onRightEncode(pin):
    encoder.increase_right()

# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Set encoder pins as input
# Also enable pull-up resistors on the encoder pins
# This ensures a clean 0V and 3.3V is always outputted from the encoders.
GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Attach rising edge interrupt to the encoder pins
GPIO.add_event_detect(LENCODER,GPIO.RISING, onLeftEncode)
GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

# Resets tick counts in both encoders
def resetCounts():
    encoder.reset()

# Returns the current amount of ticks since last reset or beginning
def getCounts():
    return (encoder.get_left(), encoder.get_right())

# Get the current speed of the robot in in/s
# Returns a tuple as (left_wheel_speed, right_wheel_speed)
def getSpeeds():
    # Get the current count of encoders
    current_left = encoder.get_left()
    current_right = encoder.get_right()
    time.sleep(1)

    # Each tick is pi/16 radians. Multiply ticks over one second by pi/16 to get angular velocity in rad/s
    left_angularv = ((encoder.get_left() - current_left) * math.pi) / 16
    right_angularv = ((encoder.get_right() - current_right) * math.pi) / 16
    
    # Find linear velocity of each wheel by using [v = wr]
    left_linear = left_angularv * 1.3
    right_linear = right_angularv * 1.3
    return (left_linear,right_linear)

# Function to move the robot in a circle of R radius at a linear velocity of V
def turnRV(R:int,V:int): 
    w = V/R
    circumference = 2* math.pi * R
    circle_time = abs(circumference/V)
    print("Circumference:  ", circumference,"Circle Time: ", circle_time)
    task_1.setSpeedsVW(V,w,circle_time)
    print(getCounts())


#turnRV(5,-1)
#exit()