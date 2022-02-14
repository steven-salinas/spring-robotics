import time
import Adafruit_PCA9685
import signal
import math
import RPi.GPIO as GPIO # README: https://pypi.org/project/RPi.GPIO/
import keyboard

def getCounts():
    global left_count, right_count
    return (left_count,right_count)

def resetCounts():
    global left_count
    global right_count
    left_count=0
    right_count=0

def ctrlC(signum, frame):
    print("Exiting")

    # Stop the servos
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    GPIO.cleanup()

    exit()

def onLeftEncode(pin):
    global left_count
    left_count=left_count+1
    # print("Left encoder ticked!")

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global right_count
    right_count=right_count+1
# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)

LSERVO = 0
RSERVO = 1

LENCODER = 17
RENCODER = 18

pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)


GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

left_count=0
right_count=0



current_val=1.5

while True:
    x=input()
    if x == "w":
        current_val=current_val+0.01

    if x == "s":
        current_val=current_val-0.01

    print("PWM: {}".format(current_val))
    pwm.set_pwm(LSERVO, 0, math.floor(current_val / 20 * 4096));
    pwm.set_pwm(RSERVO, 0, math.floor(current_val / 20 * 4096));
    resetCounts()
    time.sleep(2)
    x=getCounts()
    print(x)
