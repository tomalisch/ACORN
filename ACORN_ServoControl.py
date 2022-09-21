import cv2
import os
import time
import io
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
from time import sleep

# Set repeat amounts for testing

repeats = 2

# Set GPIO pin as pin p connected to servo on Raspberry Pi
p = 2
p2 = 3

# Initialize servo pins and suppress warnings
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(p, GPIO.OUT)
GPIO.setup(p2, GPIO.OUT)


# Setup PWM at pin p to herz h
h = 50
pwm = GPIO.PWM(p, h)
pwm2 = GPIO.PWM(p2, h)

# Reset pin angle so no servo movement occurs on script load
pwm.start(0)
pwm2.start(0)

# Define function to set servo angles
def SetAngle(servo, angle):
    
    duty = angle / 18 + 2
    GPIO.output(servo, True)
    if servo == p:
        pwm.ChangeDutyCycle(duty)
    elif servo == p2:
        pwm2.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(servo, False)
    if servo == p:
        pwm.ChangeDutyCycle(0)
    elif servo == p2:
        pwm2.ChangeDutyCycle(0) 
    
# MAIN

# Set open and close angles (Manually set with trial and error)

for rep in range(0, repeats):

    GateOpen = 10 
    GateClose = 40
    ExitOpen = 5
    ExitClose = 40

    SetAngle(p, GateClose)
    time.sleep(2)
    SetAngle(p2, ExitClose)
    time.sleep(2)
    SetAngle(p, GateOpen)
    time.sleep(2)
    SetAngle(p2, ExitOpen)
    time.sleep(5)

# Reset GPIO
pwm.stop()
pwm2.stop()
GPIO.cleanup()