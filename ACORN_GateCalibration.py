import picamera
import time
from PIL import Image, ImageDraw, ImageFont
from datetime import datetime
import cv2
import os
import time
import io
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
from time import sleep


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


# Initialize camera

camera = picamera.PiCamera(resolution=(1280, 720))

## Start loop

camera.start_preview()

camera.vflip = False
camera.hflip = False
camera.brightness = 60

# Cycle through servo angles in 5 degree intervals

for GEangle in range(0, 180, 5):

	SetAngle(p, GEangle)
	time.sleep(0.5)
	SetAngle(p2, GEangle)
	time.sleep(2)	

	camera.annotate_size = 120 
	camera.annotate_foreground = Color('black')
	camera.annotate_background = Color('white')
	camera.annotate_text = strcat("Angle = ", str(GEangle) )

## End loop

# Stop camera
time.sleep(1)
camera.stop_preview()
camera.close()

# Reset GPIO
pwm.stop()
pwm2.stop()
GPIO.cleanup()


