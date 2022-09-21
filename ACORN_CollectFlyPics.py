from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import time

# Determine run length in seconds
t_duration = 120*60

# Check current time, add t seconds to it to determine end time
t_end = time.time() + t_duration

# Set up camera resolution
print("initializing camera")
camera = PiCamera()
camera.resolution = (1920, 1088)

# Capture initial background picture to compare subsequent images with
print("capturing background image")
rawCapture = PiRGBArray(camera)
camera.capture(rawCapture, format='rgb')
image_old = rawCapture.array
image_old_gray = cv2.cvtColor(image_old, cv2.COLOR_BGR2GRAY)

# Start main loop for t seconds
print("starting image acquisition")
while time.time() < t_end:
    
    # Capture new picture to evaluate
    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format='rgb')
    image_new = rawCapture.array
    image_new_gray = cv2.cvtColor(image_new, cv2.COLOR_BGR2GRAY)
    
    # Subtract new from old image as the difference image
    image_diff = cv2.absdiff(image_old_gray, image_new_gray)

    # Compute difference score as grand sum of gray pixel value of difference image
    diff_score = np.sum(image_diff)
    if diff_score >= 30000000 or diff_score <= 10000000:
        print("difference detected:", diff_score)
        # Save unblurred new image if significant difference detected
        imagename = str(time.time()) + "_" + str(diff_score) + ".jpg"
        imagepath = "/media/pi/ACORN_THUMB/Images_Drosophila/"
        cv2.imwrite(os.path.join(imagepath, imagename), image_new)
        print("saved image as:", imagepath + imagename)
    
