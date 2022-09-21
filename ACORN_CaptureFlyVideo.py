import io
import picamera
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
import time

# Set recording duration in seconds
duration_recording = 60
# Set up video save path
savepath = "/media/pi/ACORN_THUMB/Videos/"
videopath = savepath + str(time.time()) + ".h264"

# Set up video recording with set resolution and quality
print("Starting video capture \n")
with picamera.PiCamera() as camera:
    camera.resolution = (1280, 960)
    camera.framerate = 30
    camera.start_recording(videopath)
    camera.wait_recording(duration_recording)
    camera.stop_recording()
print("Video saved \n")