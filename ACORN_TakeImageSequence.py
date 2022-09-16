## ACORN fly tracking script
## Contains functions that
## 1) take continuous images for a specified time and save them in a specified location,
## 2) sequentially open images or videos saved in a specified location and determines the centroid location
## of the largest spatial cluster in the background-subtracted image

# Dependencies
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import matplotlib.pyplot as plt
import cv2
import imutils
import os
import time
import datetime
import skimage

# Takes continuous pictures with fixed camera settings; duration in seconds
def CaptureImageSequence(saveLocation="/home/pi/Desktop/Tracking/", duration=60000, fps=10):
	
	# Set up camera settings and initialize
	camera = PiCamera()
	camera.resolution = (1920, 1088)
	camera.framerate = fps
	# Wait for the automatic gain control to settle
	time.sleep(2)
	# Now fix the values
	camera.shutter_speed = camera.exposure_speed
	camera.exposure_mode = 'off'
	g = camera.awb_gains
	camera.awb_mode = 'off'
	camera.awb_gains = g
	# ISO lower for daylight, higher for low light (100-800 range)
	camera.iso = 400

	# Set up file path
	date = datetime.now().strftime("%Y_%m_%d_%I-%M-%S-%p")
	imagepath = saveLocation + str(date) + "/"
	os.mkdir(imagepath)

	# Start image capture sequence
	stream = picamera.array.PiRGBArray(camera)
	time_start = time.time()
	imagenum = 0
	while time.time() - time_start < duration:
		time_lastimg = time.time()
		camera.capture(stream, format='bgr', use_video_port=True)
		imagenum +=1
		image = stream.array
		image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# Save image to path
		imagename = str(time.time()-time_start) + ".jpg"
		cv2.imwrite(os.path.join(imagepath, imagename), image_gray)
		stream.seek(0)
		stream.truncate()
		# Check if next capture is not earlier than fps dictate
		if time.time() - time_lastimg < 1/fps:
			time.sleep( 1/fps - (time.time() - time_lastimg) )

	return imagepath, imagenum
