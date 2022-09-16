## ACORN fly tracking script
## Contains functions that
## 1) take continuous images for a specified time and save them in a specified location,
## 2) sequentially open images or videos saved in a specified location and determines the centroid location
## of the largest spatial cluster in the background-subtracted image

# Dependencies
import numpy as np
import matplotlib.pyplot as plt
import cv2
import imutils
import os
import time
import datetime
import skimage


def TrackVideo(fileLocation, saveLocation="/home/pi/Desktop/Tracking/", minArea=25, showTracking=1):

	firstFrame = 0
	capture = cv2.VideoCapture(fileLocation)
	capture.open();
	while(capture.isOpened()):
		[readokay, frame] = capture.read()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)
		if firstFrame == 0:
			firstFrame = gray
			continue

		frameDelta = cv2.absdiff(firstFrame, gray)
		thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
		thresh = cv2.dilate(thresh, None, iterations=2)
		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		for c in cnts:
		# if the contour is too small, ignore it
			if cv2.contourArea(c) < minArea:
				continue

			M = cv2.moments(c)
			cX = int(M["m10"] / M["m00"])
			cY = int(M["m01"] / M["m00"])
			if showTracking == 1:
				cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
				cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)

		if showTracking == 1:
			cv2.imshow('grayscale_blur', frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

	cap.release()
	cv2.destroyAllWindows()
	return