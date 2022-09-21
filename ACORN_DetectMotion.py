import cv2
import os
import signal
import time
import io
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
from time import sleep
from datetime import datetime
import picamera
import imutils
from subprocess import call
import adafruit_dht


def DetectMotion(total_frames = 600, detection_fps = 10, compensation = 0):
    
    # Motion area sensitivity
    max_area = 2500
    occupied_frames = 0
    corrupted_frames = 0
    current_frame = 0
    
    #Detection area
    detect_minX = round(640 - (640/3))
    detect_maxX = round(640 + (640/3))
    detect_minY = round(480 - (480/3))
    detect_maxY = round(480 + (480/3))
    
    def handle_new_frame(frame, past_frame, max_area):
        
        arena_occupied = 0
        corrupted_frame = 0
        arena_motion_detected = 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Apply a black & white filter
        cv2.rectangle(gray, (detect_minX,detect_minY), (detect_maxX,detect_maxY), 1, 10 )
        cv2.imshow('gray', gray)
        cv2.waitKey(2)
        gray = cv2.GaussianBlur(gray, (5, 5), 0) # Blur the picture

        # if the first frame is None, initialize it because there is no frame for comparing the current one with a previous one
        if past_frame is None:
            past_frame = gray
            return past_frame, arena_occupied, corrupted_frames

        # compute the absolute difference between the current frame and first frame
        frame_delta = cv2.absdiff(past_frame, gray)
        # then apply a threshold to remove camera motion and other false positives (like light changes)
        thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]

        # dilate the thresholded image to fill in holes, then find contours on thresholded image
        thresh = cv2.dilate(thresh, None, iterations=2)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #cnts = cnts[0]
        cnts = imutils.grab_contours(cnts)

        # loop over the contours
        if cnts is not None and len(cnts) <= 20:
            print(len(cnts))
            for c in cnts:
                # Skip checking contours for entire rest of frame if one contour is too big
                # Only continue checking contours if NO motion has yet been detected in arena area
                if corrupted_frame == 0 and arena_motion_detected == 0:
                    # Compute mid point of each contour
                    M = cv2.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                              
                    # if any contour is too large, ignore all contours on that frame
                    if cv2.contourArea(c) > max_area:
                        corrupted_frame = 1
                        arena_occupied = 0
                        print('Large object detected: rejecting detection this frame')
                        #print(cv2.contourArea(c))
                        break
                    
                    # if contour right size, stop checking contours once one contour has been found inside arena area
                    elif cv2.contourArea(c) <= max_area and cX >= detect_minX and cX <= detect_maxX and cY >= detect_minY and cY <= detect_maxY:
                        arena_occupied = 1
                        arena_motion_detected = 1
                        #print(cX, cY)
                        #print(cv2.contourArea(c))
                        #print('Arena is occupied', arena_occupied)
                        break
                        
                    elif cv2.contourArea(c) <= max_area and (cX <= detect_minX or cX >= detect_maxX or cY <= detect_minY or cY >= detect_maxY):
                        arena_occupied = 0
                        #print('Detected motion outside of arena')
                    
        return gray, arena_occupied, corrupted_frame                
        

    with picamera.PiCamera() as camera:
        camera.resolution = (1280, 960)
        camera.framerate = detection_fps
        past_frame = None
        print("Starting motion detection")
        try:
            while current_frame <= total_frames:
                current_frame += 1
                stream = io.BytesIO()
                camera.capture(stream, format='jpeg', use_video_port=False)
                data = np.frombuffer(stream.getvalue(), dtype=np.uint8)
                frame = cv2.imdecode(data, 1)
                if frame is not None:
                    past_frame, arena_occupied, corrupted_frame = handle_new_frame(frame, past_frame, max_area)
                    occupied_frames += arena_occupied
                    corrupted_frames += corrupted_frame
                else:
                    print("No frame to load")
                if compensation == 1:
                    total_frames += corrupted_frame
        finally:
            print("Stopping motion detection: ", occupied_frames, '/', total_frames-corrupted_frames)
            return occupied_frames, total_frames, corrupted_frames


DetectMotion()