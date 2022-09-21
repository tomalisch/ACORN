### Main experimental loop for ACORN V2.x

## Dependencies

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


## Initializations

# Set desired assay length in seconds

duration_assay = 60*60

# Set desired Open and Close angle positions for Gate and Exit

gOpenAngle = 10
gCloseAngle = 40
eOpenAngle = 5
eCloseAngle = 40

# Set GPIO servo1 pin as pin p & servo2 pin as pin p2 connected to servo on Raspberry Pi
# Set DHT sensor to pin 7 as DHT_PIN (Note that pins are assigned in BCM mode)
p = 2
p2 = 3
dht_pin = 17

# Initialize pins and suppress warnings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(p, GPIO.OUT)
GPIO.setup(p2, GPIO.OUT)
GPIO.setup(dht_pin, GPIO.IN)

# Setup PWM at pin p & p2 to herz h
h = 50
pwm = GPIO.PWM(p, h)
pwm2 = GPIO.PWM(p2, h)

# Reset pin angles so no servo movement occurs on script load
pwm.start(0)
pwm2.start(0)

# Reset DHT sensor pin by killing associated process
name = "libgpiod_pulsein"    
# iterating through each instance of the process
for line in os.popen("ps ax | grep " + name + " | grep -v grep"):
    fields = line.split()            
    # extracting Process ID from the output
    pid = fields[0]        
    # terminating process
    os.kill(int(pid), signal.SIGKILL)

# Set up temperature and humidity sensor
dht_sensor = adafruit_dht.DHT22(dht_pin)

# Check if sensor works as expected
retry = 1
while retry == 1:
    try:
        temperature = dht_sensor.temperature
        humidity = dht_sensor.humidity
        print( "Temperature = {}C \n Humidity = {}% \n".format(temperature, humidity) )
        retry = 0
    except:
        time.sleep(0.1)
        retry = 1
    

## Functions

def SaveVideo(duration_recording = 60):

    # Set up video save path
    savepath = "/media/pi/ACORN_DATA/Videos/"
    t = datetime.now()
    t_str = t.strftime("%Y-%m-%d_%H-%M-%S")
    temperature_pre, humidity_pre = TakeTempHum()
    videopath = savepath + str(t_str) + ".h264"


    # Set up video recording with set resolution and quality
    print("Starting video capture \n")
    with picamera.PiCamera() as camera:
        camera.resolution = (1280, 720)
        camera.framerate = 10
        time.sleep(0.5)
        camera.start_recording(videopath)
        camera.wait_recording(duration_recording)
        camera.stop_recording()
    print("Video saved as: ", videopath, "\n")
    print("Converting video to mp4 \n")
    temperature_post, humidity_post = TakeTempHum()
    # Compute mean from pre and post temp/hum
    temperature = (temperature_pre + temperature_post)/2
    humidity = (humidity_pre + humidity_post)/2
    convertedpath = savepath + str(t_str) + "_" + str(temperature) + "C" + "_" + str(humidity) + "H" +".mp4"
    command = "MP4Box -add " + videopath + ":fps=10" + " " + convertedpath
    call([command], shell = True)
    print("Converted video as: ", convertedpath, "\n")
    os.remove(videopath)
    print("Removed unconverted video \n")
    

def SavePicture():
    
    # Set picture save path same as video savepath
    savepath = "/media/pi/ACORN_DATA/Videos/"
    
    with picamera.PiCamera() as camera:
        camera.resolution = (1280, 720)
        t = datetime.now()
        t_str = t.strftime("%Y-%m-%d_%H-%M-%S")
        camera.capture(savepath + str(t_str) + ".jpg")
        camera.stop_preview()
        print("Picture saved as: ", str(t_str), ".jpg \n")

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


def OpenGate(servo, angle=gOpenAngle):
    
    #SetAngle(55)
    SetAngle(servo, angle)

def CloseGate(servo, angle=gCloseAngle):
    
    #SetAngle(120)
    SetAngle(servo, angle)

def DetectMotion(total_frames = 30, detection_fps = 10, compensation = 0):
    
    # Motion area sensitivity
    max_area = 2500
    occupied_frames = 0
    corrupted_frames = 0
    current_frame = 0
    
    #Detection area
    detect_minX = round(640 - (640/3))-50
    detect_maxX = round(640 + (640/3))-50
    detect_minY = round(480 - (480/3))-270
    detect_maxY = round(480 + (480/3))-270
    
    def handle_new_frame(frame, past_frame, max_area):
        
        arena_occupied = 0
        corrupted_frame = 0
        arena_motion_detected = 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Apply a black & white filter
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


def TakeTempHum(retries=10):
    retry = 1
    while retry <= retries:
        try:
            temperature = dht_sensor.temperature
            humidity = dht_sensor.humidity
            print( "Temperature = {}C \n Humidity = {}% \n".format(temperature, humidity) )
            # Check that temperature is not defaulting to erroneous values
            if temperature != -50 and humidity != 1:
                if temperature is not None and humidity is not None:
                    retry = retries + 1
            else:
                retry += 1
                # If sensor not working, set output to nonsense to maintain rest of routine
                temperature = 99
                humidity = 0
        except:
            time.sleep(0.1)
            retry += 1
            # If sensor not working, set output to nonsense to maintain rest of routine
            temperature = 99
            humidity = 0
    temperature = round(temperature,2)
    humidity = round(humidity,2)
    return(temperature, humidity)

## MAIN

while True:


    # Check occupancy status, recheck if too many corrupted frames
    corrupted_frames = 30
    total_frames = 30
    while corrupted_frames/total_frames >= 0.25:
        occupied_frames, total_frames, corrupted_frames = DetectMotion(total_frames = 30)
        print(occupied_frames / (total_frames - corrupted_frames))

    # If fewer than 25% of valid frames show motion, consider arena empty
    if occupied_frames / (total_frames - corrupted_frames) <= 0.25:
        print('Arena considered empty')
        
        # Capture control picture for later manual occupancy control (if fly visible in picture -> reject subsequent video)
        SavePicture()
        time.sleep(2)
        # Open gate to allow new fly to enter, making sure exit is closed
        print('Opening arena')
        OpenGate(servo=p)
        time.sleep(2)
        print('Closing exit')
        CloseGate(servo=p2, angle = eCloseAngle)
        time.sleep(10)
        # Try opening again, sometimes first command does not take
        OpenGate(servo=p)
        time.sleep(2)
        CloseGate(servo=p2, angle = eCloseAngle)
        time.sleep(2)
        
        # Check whether fly has entered
        occupied_frames = 0
        corrupted_frames = 0
        total_frames = 30
        tspent = 0 # Approximately ~10s
        while occupied_frames / (total_frames - corrupted_frames) <= 0.25 or corrupted_frames/total_frames >= 0.25:
            print('Waiting for fly to enter...')
            tspent += 1
            occupied_frames, total_frames, corrupted_frames = DetectMotion(total_frames = 30)
            # If spent too long without detecting fly, open gate again to make sure it's open
            if tspent >= 200:
                        OpenGate(servo=p)
                        time.sleep(2)
                        tspent = 150
                        print('No fly detected for too long - attempting to reopen Gate...')
        
        print('Arena considered occupied')
        tspent = 0
        
        # Close gate and start assay (send close command to exit gate as well to be sure arena is closed)
        print('Closing gates and starting assay')
        CloseGate(servo=p)
        time.sleep(2)
        CloseGate(servo=p2, angle = eCloseAngle)
        time.sleep(2)        
        CloseGate(servo=p)
        time.sleep(2)
        CloseGate(servo=p2, angle = eCloseAngle)
        time.sleep(2)
        
        # Record video
        SaveVideo(duration_recording = duration_assay)
        
        # Allow fly to exit through exit gate after assay
        print('Opening exit gate')
        OpenGate(servo=p2, angle = eOpenAngle)
        # Wait 60 seconds to allow fly to exit
        time.sleep(60)
        
        # Check if fly exited
        print('Checking whether fly exited arena...')
        occupied_frames = 30
        corrupted_frames = 0
        total_frames = 30
        while occupied_frames / (total_frames - corrupted_frames) >= 0.25 or corrupted_frames/total_frames >= 0.25:
            occupied_frames, total_frames, corrupted_frames = DetectMotion(total_frames = 30)
        
        # Break out of while loop if no motion detected any more after allowing fly to finish exiting
        time.sleep(30)
        CloseGate(servo=p2, angle = eCloseAngle)
        time.sleep(2)
        print('Fly exited after assay: ready for next fly')
 
    # If more than 25% of valid frames show motion, loop over motion detection with gate open
    elif occupied_frames / (total_frames - corrupted_frames) > 0.25:
        print('Arena not considered empty: retrying evacuation')
        
        # Open exit gate to allow old fly to exit
        OpenGate(servo=p2, angle = eOpenAngle)
        time.sleep(30)
        
        # Check if fly exited
        occupied_frames = 30
        corrupted_frames = 0
        total_frames = 30
        while occupied_frames / (total_frames - corrupted_frames) >= 0.25 or corrupted_frames/total_frames >= 0.25:
            occupied_frames, total_frames, corrupted_frames = DetectMotion(total_frames = 30)
        
        # Break out of while loop if no motion detected any more
        CloseGate(servo=p2, angle = eCloseAngle)
        time.sleep(2)
        print('Fly exited after assay: ready for next fly')
        
    

## Cleanup

# Reset GPIO
pwm.stop()
GPIO.cleanup()