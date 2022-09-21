import picamera
import time
from PIL import Image, ImageDraw, ImageFont
from datetime import datetime

camera = picamera.PiCamera(resolution=(1280, 720))
 
camera.start_preview()
camera.vflip = False
camera.hflip = False
camera.brightness = 60


img = Image.new("RGBA", (1280, 720))
draw = ImageDraw.Draw(img, "RGBA")

#Detection area
detect_minX = round(640 - (640/3))-50
detect_maxX = round(640 + (640/3))-50
detect_minY = round(480 - (480/3))-270
detect_maxY = round(480 + (480/3))-270

draw.rectangle(( (detect_minX,detect_minY), (detect_maxX,detect_maxY) ), fill=(0,0,255,60))

o=camera.add_overlay(img.tobytes(), layer=3, size=img.size, alpha=32);
 
time.sleep(20)
camera.stop_preview()
camera.close()

