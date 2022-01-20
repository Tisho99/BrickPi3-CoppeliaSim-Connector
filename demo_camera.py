import cv2

import CoppeliaAPI.csimCamera as picamera
from CoppeliaAPI.csimCamera import PiRGBArray

# camera conf
cam = picamera.PiCamera()
cam.resolution = (512, 512)
#cam.framerate = 32

# reference return format
rawCapture = PiRGBArray(cam, size=(512, 512))
try:
    while True:
        # take photo
        cam.capture(rawCapture, format="bgr")
        img = rawCapture.array
        cv2.imshow("image", img)
        cv2.waitKey()
except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    cam.close()        # Unconfigure the sensors, disable the motors.