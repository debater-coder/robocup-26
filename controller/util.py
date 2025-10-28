"""
Utility from https://www.zair.top/en/post/hsv-color-tool/
"""

import cv2
import numpy as np
from picamera2 import Picamera2

height = 480
width = 640

cam = Picamera2()

cam.configure(
    cam.create_video_configuration(main={"format": "XRGB8888", "size": (width, height)})
)
cam.start()


# Callback function for the sliders to get the value at the slider position
def empty(a):
    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
    print(h_min, h_max, s_min, s_max, v_min, v_max)
    return h_min, h_max, s_min, s_max, v_min, v_max


# Create a window to place 6 sliders
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 240)
cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
cv2.createTrackbar("Hue Max", "TrackBars", 19, 179, empty)
cv2.createTrackbar("Sat Min", "TrackBars", 110, 255, empty)
cv2.createTrackbar("Sat Max", "TrackBars", 240, 255, empty)
cv2.createTrackbar("Val Min", "TrackBars", 153, 255, empty)
cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

while True:
    img = cam.captureArray()
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Call the callback function to get the slider values
    h_min, h_max, s_min, s_max, v_min, v_max = empty(0)
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    # Obtain a mask within the specified color range
    mask = cv2.inRange(imgHSV, lower, upper)
    # Perform bitwise AND operation on the original image, keeping the mask area
    imgResult = cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", imgResult)
    cv2.waitKey(1)
