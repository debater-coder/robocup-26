import cv2
import numpy as np
from picamera2 import Picamera2

height = 480
width = 640

orangeLower = (0, 100, 20)
orangeUpper = (20, 255, 255)

cam = Picamera2()

cam.configure(
    cam.create_video_configuration(main={"format": "XRGB8888", "size": (width, height)})
)
cam.start()

while True:
    frame = cam.capture_array()

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )[0]

    if len(contours) > 0:
        contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 0, 255), 5)

    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    cv2.waitKey(1)
