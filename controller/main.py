from typing import TYPE_CHECKING, Sequence
import cv2
import numpy as np
from picamera2 import Picamera2

if TYPE_CHECKING:
    from cv2.typing import MatLike

height = 480
width = 640

orangeLower = (5, 50, 50)
orangeUpper = (15, 255, 255)

cam = Picamera2()

cam.configure(
    cam.create_video_configuration(main={"format": "XRGB8888", "size": (width, height)})
)
cam.start()

while True:
    frame = cam.capture_array()

    blurred = cv2.GaussianBlur(frame, (22, 22), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, orangeLower, orangeUpper)
    mask = cv2.erode(mask, None, iteration=2)
    mask = cv2.dilate(mask, None, iteration=2)
    contours: Sequence[MatLike] = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )[0]

    if len(contours) > 0:
        contour = max(contours, key=cv2.contourArea)
        (centre, radius) = cv2.minEnclosingCircle(contour)
        cv2.circle(frame, centre, 5, (0, 0, 255), -1)

    cv2.imshow("f", frame)
    cv2.waitKey(1)
