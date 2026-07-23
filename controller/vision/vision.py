import typing
from dataclasses import dataclass

import cv2
import numpy as np
import rerun as rr
from cv2.typing import MatLike

ORANGE_LOWER = np.array([0, 160, 120])
ORANGE_UPPER = np.array([15, 255, 255])

YELLOW_LOWER = np.array([20, 100, 120])
YELLOW_UPPER = np.array([35, 255, 255])

CYAN_LOWER = np.array([85, 50, 50])
CYAN_UPPER = np.array([130, 255, 255])

WHITE_LOWER = np.array([0, 0, 180])
WHITE_UPPER = np.array([180, 30, 255])


@dataclass
class VisionInfo:
    ball_centre: tuple[float, float] | None
    ball_radius: float | None
    goal_bb: tuple[float, float, float, float] | None


def process_frame(frame: MatLike, go_to_cyan: bool):
    goal_lower = CYAN_LOWER if go_to_cyan else YELLOW_LOWER
    goal_upper = CYAN_UPPER if go_to_cyan else YELLOW_UPPER

    rr.log("/camera/image", rr.Image(frame).compress())
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

    # Ball
    mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    rr.log("/camera/ball/mask", rr.Image(mask).compress())
    mask = cv2.erode(mask, typing.cast(MatLike, None), iterations=2)
    mask = cv2.dilate(mask, typing.cast(MatLike, None), iterations=2)
    contours = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )[0]

    # White lines
    mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    rr.log("/camera/lines/mask", rr.Image(mask).compress())

    if len(contours) > 0:
        contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        rr.log(
            "/camera/ball/circle",
            rr.Ellipses2D(half_sizes=[(radius, radius)], centers=[(x, y)]),
        )

        ball_centre = (x, y)
        ball_radius = radius

    else:
        rr.log("/camera/ball/circle", rr.Clear(recursive=True))
        ball_centre = None
        ball_radius = None

    # Goal
    mask = cv2.inRange(hsv, goal_lower, goal_upper)
    rr.log("/camera/goal/mask", rr.Image(mask).compress())
    mask = cv2.erode(mask, typing.cast(MatLike, None), iterations=2)
    mask = cv2.dilate(mask, typing.cast(MatLike, None), iterations=2)
    contours = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )[0]
    if len(contours) > 0:
        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        rr.log(
            "/camera/goal/bb",
            rr.Boxes2D(mins=[(x, y)], sizes=[(w, h)]),
        )
        goal_bb = (x, y, w, h)

    else:
        rr.log("/camera/goal/bb", rr.Clear(recursive=True))
        goal_bb = None

    return VisionInfo(ball_centre=ball_centre, ball_radius=ball_radius, goal_bb=goal_bb)


# Vision testing repl
if __name__ == "__main__":
    rr.init("vision-test")
    rr.connect_grpc()

    # drop into repl
    import code

    code.interact(local=locals())
