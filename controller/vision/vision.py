import json
import os
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

config_path = os.path.join(
    os.path.dirname(__file__), "../config", "camera_calibration.json"
)

with open(config_path, "r") as f:
    calib_data = json.load(f)

camera_matrix = np.array(calib_data["camera_matrix"], dtype=np.float64)
dist_coeffs = np.array(calib_data["distortion_coefficients"], dtype=np.float64)
calib_w, calib_h = calib_data["resolution"]


@dataclass
class VisionInfo:
    ball_centre: tuple[float, float] | None
    ball_radius: float | None
    goal_bb: tuple[float, float, float, float] | None


def log_image(path: str, frame: MatLike, idx):
    if idx % 5 == 0:
        rr.log(path, rr.Image(frame).compress(jpeg_quality=50))


def process_frame(frame: MatLike, go_to_cyan: bool, frame_idx=0):
    goal_lower = CYAN_LOWER if go_to_cyan else YELLOW_LOWER
    goal_upper = CYAN_UPPER if go_to_cyan else YELLOW_UPPER

    k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        camera_matrix, dist_coeffs, (calib_w, calib_h), np.eye(3), balance=1
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K=camera_matrix,
        D=dist_coeffs,
        R=np.eye(3),
        P=k,
        size=(calib_w, calib_h),
        m1type=cv2.CV_16SC2,
    )

    frame = cv2.remap(
        frame,
        map1,
        map2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )

    log_image("/camera/image", frame, frame_idx)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

    # Ball
    mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    log_image("/camera/ball/mask", mask, frame_idx)
    mask = cv2.erode(mask, typing.cast(MatLike, None), iterations=2)
    mask = cv2.dilate(mask, typing.cast(MatLike, None), iterations=2)
    contours = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )[0]

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

    # White lines
    mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    log_image("/camera/lines/mask", mask, frame_idx)
    edges = cv2.Canny(mask, 50, 200)
    log_image("/camera/lines/edges", edges, frame_idx)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=50
    )
    rr.log("/camera/lines/lines", rr.LineStrips2D(lines.reshape(-1, 2, 2)))

    # Goal
    mask = cv2.inRange(hsv, goal_lower, goal_upper)
    log_image("/camera/goal/mask", mask, frame_idx)
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
