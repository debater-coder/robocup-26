import json
import os
import threading
import typing
from dataclasses import dataclass

import cv2
import numpy as np
import rerun as rr
from cv2.typing import MatLike
from scipy.spatial.transform import Rotation as R

CAMERA_HEIGHT = 145  # mm
CAMERA_ELEVATION_ANGLE = np.radians(15)  # down = +ve

ORANGE_LOWER = np.array([0, 160, 120])
ORANGE_UPPER = np.array([15, 255, 255])

YELLOW_LOWER = np.array([20, 100, 120])
YELLOW_UPPER = np.array([35, 255, 255])

CYAN_LOWER = np.array([85, 50, 50])
CYAN_UPPER = np.array([130, 255, 255])

WHITE_LOWER = np.array([0, 0, 230])
WHITE_UPPER = np.array([180, 30, 255])

config_path = os.path.join(
    os.path.dirname(__file__), "../config", "camera_calibration.json"
)

with open(config_path, "r") as f:
    calib_data = json.load(f)

camera_matrix = np.array(calib_data["camera_matrix"], dtype=np.float64)
dist_coeffs = np.array(calib_data["distortion_coefficients"], dtype=np.float64)
calib_w, calib_h = calib_data["resolution"]

new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    camera_matrix,
    dist_coeffs,
    (calib_w, calib_h),
    np.eye(3),
    balance=1,  # undistorts so that entire frame fits in the remapped frame
)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K=camera_matrix,
    D=dist_coeffs,
    R=np.eye(3),
    P=new_camera_matrix,
    size=(calib_w, calib_h),
    m1type=cv2.CV_16SC2,
)
rotation = R.from_euler("x", -CAMERA_ELEVATION_ANGLE)

# This is a homography matrix, the last element gets scaled to one after matrix multiplication

# vector gets converted to normalised coordinates, then rotated downwards
ground_matrix = rotation.as_matrix()
ground_matrix = (
    np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]]) @ ground_matrix
)  # reshuffle the vector so it has X-Z first and Y is used to scale so it intersects with a ground plane

ground_matrix = (
    np.diag([CAMERA_HEIGHT, CAMERA_HEIGHT, 1]) @ ground_matrix
)  # By scaling up X, Z by CAMERA_HEIGHT, has the effect of finding X and Z values such that Y = CAMERA_HEIGHT while only extending the ray, i.e
# X *= CAMERA_HEIGHT / Y
# Z *= CAMERA_HEIGHT / Y


def transform_array(arr):
    """Transforms an array of points of shape (N, 2) from image coordinates to plane coordinates (in mm)"""
    points = cv2.fisheye.undistortPoints(
        arr.reshape(-1, 1, 2).astype(np.float64), camera_matrix, dist_coeffs
    ).reshape((-1, 2))
    points = np.hstack(
        [points, np.ones((len(arr), 1))]
    )  # make into 3D arrays (X, Y, 1)
    projected = (
        points @ ground_matrix.T
    )  # project every **row** as if it was a column vector

    valid = (
        projected[:, 2] > 0
    )  # Don't project the points backwards through the camera onto the plane

    plane_coords = np.full((len(arr), 2), np.nan)  # Initialise NaN plane coords
    # Renormalise so that it is actually intersecting with that plane
    plane_coords[valid] = (
        projected[valid, :2] / projected[valid, 2:3]
    )  # Numpy boolean masks are very cool

    return plane_coords


@dataclass
class VisionInfo:
    ball: tuple[float, float] | None
    goal: tuple[float, float] | None


def log_image(path: str, frame: MatLike, idx):
    if idx % 5 == 0:
        rr.log(path, rr.Image(frame).compress(jpeg_quality=50))


def process_frame(frame: MatLike, go_to_cyan: bool, frame_idx=0):
    goal_lower = CYAN_LOWER if go_to_cyan else YELLOW_LOWER
    goal_upper = CYAN_UPPER if go_to_cyan else YELLOW_UPPER

    log_image("/camera/image", frame, frame_idx)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

    # Ball
    mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    log_image("/camera/ball/mask", mask, frame_idx)
    mask = cv2.erode(mask, typing.cast(MatLike, None), iterations=2)
    mask = cv2.dilate(mask, typing.cast(MatLike, None), iterations=2)
    contours = list(
        cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    )

    contours.sort(key=cv2.contourArea, reverse=True)

    ball_centre = None
    for contour in contours:
        ((x, y), radius) = cv2.minEnclosingCircle(contour)
        rr.log(
            "/camera/ball/circle",
            rr.Ellipses2D(half_sizes=[(radius, radius)], centers=[(x, y)]),
        )

        pt = transform_array(np.array([[x, y]]))[0]

        if np.isnan(pt).any() or np.linalg.norm(pt) > 3000:  # filter out too far away
            continue

        ball_centre = tuple(pt)

        rr.log(
            "/camera/world/ball",
            rr.Points3D(positions=[(ball_centre[0], ball_centre[1], 0)]),
        )

    if ball_centre is None:
        rr.log("/camera/ball/circle", rr.Clear(recursive=True))
        rr.log("/camera/world/ball", rr.Clear(recursive=True))

    # White lines
    mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    log_image("/camera/lines/mask", mask, frame_idx)
    edges = cv2.Canny(mask, 50, 200)
    log_image("/camera/lines/edges", edges, frame_idx)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=50
    )
    if lines is not None:
        lines = lines.reshape(-1, 2, 2)

        rr.log("/camera/lines/lines", rr.LineStrips2D(lines))

        projected_lines = (
            np.hstack(
                [transform_array(lines.reshape(-1, 2)), np.zeros((len(lines) * 2, 1))]
            )
            .reshape(-1, 2, 3)
            .astype(np.float32)
        )
        rr.log("/camera/world/lines", rr.LineStrips3D(projected_lines))
    else:
        rr.log("/camera/lines", rr.Clear(recursive=True))
        rr.log("/camera/world/lines", rr.Clear(recursive=True))

    # Goal
    mask = cv2.inRange(hsv, goal_lower, goal_upper)
    log_image("/camera/goal/mask", mask, frame_idx)
    mask = cv2.erode(mask, typing.cast(MatLike, None), iterations=2)
    mask = cv2.dilate(mask, typing.cast(MatLike, None), iterations=2)
    contours = list(
        cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    )

    contours.sort(key=cv2.contourArea, reverse=True)

    goal_centre = None

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        pt = (x + w / 2, y + h)
        rr.log("/camera/goal/candidate_point", rr.Points2D(positions=[pt]))
        rr.log(
            "/camera/goal/candidate_bb",
            rr.Boxes2D(mins=[(x, y)], sizes=[(w, h)]),
        )
        pt = transform_array(np.array([[pt[0], pt[1]]]))[0]

        if np.isnan(pt).any():
            rr.log("/camera/log", rr.TextLog("Goal detected out of range!"))
            continue

        goal_centre = tuple(pt)

        rr.log(
            "/camera/world/goal",
            rr.Points3D(positions=[goal_centre[1], goal_centre[0], 0]),
        )

    if goal_centre is None:
        rr.log("/camera/goal/bb", rr.Clear(recursive=True))
        rr.log("/camera/world/goal", rr.Clear(recursive=True))
    return VisionInfo(ball=ball_centre, goal=goal_centre)


# Vision testing repl
if __name__ == "__main__":
    rr.init("vision-test")
    rr.connect_grpc()

    # drop into repl
    import code

    code.interact(local=locals())
