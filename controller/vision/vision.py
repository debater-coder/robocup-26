import json
import os
import typing
from dataclasses import dataclass

import cv2
import numpy as np
import rerun as rr
from cv2.typing import MatLike
from scipy.spatial.transform import Rotation as R

CAMERA_HEIGHT = 120  # mm
CAMERA_ELEVATION_ANGLE = np.radians(15)  # down = +ve

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

new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    camera_matrix,
    dist_coeffs,
    (calib_w, calib_h),
    np.eye(3),
    balance=1,  # undistorts so that entire frame fits in the remapped frame
)
inverse_matrix = np.linalg.inv(new_camera_matrix)
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
ground_matrix = rotation.as_matrix() @ inverse_matrix
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
    points = np.hstack([arr, np.ones((len(arr), 1))])  # make into 3D arrays (X, Y, 1)
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
    ball_centre: tuple[float, float] | None
    ball_radius: float | None
    goal_bb: tuple[float, float, float, float] | None


def log_image(path: str, frame: MatLike, idx):
    if idx % 5 == 0:
        rr.log(path, rr.Image(frame).compress(jpeg_quality=50))


def process_frame(frame: MatLike, go_to_cyan: bool, frame_idx=0):
    goal_lower = CYAN_LOWER if go_to_cyan else YELLOW_LOWER
    goal_upper = CYAN_UPPER if go_to_cyan else YELLOW_UPPER

    frame = cv2.remap(
        frame,
        map1,
        map2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )

    log_image("/camera/image", frame, frame_idx)

    # to stop negative cutoff
    translation = np.array(
        [
            [
                1,
                0,
                1500,
            ],
            [
                0,
                1,
                1500,
            ],
            [0, 0, 1],
        ],
        dtype=np.float32,
    )
    world_image = cv2.warpPerspective(frame, translation @ ground_matrix, (3000, 3000))
    log_image("/camera/world/image", world_image, frame_idx)
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
    ).reshape(-1, 2, 2)

    rr.log("/camera/lines/lines", rr.LineStrips2D(lines))

    projected_lines = transform_array(lines.reshape(-1, 2)).reshape(-1, 2, 2)
    rr.log(
        "/camera/world",
        rr.Transform3D(
            translation=[0, 0, -1],
        ),
    )
    rr.log("/camera/world/lines", rr.LineStrips2D(projected_lines))

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
