"""
Calibration script modified from: https://docs.opencv.org/4.11.0/dc/dbb/tutorial_py_calibration.html
"""

import json
import os
from multiprocessing.shared_memory import SharedMemory
from time import sleep

import cv2 as cv
import numpy as np
import rerun as rr

rr.init("calibration")
rr.connect_grpc()

# === connect to shared memory camera streamer ===
frame_shape_shm = SharedMemory(name="frame_shape", track=False)
frame_shape = np.ndarray([3], buffer=frame_shape_shm.buf, dtype="i4")

# Framebuffer
frame_buffer_shm = SharedMemory(name="frame_buffer", track=False)

frame_buffer = np.ndarray(frame_shape, buffer=frame_buffer_shm.buf, dtype="u1")

# ===

# termination criteria
CRITERIA = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((10 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:10, 0:7].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.


while len(objpoints) < 20:
    frame = frame_buffer[:, :, :3]
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (10, 7), None)

    # If found, add object points, image points (after refining them)
    if ret:
        # The object points (coordinates of checkerboard points in 3D) remain the same for each frame
        objpoints.append(np.reshape(objp, (1, -1, 3)))

        refined_corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
        imgpoints.append(np.reshape(refined_corners, (-1, 1, 2)))

        print(f"Successfully captured frame {len(objpoints)}/20")
        sleep(0.2)  # wait for new frame
    sleep(0.02)


h, w = gray.shape
camera_matrix = np.zeros((3, 3), dtype=np.float64)
distortion = np.zeros((4, 1), dtype=np.float64)
ret, camera_matrix, distortion, rvecs, tvecs = cv.fisheye.calibrate(
    objpoints,
    imgpoints,
    (w, h),
    K=camera_matrix,
    D=distortion,
    flags=cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv.fisheye.CALIB_FIX_SKEW,
    criteria=(cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
)

print(f"\nCalibration complete: reproj error = {ret}")

output_data = {
    "camera_matrix": camera_matrix.tolist(),
    "distortion_coefficients": distortion.tolist(),
    "resolution": [w, h],
}


os.makedirs(f"{os.path.dirname(__file__)}/config", exist_ok=True)

with open(f"{os.path.dirname(__file__)}/config/camera_calibration.json", "w") as f:
    json.dump(output_data, f, indent=4)
