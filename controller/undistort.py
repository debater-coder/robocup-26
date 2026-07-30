import json
import os
from multiprocessing.shared_memory import SharedMemory
from time import sleep

import cv2 as cv
import numpy as np
import rerun as rr

rr.init("undistort")
rr.connect_grpc()

config_path = os.path.join(
    os.path.dirname(__file__), "config", "camera_calibration.json"
)

with open(config_path, "r") as f:
    calib_data = json.load(f)

camera_matrix = np.array(calib_data["camera_matrix"], dtype=np.float64)
dist_coeffs = np.array(calib_data["distortion_coefficients"], dtype=np.float64)
calib_w, calib_h = calib_data["resolution"]

# === connect to shared memory camera streamer ===
frame_shape_shm = SharedMemory(name="frame_shape", track=False)
frame_shape = np.ndarray([3], buffer=frame_shape_shm.buf, dtype="i4")

# Framebuffer
frame_buffer_shm = SharedMemory(name="frame_buffer", track=False)

frame_buffer = np.ndarray(frame_shape, buffer=frame_buffer_shm.buf, dtype="u1")

while True:
    sleep(0.02)
    frame = frame_buffer[:, :, :3]
    h, w = frame.shape[:2]

    if (w, h) != (calib_w, calib_h):
        print("Warning: Image resolution does not match calibration resolution!")

    new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), alpha=1, newImgSize=(w, h)
    )

    undistorted_img = cv.undistort(
        frame, camera_matrix, dist_coeffs, None, new_camera_matrix
    )

    rr.log("frame", rr.Image(undistorted_img).compress(jpeg_quality=50))
