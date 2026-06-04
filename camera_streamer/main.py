"""
Refer to:
https://stackoverflow.com/questions/66353952/how-to-pass-video-stream-from-one-python-script-to-another
for shared memory explanation
"""

from multiprocessing.shared_memory import SharedMemory
from time import sleep

import numpy as np
from picamera2 import Picamera2

cam = Picamera2()
cam.start()
frame = np.rot90(cam.capture_array(), k=2, axes=(0, 1))


# Shared memory for the frame dimensions
frame_shape_shm = SharedMemory(name="frame_shape", create=True, size=frame.ndim * 4)
frame_shape = np.ndarray(3, buffer=frame_shape_shm.buf, dtype="i4")
frame_shape[:] = frame.shape

# Shared memory for the frame itself
frame_buffer_shm = SharedMemory(name="frame_buffer", create=True, size=frame.nbytes)
frame_buffer = np.ndarray(frame_shape, buffer=frame_buffer_shm.buf, dtype=frame.dtype)


try:
    while True:
        frame_buffer[:] = np.rot90(
            cam.capture_array(), k=3, axes=(0, 1)
        )  # in place updating of the buffer
        sleep(0.01)
finally:
    frame_buffer_shm.close()
    frame_shape_shm.close()
