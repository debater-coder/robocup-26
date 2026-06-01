"""
Refer to:
https://stackoverflow.com/questions/66353952/how-to-pass-video-stream-from-one-python-script-to-another
for shared memory explanation
"""

import argparse
from multiprocessing.shared_memory import SharedMemory
from time import sleep

import numpy as np
import rerun as rr

rr.init("camtest")
rr.connect_grpc()

# Frame Shape
frame_shape_shm = SharedMemory(name="frame_shape")
frame_shape = np.ndarray([3], buffer=frame_shape_shm.buf, dtype="i4")

# Framebuffer
frame_buffer_shm = SharedMemory(name="frame_buffer")
frame_buffer = np.ndarray(frame_shape, buffer=frame_buffer_shm.buf, dtype="u1")

try:
    while True:
        rr.log("/camera/image", rr.Image(frame_buffer).compress())
        sleep(0.05)
finally:
    # cleanup: IMPORTANT the writer process should close before this one, so nothing
    #  tries to access the shm after unlink() is called. (less important on windows)
    frame_buffer_shm.close()
    frame_buffer_shm.unlink()
    frame_shape_shm.close()
    frame_shape_shm.unlink()
