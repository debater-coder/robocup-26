import typing
from multiprocessing.shared_memory import SharedMemory
from time import sleep

import cv2
import numpy as np
import py_trees
import rerun as rr
from cv2.typing import MatLike

ORANGE_LOWER = np.array([0, 160, 120])
ORANGE_UPPER = np.array([20, 255, 255])


class CameraBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Camera Behaviour")

        self.blackboard.register_key("ball_centre", access=py_trees.common.Access.WRITE)

        self.blackboard.register_key("ball_radius", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        print("setup")
        # Frame Shape
        self.frame_shape_shm = SharedMemory(name="frame_shape")
        self.frame_shape = np.ndarray([3], buffer=self.frame_shape_shm.buf, dtype="i4")

        # Framebuffer
        self.frame_buffer_shm = SharedMemory(name="frame_buffer")

        self.frame_buffer = np.ndarray(
            self.frame_shape, buffer=self.frame_buffer_shm.buf, dtype="u1"
        )

    def initialise(self): ...

    def update(self):
        frame = self.frame_buffer[:, :, :3]
        rr.log("/camera/image", rr.Image(frame).compress())

        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
        rr.log("/camera/mask", rr.Image(mask).compress())
        mask = cv2.erode(mask, typing.cast(MatLike, None), iterations=2)
        mask = cv2.dilate(mask, typing.cast(MatLike, None), iterations=2)
        contours = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[0]

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            rr.log(
                "/camera/ball_circle",
                rr.Ellipses2D(half_sizes=[(radius, radius)], centers=[(x, y)]),
            )

            self.blackboard.ball_centre = (x, y)
            self.blackboard.ball_radius = radius

        else:
            rr.log("/camera/ball_circle", rr.Clear(recursive=True))
            self.blackboard.ball_centre = None
            self.blackboard.ball_radius = None

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status): ...
