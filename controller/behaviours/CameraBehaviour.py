import typing
from multiprocessing.shared_memory import SharedMemory
from time import sleep

import cv2
import numpy as np
import py_trees
import rerun as rr
from cv2.typing import MatLike
from gpiozero import LED, Button

from vision.vision import process_frame


class CameraBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Camera Behaviour")

        self.blackboard.register_key(
            "/ball/centre", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "/ball/radius", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key("/goal/bb", access=py_trees.common.Access.WRITE)

        self.go_to_cyan = False

    def setup(self, **kwargs):
        print("setup")
        # Frame Shape
        self.frame_shape_shm = SharedMemory(name="frame_shape", track=False)
        self.frame_shape = np.ndarray([3], buffer=self.frame_shape_shm.buf, dtype="i4")

        # Framebuffer
        self.frame_buffer_shm = SharedMemory(name="frame_buffer", track=False)

        self.frame_buffer = np.ndarray(
            self.frame_shape, buffer=self.frame_buffer_shm.buf, dtype="u1"
        )
        self.button = Button(19)
        self.led = LED(13)

        def button_handler():
            self.go_to_cyan = not self.go_to_cyan
            if self.go_to_cyan:
                self.led.on()
                self.feedback_message = "scoring cyan"
            else:
                self.led.off()
                self.feedback_message = "scoring yellow"

        self.button.when_activated = button_handler

    def initialise(self): ...

    def update(self):
        frame = self.frame_buffer[:, :, :3]
        info = process_frame(frame, self.go_to_cyan)

        self.blackboard.ball.centre = info.ball_centre
        self.blackboard.ball.radius = info.ball_radius
        self.blackboard.goal.bb = info.goal_bb

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status): ...
