from multiprocessing.shared_memory import SharedMemory

import numpy as np
import py_trees
import zmq

from vision.vision import process_frame


class CameraBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Camera Behaviour")

        self.blackboard.register_key("/ball", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "/goal/centre", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "/goal/target", access=py_trees.common.Access.WRITE
        )

        self.blackboard.register_key(
            "/camera/shape", access=py_trees.common.Access.WRITE
        )

        self.go_to_cyan = False

    def setup(self, **kwargs):
        print("setup")
        # Frame Shape
        self.frame_shape_shm = SharedMemory(name="frame_shape", track=False)
        self.frame_shape = np.ndarray([3], buffer=self.frame_shape_shm.buf, dtype="i4")

        self.blackboard.camera.shape = (self.frame_shape[1], self.frame_shape[0])

        # Framebuffer
        self.frame_buffer_shm = SharedMemory(name="frame_buffer", track=False)

        self.frame_buffer = np.ndarray(
            self.frame_shape, buffer=self.frame_buffer_shm.buf, dtype="u1"
        )
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("ipc://@ui")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "goal_change")

    def initialise(self): ...

    def update(self):
        try:
            while True:
                _topic, payload = self.socket.recv_multipart(flags=zmq.NOBLOCK)
                self.go_to_cyan = payload == b"cyan"
        except zmq.Again:
            pass

        frame = self.frame_buffer[:, :, :3]
        info = process_frame(frame, self.go_to_cyan)

        self.blackboard.ball = info.ball
        self.blackboard.goal.centre = info.goal
        self.blackboard.goal.target = info.goal_bb

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status): ...
