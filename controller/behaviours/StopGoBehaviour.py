import typing

import py_trees
import zmq


class StopGoBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.go = False

    def setup(self, **kwargs: typing.Any) -> None:
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("ipc://@ui")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "go_change")

    def update(self):
        try:
            while True:
                _topic, payload = self.socket.recv_multipart(flags=zmq.NOBLOCK)
                self.go = payload == b"go"
        except zmq.Again:
            pass

        if self.go:
            self.feedback_message = "GO"
            return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "STOP"
            return py_trees.common.Status.RUNNING
