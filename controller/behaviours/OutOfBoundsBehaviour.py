import time
import typing

import py_trees

from protocols.command import SupportsCommand


class OutOfBoundsBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)
        self.timeout = None

    def setup(self, **kwargs: typing.Any) -> None:
        if "command" in kwargs and isinstance(
            command := kwargs["command"], SupportsCommand
        ):
            self.command = command
        else:
            raise TypeError(
                "Expected command to be passed in OutOfBoundsBehaviour.setup()"
            )

    def update(self):
        line_status = self.command.get_line_status()
        match line_status:
            case 1:
                self.feedback_message = "Going right"
                self.timeout = (time.time(), (0, -100, 0, 1))
            case 2:
                self.feedback_message = "Going forwards"
                self.timeout = (time.time(), (0, 100, 0, 1))
            case 3:
                self.feedback_message = "Going forwards right"
                self.timeout = (time.time(), (100, -100, 0, 1))
            case 4:
                self.feedback_message = "Going left"
                self.timeout = (time.time(), (0, 100, 0, 1))
            case 5:
                self.feedback_message = "Going backwards"
                self.timeout = (time.time(), (-100, 0, 0, 1))
            case 6:
                self.feedback_message = "Going forwards left"
                self.timeout = (time.time(), (100, 100, 0, 1))

        if (timeout := self.timeout) and time.time() - timeout[0] < 2:
            self.command.send_command(*timeout[1])
            return py_trees.common.Status.RUNNING

        if line_status != 0:
            self.feedback_message = "Invalid line sensor state"

        self.feedback_message = "No line detected"
        return py_trees.common.Status.FAILURE
