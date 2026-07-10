import typing

import py_trees

from protocols.command import SupportsCommand


class OutOfBoundsBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

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
                self.command.send_command(0, -300, 0, 1)
                return py_trees.common.Status.RUNNING
            case 2:
                self.feedback_message = "Going forwards"
                self.command.send_command(0, 300, 0, 1)
                return py_trees.common.Status.RUNNING
            case 3:
                self.feedback_message = "Going forwards right"
                self.command.send_command(200, -200, 0, 1)
                return py_trees.common.Status.RUNNING
            case 4:
                self.feedback_message = "Going left"
                self.command.send_command(0, 300, 0, 1)
                return py_trees.common.Status.RUNNING
            case 5:
                self.feedback_message = "Going backwards"
                self.command.send_command(-300, 0, 0, 1)
                return py_trees.common.Status.RUNNING
            case 6:
                self.feedback_message = "Going forwards left"
                self.command.send_command(200, 200, 0, 1)
                return py_trees.common.Status.RUNNING

        if line_status != 0:
            self.feedback_message = "Invalid line sensor state"
        return py_trees.common.Status.FAILURE
