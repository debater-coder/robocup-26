import typing

import py_trees

from protocols.command import SupportsCommand


class KickBallBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        if "command" in kwargs and isinstance(
            command := kwargs["command"], SupportsCommand
        ):
            self.command = command
        else:
            raise TypeError(
                "Expected command to be passed in KickBallBehaviour.setup()"
            )

    def update(self):
        self.command.send_command(0, 0, 0, -1)

        return py_trees.common.Status.SUCCESS
