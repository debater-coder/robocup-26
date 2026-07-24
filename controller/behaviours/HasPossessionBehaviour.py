import typing

import py_trees

from protocols.command import SupportsCommand


class HasPossessionBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        if "command" in kwargs and isinstance(
            command := kwargs["command"], SupportsCommand
        ):
            self.command = command
        else:
            raise TypeError(
                "Expected command to be passed in HasPossessionBehaviour.setup()"
            )

    def update(self):
        tof = self.command.get_ball_tof()

        if tof is not None and tof < 200:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE
