import typing

import py_trees

from protocols.command import SupportsCommand


class GoForwardsBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name="Ball Chase")

        self.blackboard.register_key(
            "ball_centre", access=py_trees.common.Access.READ, required=True
        )

        self.blackboard.register_key(
            "ball_radius", access=py_trees.common.Access.READ, required=True
        )

    def setup(self, **kwargs: typing.Any) -> None:
        if "command" in kwargs and isinstance(
            command := kwargs["command"], SupportsCommand
        ):
            self.command = command
        else:
            raise TypeError(
                "Expected command to be passed in GoForwardsBehaviour.setup()"
            )

    def update(self):
        self.command.send_command(200, 0, 0, 1)
        self.feedback_message = "forwards"
        return py_trees.common.Status.RUNNING
