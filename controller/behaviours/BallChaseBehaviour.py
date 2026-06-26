import typing

import py_trees

from protocols.command import SupportsCommand


class BallChaseBehaviour(py_trees.behaviour.Behaviour):
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
                "Expected command to be passed in BallChaseBehaviour.setup()"
            )

    def update(self):
        if (centre := self.blackboard.ball_centre) and (
            radius := self.blackboard.ball_radius
        ):
            if centre[0] > 300:
                self.command.send_command(100, 4, 0, 1)
                self.feedback_message = "turning right"
            elif centre[0] < 180:
                self.command.send_command(100, -0.1, 0, 1)
                self.feedback_message = "forwards"
            else:
                self.command.send_command(100, 0, 0, 1)
                self.feedback_message = "turning left"

        else:
            self.command.send_command(0, 0, 10, 0)
            self.feedback_message = "looking for ball"
        return py_trees.common.Status.RUNNING
