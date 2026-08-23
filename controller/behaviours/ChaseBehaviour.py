import typing

import numpy as np
import py_trees
import rerun as rr

from protocols.command import SupportsCommand

K_p = 3


class ChaseBehaviour(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        remap_to: dict[str, str] = {
            "/target": "/target",
            "/camera/shape": "/camera/shape",
        },
    ):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(
            "/target",
            access=py_trees.common.Access.READ,
            required=True,
            remap_to=remap_to.get("/target"),
        )

        self.blackboard.register_key(
            "/camera/shape",
            access=py_trees.common.Access.READ,
            required=True,
            remap_to=remap_to.get("/camera/shape"),
        )

    def setup(self, **kwargs: typing.Any) -> None:
        if "command" in kwargs and isinstance(
            command := kwargs["command"], SupportsCommand
        ):
            self.command = command
        else:
            raise TypeError("Expected command to be passed in ChaseBehaviour.setup()")

    def update(self):
        target = self.blackboard.target
        if target is not None:
            self.feedback_message = "chasing target"
            velocity = K_p * np.array(target)
            theta = np.arctan2(target[1], target[0])
            vel_length = np.linalg.norm(velocity)

            if vel_length != 0:
                velocity *= min(vel_length, 300) / vel_length

            rr.log(
                "/camera/world/velocity",
                rr.Arrows3D(vectors=[velocity[0], velocity[1], 0]),
            )

            self.command.send_command(0, 0, theta, 1)

        else:
            self.command.send_command(0, 0, 10, 1)
            self.feedback_message = "looking for target"
        return py_trees.common.Status.RUNNING
