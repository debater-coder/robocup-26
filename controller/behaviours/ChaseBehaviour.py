import typing

import numpy as np
import py_trees
import rerun as rr
from simple_pid import PID

from protocols.command import SupportsCommand


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

        self.pid_x = PID(3, 0, 0, setpoint=0)
        self.pid_y = PID(3, 0, 0, setpoint=0)
        self.pid_theta = PID(-10, 0, -5, setpoint=0, output_limits=(-20, 20))

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
            x = target[0]
            y = target[1]
            theta = np.arctan2(target[0], target[1])

            # Update errors
            # vel_x = -self.pid_x(x)
            vel_x = 0
            # vel_y = -self.pid_y(y)
            vel_y = 0
            vel_theta = self.pid_theta(theta)

            rr.log("/chase/theta/measured", rr.Scalars(theta))
            rr.log("/chase/theta/p", rr.Scalars(self.pid_theta.components[0]))
            rr.log("/chase/theta/i", rr.Scalars(self.pid_theta.components[1]))
            rr.log("/chase/theta/d", rr.Scalars(self.pid_theta.components[2]))

            velocity = np.array([vel_x, vel_y])

            # Normalise velocity vector length
            vel_length = np.linalg.norm(velocity)
            if vel_length != 0:
                velocity *= min(vel_length, 300) / vel_length

            rr.log(
                "/camera/world/velocity",
                rr.Arrows3D(vectors=[velocity[0], velocity[1], 0]),
            )

            # X-forwards coordinates
            self.command.send_command(velocity[1], velocity[0], vel_theta, 1)

        else:
            self.command.send_command(0, 0, 10, 1)
            self.feedback_message = "looking for target"
        return py_trees.common.Status.RUNNING
