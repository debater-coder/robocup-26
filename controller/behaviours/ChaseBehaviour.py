import typing

import py_trees

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

    def setup(self, **kwargs: typing.Any) -> None:
        if "command" in kwargs and isinstance(
            command := kwargs["command"], SupportsCommand
        ):
            self.command = command
        else:
            raise TypeError("Expected command to be passed in ChaseBehaviour.setup()")

    def update(self):
        w, h = self.blackboard.camera.shape
        if centre := self.blackboard.target:
            if centre[0] > w * 0.6:
                self.command.send_command(200, 0, 4, 1)
                self.feedback_message = "turning right"
            elif centre[0] < w * 0.4:
                self.command.send_command(200, 0, -4, 1)
                self.feedback_message = "turning left"
            else:
                self.command.send_command(300, 0, 0, 1)
                self.feedback_message = "forwards"
        else:
            self.command.send_command(0, 0, 10, 1)
            self.feedback_message = "looking for target"
        return py_trees.common.Status.RUNNING
