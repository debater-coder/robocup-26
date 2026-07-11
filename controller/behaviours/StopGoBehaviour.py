import typing

import py_trees
from gpiozero import Button


class StopGoBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        self.button = Button(26)

    def update(self):
        if not self.button.is_active:
            self.feedback_message = "GO"
            return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "STOP"
            return py_trees.common.Status.RUNNING
