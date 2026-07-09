import typing

import gpiozero
import py_trees
from gpiozero import Button


class StopGoBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

    def setup(self, **kwargs: typing.Any) -> None:
        if "pin_factory" in kwargs and isinstance(
            pin_factory := kwargs["pin_factory"], gpiozero.Factory
        ):
            self.button = Button(26, pin_factory=pin_factory)
        else:
            raise TypeError(
                "Expected pin_factory to be passed in StopGoBehaviour.setup()"
            )

    def update(self):
        if self.button.is_active:
            self.feedback_message = "GO"
            return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "STOP"
            return py_trees.common.Status.SUCCESS
