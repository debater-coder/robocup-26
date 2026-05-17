from abc import abstractmethod
from dataclasses import dataclass
from typing import Protocol


@dataclass
class RobotState:
    pose: tuple[float, float, float]
    vel: tuple[float, float, float]


@dataclass
class World:
    """
    State of the world. 3-tuples in form (x, y, w), 2-tuples in (x, y).
    Distances in mm, angles in radians.
    """

    can_move: bool

    ball_pos: tuple[float, float] | None
    ball_vel: tuple[float, float] | None

    self_state: RobotState | None
    teammate_state: RobotState | None

    opponents: list[RobotState]


class SupportsWorld(Protocol):
    """Protocol for receiving the state of the world"""

    @abstractmethod
    def get_world(self) -> World:
        pass
