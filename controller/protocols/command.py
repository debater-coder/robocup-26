from abc import abstractmethod
from typing import Protocol, runtime_checkable


class CommandFailedError(Exception):
    pass


@runtime_checkable
class SupportsCommand(Protocol):
    """Protocol for controlling robot via velocity commands."""

    @abstractmethod
    def send_command(self, vx: float, vy: float, vw: float, dribbler: float) -> None:
        """Sends a velocity command to the robot (relative to the robot).

        Arguments:
        vx -- velocity in the x direction (+ve = forwards) (mm/s)
        vy -- velocity in the y direction (+ve = left) (mm/s)
        vw -- angular velocity (+ve = anticlockwise) (rad/s)
        dribbler -- -1 to 1
        """
        raise NotImplementedError

    @abstractmethod
    def get_odometry(self) -> tuple[float, float, float]:
        """Returns the current odometry of the robot.

        The odometry gives the total displacement from an initial position. You
        should not assume this initial position directly, rather store offsets
        of different known odometry and compute differences between them.
        Odometry is prone to drift, so this should be supplemented by data from
        vision, and used primarily to smooth in between camera frames or to
        control velocity.

        Return value (tuple[x, y, w]):
        x -- odometry in the x direction in m (+ve = forwards)
        y -- odometry n the y direction in m (+ve = left)
        w -- relative angle in radians (+ve = anticlockwise)
        """
        raise NotImplementedError
