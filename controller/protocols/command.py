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

    @abstractmethod
    def get_ball_tof(self) -> int | None:
        """
        Returns the TOF from ball sensor as integer or None if not available
        """
        raise NotImplementedError

    @abstractmethod
    def get_line_status(self) -> int:
        r"""
        Returns the line status from the IR line sensors. Represented as a 3-bit
        integer (with LSB being bit 0):
            - bit 0: left_ir
            - bit 1: back_ir
            - bit 2: right_ir

        A 1 bit represent line detected, while a 0 bit means no line detected.
        Having the bits packed this way allows for easy matching of the 8
        possible line detection cases (6 different line orientations, 1 for no
        line, and 1 for invalid).

        ```
        FRONT OF THE ROBOT
           1----5----4
            \       /
             3     6
              \   /
                2
        ```
        Combinations of two sensors give a virtual sensor at the midpoint as shown.
        """
        raise NotImplementedError
