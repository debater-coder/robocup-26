import math

import rerun as rr
import serial
from cobs import cobs

from protocols.command import SupportsCommand


class CommandFailedError(Exception):
    pass


class PicoCommand(SupportsCommand):
    def __init__(self):
        self.ser = serial.Serial("/dev/ttyACM0", timeout=0.02, write_timeout=0.02)
        self.command = (0, 0, 0, 0)

        self.res = [0, 0, 0, 0]

    def read_cobs_packet(self):
        buf = bytearray()

        while True:
            b = self.ser.read(1)
            if not b:
                return None

            if b == b"\x00":  # The delimiter byte is not included in packet
                if not buf:
                    continue
                try:
                    return cobs.decode(bytes(buf))
                except cobs.DecodeError:
                    print("COBS decode error, dropping packet")
                    buf.clear()
                    continue
            else:
                buf += b

    def send_packet(self, controls: tuple[int, int, int, int]):
        for i in range(5):
            try:
                self.ser.write(
                    b"\0"
                    + cobs.encode(
                        controls[0].to_bytes(4, "big", signed=True)
                        + controls[1].to_bytes(4, "big", signed=True)
                        + controls[2].to_bytes(4, "big", signed=True)
                        + controls[3].to_bytes(4, "big", signed=True)
                    )
                    + b"\0"
                )
            except serial.SerialTimeoutException:
                continue
            self.ser.flush()
            response = self.read_cobs_packet()

            if response:
                return (
                    int.from_bytes(response[:4], "big", signed=True),
                    int.from_bytes(response[4:8], "big", signed=True),
                    int.from_bytes(response[8:12], "big", signed=True),
                    int.from_bytes(response[12:14], "big", signed=False),
                )
            print("No response received, retrying...")

        raise CommandFailedError("Failed to receive command response.")

    def send_command(self, vx: float, vy: float, vw: float, dribbler: float) -> None:
        """Sends a velocity command to the robot (relative to the robot).

        Arguments:
        vx -- velocity in the x direction (+ve = forwards) (mm/s)
        vy -- velocity in the y direction (+ve = left) (mm/s)
        vw -- angular velocity (+ve = anticlockwise) (rad/s)
        dribbler -- -1 to 1
        """
        self.command = (int(vx), int(vy), int(math.degrees(vw)), int(dribbler * 100))
        self.res = self.send_packet(self.command)
        rr.log("/pico/command/vx", rr.Scalars(vx))
        rr.log("/pico/command/vy", rr.Scalars(vy))
        rr.log("/pico/command/vw", rr.Scalars(vw))
        rr.log("/pico/command/dribbler", rr.Scalars(dribbler))

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
        return (self.res[0], self.res[1], math.radians(self.res[2]))

    def get_ball_tof(self) -> int | None:
        """
        Returns the TOF from ball sensor as integer or None if not available
        """
        tof = None if self.res[3] == 0 else self.res[3]
        if tof:
            rr.log("/pico/tof", rr.Scalars(tof))
        else:
            rr.log("/pico/tof", rr.Clear(recursive=True))

        return tof
