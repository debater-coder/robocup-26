import math
import multiprocessing

import rerun as rr
import serial
import serial.tools.list_ports
from cobs import cobs

from protocols.command import SupportsCommand
from serialdebug import task


class CommandFailedError(Exception):
    pass


class LineStatusDebounce:
    def __init__(
        self,
        debounce_ticks=3,
    ):
        self.debounce_ticks = debounce_ticks
        self.seen_for = 0

    def tick(self, x: int) -> int:
        if x == 0:
            self.seen_for = 0
            return 0
        self.seen_for += 1
        if self.seen_for > self.debounce_ticks:
            return x
        return 0


class PicoCommand(SupportsCommand):
    def __init__(self):
        self.ser = self.init_serial()
        self.command = (0, 0, 0, 0)

        self.res = [0, 0, 0, 0, 0]
        self.line_status_debounce = LineStatusDebounce()

    def init_serial(self):
        ports = [
            p.device for p in serial.tools.list_ports.comports() if "ttyACM" in p.device
        ]
        ports.sort()
        return serial.Serial(ports[0], timeout=1, write_timeout=1)

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
                    # odom x
                    int.from_bytes(b, "big", signed=True)
                    if len(b := response[:4]) == 4
                    else None,
                    # odom y
                    int.from_bytes(b, "big", signed=True)
                    if len(b := response[4:8]) == 4
                    else None,
                    # odom w
                    int.from_bytes(b, "big", signed=True)
                    if len(b := response[8:12]) == 4
                    else None,
                    # tof
                    int.from_bytes(b, "big", signed=False)
                    if len(b := response[12:14]) == 2
                    else None,
                    # line sensors
                    int.from_bytes(b, "big", signed=False)
                    if len(b := response[14:15]) == 1
                    else None,
                )
            print("No response received, retrying...")

        raise CommandFailedError("Failed to receive command response.")

    def send_command(self, vx: float, vy: float, vw: float, dribbler: float) -> None:
        """Sends a velocity command to the robot (relative to the robot).

        Arguments:
        vx -- velocity in the x direction (+ve = forwards) (mm/s)
        vy -- velocity in the y direction (+ve = left) (mm/s)
        vw -- angular velocity (+ve = anticlockwise) (rad/s)
        dribbler -- -1 to 1 (+ve is attractive)
        """
        self.command = (int(vx), int(vy), int(math.degrees(vw)), -int(dribbler * 100))
        try:
            self.res = [
                new if new is not None else old
                for old, new in zip(self.res, self.send_packet(self.command))
            ]
        except:
            try:
                self.ser.close()
            except:
                pass

            self.ser = self.init_serial()

        rr.log("/pico/command/vx", rr.Scalars(vx))
        rr.log("/pico/command/vy", rr.Scalars(vy))
        rr.log("/pico/command/vw", rr.Scalars(vw))
        rr.log("/pico/command/dribbler", rr.Scalars(dribbler))

        if tof := self.res[3]:
            rr.log("/pico/tof", rr.Scalars(tof))
        else:
            rr.log("/pico/tof", rr.Clear(recursive=True))

        rr.log("/pico/line_status", rr.Scalars(self.res[4] or 0))

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
        return None if self.res[3] == 0 else self.res[3]

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
        return self.line_status_debounce.tick(self.res[4] or 0)
