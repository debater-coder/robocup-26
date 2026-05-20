import time

import numpy as np
import numpy.typing as npt
import serial

from serialtest import send_command

ser = serial.Serial("/dev/ttyACM0", timeout=1, write_timeout=1)

# Move in a square

odom = np.array(send_command(ser, [0, 0, 0, 0]))


def go_to_setpoint(setpoint: npt.ArrayLike):
    global odom
    setpoint = np.asarray(setpoint)
    error = setpoint[:2] - odom[:2]
    command_pos = error / np.linalg.norm(error) * 100
    angle_error = setpoint[2] - odom[2]

    print(odom)

    odom = np.array(
        send_command(
            ser, [int(command_pos[0]), int(command_pos[1]), -int(angle_error * 10), 0]
        )
    )

    return np.linalg.norm(error)


while go_to_setpoint([300, 0, 0]) > 10:
    time.sleep(0.05)

while go_to_setpoint([300, 300, 0]) > 10:
    time.sleep(0.05)

while go_to_setpoint([0, 300, 0]) > 10:
    time.sleep(0.05)

while go_to_setpoint([0, 0, 0]) > 10:
    time.sleep(0.05)

print(send_command(ser, [0, 0, 0, 0]))
