import re

import rerun as rr
import serial

rr.init("serial_debug")
rr.connect_grpc()

ser = serial.Serial("/dev/ttyACM1")
while True:
    line = ser.readline().decode()
    rr.log("serial_logs", rr.TextLog(line))

    if found := re.search(r"\[MEASURED_SPEED_(\d+)\]:\s*(-?\d+)", line):
        rr.log(f"{found.group(1)}/measured_speed", rr.Scalars(int(found.group(2))))

    if found := re.search(r"\[SETPOINT_SPEED_(\d+)\]:\s*(-?\d+)", line):
        rr.log(f"{found.group(1)}/setpoint_speed", rr.Scalars(int(found.group(2))))

    if found := re.search(r"\[CONTROL_OUT_(\d+)\]:\s*(-?\d+)", line):
        rr.log(f"{found.group(1)}/control_out", rr.Scalars(int(found.group(2))))
    if found := re.search(r"\[PID_OUT_(\d+)\]:\s*(-?\d+)", line):
        rr.log(f"{found.group(1)}/pid_out", rr.Scalars(int(found.group(2))))
