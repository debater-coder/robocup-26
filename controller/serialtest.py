from time import sleep

import serial
from cobs import cobs

ser = serial.Serial("/dev/ttyACM0", timeout=1, write_timeout=1)


class CommandFailedError(Exception):
    pass


def read_cobs_packet(ser: serial.Serial):
    buf = bytearray()

    while True:
        b = ser.read(1)
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


def send_command(ser: serial.Serial, freq: int):
    for i in range(5):
        try:
            ser.write(
                b"\0" + cobs.encode(freq.to_bytes(4, "big", signed=False)) + b"\0"
            )
        except serial.SerialTimeoutException:
            continue
        ser.flush()
        response = read_cobs_packet(ser)

        if response:
            return int.from_bytes(response, "big", signed=True)
        print("No response received, retrying...")

    raise CommandFailedError("Failed to receive command response.")


while True:
    period = send_command(ser, 5)
    print(f"New odom: {period} mm")
    sleep(0.2)
