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


def send_command(ser: serial.Serial, controls: list[int]):
    for i in range(5):
        try:
            ser.write(
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
        ser.flush()
        response = read_cobs_packet(ser)

        if response:
            return [
                int.from_bytes(response[:4], "big", signed=True),
                int.from_bytes(response[4:8], "big", signed=True),
                int.from_bytes(response[8:12], "big", signed=True),
                int.from_bytes(response[12:14], "big", signed=False),
                int(response[14]),
            ]
        print("No response received, retrying...")

    raise CommandFailedError("Failed to receive command response.")


if __name__ == "__main__":
    while True:
        i = input("Controls (_ _ _ _): ")
        if i == "reset":
            ser.write(b"\0" + cobs.encode("\xff") + b"\0")
            ser.flush()
            continue
        x = list(map(int, i.split(" ")))
        periods = send_command(ser, x)
        print(f"New data: {' '.join([f'{period} mm' for period in periods])}")
