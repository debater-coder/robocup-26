import serial
from cobs import cobs
import time

ser = serial.Serial("/dev/ttyACM0")


def send_command(freq: int):
    ser.write(b"\0" + cobs.encode(freq.to_bytes(4, "big", signed=False)) + b"\0")
    ser.flush()


while True:
    freq = int(input("Frequency (Hz): "))
    send_command(freq)
