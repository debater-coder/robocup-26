from machine import UART, Pin

uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

while True:
    uart.write(uart.readline())
