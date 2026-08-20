import signal
import subprocess
import time

import zmq
from gpiozero import LED, PWMLED, Button

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("ipc://@ui")

# GPIOs
encoder_button = Button(19)
stop_go_switch = Button(26)

# LEDs
ready = PWMLED(2)
ready.pulse()
cyan_goal = LED(27)

# State
go_to_cyan = False
go = False

last_pressed = 0


def handle_goal_change():
    global go_to_cyan, last_pressed
    last_pressed = time.time()
    go_to_cyan = not go_to_cyan
    if go_to_cyan:
        socket.send_multipart([b"goal_change", b"cyan"])
        cyan_goal.on()
    else:
        socket.send_multipart([b"goal_change", b"yellow"])
        cyan_goal.off()


def handle_encoder_release():
    if time.time() - last_pressed > 10:
        subprocess.run(["sudo", "shutdown", "-h", "now"])


def handle_go_change():
    global go
    go = not go
    if not stop_go_switch.is_active:
        socket.send_multipart([b"go_change", b"go"])
    else:
        socket.send_multipart([b"go_change", b"stop"])


encoder_button.when_activated = handle_goal_change
encoder_button.when_deactivated = handle_encoder_release


stop_go_switch.when_activated = handle_go_change
stop_go_switch.when_deactivated = handle_go_change

signal.pause()
