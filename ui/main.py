import signal

import zmq
from gpiozero import Button

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("ipc://@ui")

# GPIOs
encoder_button = Button(19)
stop_go_switch = Button(26)

# State
go_to_cyan = False
go = False


def handle_goal_change():
    global go_to_cyan
    go_to_cyan = not go_to_cyan
    if go_to_cyan:
        socket.send_multipart([b"goal_change", b"cyan"])
    else:
        socket.send_multipart([b"goal_change", b"yellow"])


def handle_go_change():
    global go
    go = not go
    if not stop_go_switch.is_active:
        socket.send_multipart([b"go_change", b"go"])
    else:
        socket.send_multipart([b"go_change", b"stop"])


encoder_button.when_activated = handle_goal_change
stop_go_switch.when_activated = handle_go_change
stop_go_switch.when_deactivated = handle_go_change

signal.pause()
