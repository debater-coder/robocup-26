import argparse

import numpy as np
import requests

parser = argparse.ArgumentParser()

parser.add_argument("-t", "--team")
parser.add_argument("-i", "--index")

args = parser.parse_args()

team = args.team
index = args.index

base_url = "http://localhost:3000"

session = requests.post(
    f"{base_url}/session/register",
    json={"team": team, "robot_index": int(index)},
).json()

print("session:", session)

tick = session["next_tick"]
robot_id = session["robot_id"]


def world_to_local(vel, rotation):
    z: complex = (vel[0] + vel[1] * 1j) / (rotation[0] + rotation[1] * 1j)
    return np.array([z.real, z.imag])


while True:
    world = requests.get(
        f"{base_url}/world/tick", params={"tick": tick, "robot_id": robot_id}
    ).json()

    command = None

    if world["can_move"]:
        ball_pos = np.array(world["ball"]["pose"]["translation"])
        self_pos = np.array(world["self"]["pose"]["translation"])

        vel = ball_pos - self_pos

        vel = vel / np.linalg.norm(vel) * 0.2
        vel = world_to_local(vel, np.array(world["self"]["pose"]["rotation"]))

        command = {"vx": vel[0], "vy": vel[1], "omega": 0}

    requests.post(
        f"{base_url}/robot/command",
        json={"robot_id": robot_id, "tick": tick, "command": command},
    )

    tick += 1
