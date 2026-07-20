import argparse
import sys
from datetime import datetime

import py_trees
import rerun as rr

from root import create_root
from services.PicoCommand import PicoCommand

print("Started.")

parser = argparse.ArgumentParser(
    prog="Robocup controller",
    description="Controls a single Raspberry Pi to execute Robocup strategy.",
)
parser.add_argument(
    "-r", "--render", action="store_true", help="Renders the behaviour tree"
)
parser.add_argument(
    "-s", "--sim", action="store_true", help="Connect to simulator server"
)
parser.add_argument(
    "-i", "--recording-id", help="Rerun recording ID (keep same for combined recording)"
)

parser.add_argument(
    "-S",
    "--stream",
    action="store_true",
    help="Stream to grpc server (instead of storing in recordings folder)",
)

args = parser.parse_args()

root = create_root()

if args.render:
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
    sys.exit()

rr.init("controller")
if args.stream:
    rr.connect_grpc()
else:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    rr.save(f"recordings/{timestamp}.rrd")


def post_tick(tree):
    rr.log(
        "blackboard",
        rr.TextDocument(
            py_trees.display.ascii_blackboard()
            + "\n-----------------------------\n"
            + py_trees.display.ascii_tree(root, show_status=True),
        ),
    )


tree = py_trees.trees.BehaviourTree(root)
print("setup start")
tree.setup(command=PicoCommand())
print("setup done")

tree.tick_tock(
    period_ms=25,
    post_tick_handler=post_tick,
    number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
)
