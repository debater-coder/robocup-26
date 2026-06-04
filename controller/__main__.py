import argparse
import sys

import py_trees
import rerun as rr

from root import create_root

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

rr.init("controller")
rr.connect_grpc()

args = parser.parse_args()

root = create_root()

if args.render:
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
    sys.exit()


def post_tick(tree):
    rr.log("blackboard", rr.TextDocument(py_trees.display.unicode_blackboard()))


tree = py_trees.trees.BehaviourTree(root)
print("setup start")
tree.setup()
print("setup done")

tree.tick_tock(period_ms=50, post_tick_handler=post_tick, number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK)
