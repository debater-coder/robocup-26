import argparse
import sys

import py_trees
import rerun as rr

from root import create_root

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
rr.script_add_args(parser)

args = parser.parse_args()

root = create_root()

if args.render:
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
    sys.exit()

rr.script_setup(args, "robocup")


def post_tick(tree):
    rr.log("blackboard", rr.TextDocument(py_trees.display.unicode_blackboard()))


root = create_root()

tree = py_trees.trees.BehaviourTree(root)
tree.setup(timeout=5)

tree.tick_tock(period_ms=16, post_tick_handler=post_tick)

rr.script_teardown(args)
