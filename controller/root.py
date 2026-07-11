import py_trees

from behaviours.BallChaseBehaviour import BallChaseBehaviour
from behaviours.CameraBehaviour import CameraBehaviour
from behaviours.GoForwardsBehaviour import GoForwardsBehaviour
from behaviours.OutOfBoundsBehaviour import OutOfBoundsBehaviour
from behaviours.StopGoBehaviour import StopGoBehaviour


def create_root():
    """
    Creates a behaviour tree for controlling the state of a single robot.

    Resources:
    - https://arxiv.org/pdf/1709.00084
    - https://py-trees-ros-tutorials.readthedocs.io/en/devel/tutorials.html#before-we-start
    - https://py-trees.readthedocs.io/en/devel/trees.html

    This is kept in its own method so it can be used to generate diagrams with py-trees-render
    """

    parallel_root = py_trees.composites.Parallel(
        name="RoboCup Controller",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    # camera always runs
    camera = CameraBehaviour("Camera")
    selector = py_trees.composites.Selector(
        name="Movement", memory=False
    )  # selector for different movement operations
    parallel_root.add_children([camera, selector])

    # P1: stop/go controls
    stop_go = StopGoBehaviour("Stop/Go")

    # P2: avoid out of bounds
    out_of_bounds = OutOfBoundsBehaviour("Avoid Out of Bounds")

    # P3: chase the ball
    # ball_chase = BallChaseBehaviour("Ball Chase")
    forwards = GoForwardsBehaviour("Forwards")

    selector.add_children([stop_go, out_of_bounds, forwards])

    return parallel_root
