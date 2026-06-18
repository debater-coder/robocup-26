import py_trees

from behaviours.BallChaseBehaviour import BallChaseBehaviour
from behaviours.CameraBehaviour import CameraBehaviour


def create_root():
    """
    Creates a behaviour tree for controlling the state of a single robot.

    Resources:
    - https://arxiv.org/pdf/1709.00084
    - https://py-trees-ros-tutorials.readthedocs.io/en/devel/tutorials.html#before-we-start
    - https://py-trees.readthedocs.io/en/devel/trees.html

    This is kept in its own method so it can be used to generate diagrams with py-trees-render
    """

    root = py_trees.composites.Parallel(
        name="Robocup Controller", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    camera = CameraBehaviour("Camera")
    ball_chase = BallChaseBehaviour("Ball Chase")

    root.add_children([camera, ball_chase])

    return root
