import py_trees

from behaviours.CameraBehaviour import CameraBehaviour
from behaviours.ChaseBehaviour import ChaseBehaviour
from behaviours.GetGoalTargetBehaviour import GetGoalTargetBehaviour
from behaviours.HasPossessionBehaviour import HasPossessionBehaviour
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

    # Data gathering
    camera = CameraBehaviour("Camera")

    # P1: stop/go controls
    stop_go = StopGoBehaviour("Stop/Go")

    # P2: avoid out of bounds
    out_of_bounds = OutOfBoundsBehaviour("Avoid Out of Bounds")

    # P3: try to score a goal
    has_possession = HasPossessionBehaviour("Has Posession?")
    get_target = GetGoalTargetBehaviour("Get Goal Target")
    goal_chase = ChaseBehaviour("Goal Chase", remap_to={"/target": "/goal/target"})
    score_goal = py_trees.composites.Sequence(
        "Score Goal", memory=False, children=[has_possession, get_target, goal_chase]
    )

    # P4: chase the ball
    ball_chase = ChaseBehaviour("Ball Chase", remap_to={"/target": "/ball/centre"})

    # Movement selector
    movement = py_trees.composites.Selector(
        name="Movement",
        memory=False,
        children=[stop_go, out_of_bounds, score_goal, ball_chase],
    )

    # Root
    parallel_root = py_trees.composites.Parallel(
        name="RoboCup Controller",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[camera, movement],
    )
    return parallel_root
