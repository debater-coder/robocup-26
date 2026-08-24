import py_trees

from behaviours import (
    CameraBehaviour,
    ChaseBehaviour,
    ChaseBehaviourSimple,
    GetGoalTargetBehaviour,
    HasPossessionBehaviour,
    OutOfBoundsBehaviour,
    StopGoBehaviour,
)


def create_attempt_goal_root():
    has_possession = HasPossessionBehaviour("Has Posession?")
    get_target = GetGoalTargetBehaviour("Get Goal Target")
    goal_chase = ChaseBehaviourSimple(
        "Goal Chase", remap_to={"/target": "/goal/target"}
    )

    attempt_goal = py_trees.composites.Sequence(
        "Attempt Goal", memory=False, children=[has_possession, get_target, goal_chase]
    )
    return attempt_goal


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

    # P3: attempt goal
    attempt_goal = create_attempt_goal_root()

    # P4: chase the ball
    ball_chase = ChaseBehaviour("Ball Chase", remap_to={"/target": "/ball"})

    # Movement selector
    movement = py_trees.composites.Selector(
        name="Movement",
        memory=False,
        children=[stop_go, out_of_bounds, attempt_goal, ball_chase],
    )

    # Root
    parallel_root = py_trees.composites.Parallel(
        name="RoboCup Controller",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[camera, movement],
    )
    return parallel_root
