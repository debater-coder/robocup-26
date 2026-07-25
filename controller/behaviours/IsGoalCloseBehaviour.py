import py_trees


class IsGoalCloseBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(
            "/goal/bb",
            access=py_trees.common.Access.READ,
            required=True,
        )

    def update(self):
        if self.blackboard.goal.bb:
            x, y, w, h = self.blackboard.goal.bb
            return (
                py_trees.common.Status.SUCCESS
                if w > 700 and h > 500
                else py_trees.common.Status.FAILURE
            )

        else:
            return py_trees.common.Status.FAILURE
