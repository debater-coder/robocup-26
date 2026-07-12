import py_trees


class GetGoalTargetBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name)

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(
            "/goal/bb",
            access=py_trees.common.Access.READ,
            required=True,
        )
        self.blackboard.register_key(
            "/goal/target",
            access=py_trees.common.Access.WRITE,
        )

    def update(self):
        if self.blackboard.goal.bb:
            x, y, w, h = self.blackboard.goal.bb
            self.blackboard.goal.target = (x + w / 2, y + w / 2)
        else:
            self.blackboard.goal.target = None

        return py_trees.common.Status.SUCCESS
