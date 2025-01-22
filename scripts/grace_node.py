#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


from typing import List, Union

import rospy
from grace.msg import RobotState


# TODO: Change parent_object and child_object to work with YOLO objects only
class RobotGoal:
    """A RobotGoal consists of a `parent_object` and a `child_object`. The `parent_object` is the destination to place the `child_object`.

    An example is with a coffee cup and a table, the `parent_object` would be the table and the `child_object` is the coffee cup.
    """

    parent_object: str
    """The object to place the `child_object` on. The table in a coffee cup/table relationship."""
    child_object: str
    """The object to place on a `parent_object`. The coffee cup in a coffee cup/table relationship."""

    def __init__(self, parent_object: str, child_object: str) -> None:
        self.parent_object = parent_object
        self.child_object = child_object


_valid_states: List[Union[int, float]] = [
    value
    for key, value in RobotState.__dict__.items()
    if not key.startswith("_")
    and not callable(value)
    and isinstance(value, (int, float))
]


class GraceNode:
    def __init__(self) -> None:
        self._state: int
        self.goal: RobotGoal # TODO: Implement a RobotGoal msg

        # TODO: Add a user-friendly input to change the state and goal.
        self._state = RobotState.WAITING

        self.state_publisher = rospy.Publisher(
            name="/state", data_class=RobotState, queue_size=5
        )

        # BUG: Goal cannot be a class, it has to be a msg.
        # self.goal_publisher = rospy.Publisher(name="/goal", data_class=RobotGoal)
        self.state_subscriber = rospy.Subscriber(
            "/state", data_class=RobotState, callback=self.state_callback
        )
        # self.goal_subscriber = rospy.Subscriber(
        #     "/goal", data_class=RobotGoal, callback=self.goal_callback
        # )

        self.state = self._state  # Run the guard function in the setter
        self.state_publisher.publish(self.state)

        self.goal = RobotGoal(parent_object="table", child_object="cup")
        # self.goal_publisher.publish(self.goal)

    def __call__(self) -> None:
        self.run()

    def __repr__(self) -> str:
        return f"GraceNode(state='{self.state}', goal='{self.goal}')"

    def run(self) -> None:
        rospy.spin()

    def state_callback(self, msg: RobotState) -> None:
        self.state = msg.state

    def goal_callback(self, msg: RobotGoal) -> None:
        self.goal = msg

    @property
    def state(self) -> int:
        """The state of the turtlebot.

        Returns:
            int: A value within msg/RobotState.msg's constants.
        """
        return self._state

    @state.setter
    def state(self, new_state: int) -> None:
        if new_state not in _valid_states:
            raise ValueError(
                f"Invalid state {new_state}. It is not a defined state in RobotState.msg"
            )
        self._state = new_state


if __name__ == "__main__":
    try:
        rospy.init_node(name="GraceNode")  # type: ignore
        grace = GraceNode()
        grace.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("GraceNode terminated.")  # type: ignore
