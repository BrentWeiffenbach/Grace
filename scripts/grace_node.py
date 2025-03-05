#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python


import threading
from typing import Dict, Final, List, Tuple, Union

import rospy
from grace.msg import RobotGoalMsg, RobotState
from move_base_msgs.msg import MoveBaseResult

# Note: from grace_navigation import GraceNavigation is locally imported below


def get_constants_from_msg(msg: type) -> Dict[int, str]:
    """
    The keys and values are accessed by getting all the non-callable int constants from the autogenerated python file.

    Args:
        msg (type): The imported msg class to get the constants from. e.g. for `from grace.msg import RobotGoalMsg`, this would be `RobotGoalMsg`.

    Returns:
        Dict (int, str): The enum value and their corresponding states from `msg`'s autogenerated python file. Follows key/value of int: STATE.
    """
    return {
        value: key
        for key, value in msg.__dict__.items()
        if not key.startswith("_") and not callable(value) and isinstance(value, int)
    }


class RobotGoal:
    """A RobotGoal consists of a `parent_object` and a `child_object`. The `parent_object` is the destination to place the `child_object`.

    An example is with a coffee cup and a table, the `parent_object` would be the table and the `child_object` is the coffee cup.
    """

    classes: List[str]= ["person","bicycle","car","motorcycle","airplane","bus","train",
                         "truck","boat","traffic light","fire hydrant","stop sign",
                         "parking meter","bench","bird","cat","dog","horse","sheep",
                         "cow","elephant","bear","zebra","giraffe","backpack","umbrella",
                         "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
                         "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
                         "bottle","wine glass","cup","fork","knife","spoon","bowl","banana",
                         "apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
                         "donut","cake","chair","couch","potted plant","bed","dining table",
                         "toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone",
                         "microwave","oven","toaster","sink","refrigerator","book","clock",
                         "vase","scissors","teddy bear","hair drier","toothbrush"]  # fmt: skip

    def __init__(self, parent_object: str, child_object: str) -> None:
        """
        Args:
            parent_object (str): The object to place the `child_object` on. Must be a valid YOLO class.
            child_object (str): The object to place on a `parent_object`. Must be a valid YOLO class.
        """
        self.parent_object = parent_object
        self.child_object = child_object

    @property
    def parent_object(self) -> str:
        """The object to place the `child_object` on. The table in a coffee cup/table relationship.

        Returns:
            str: The parent object's class name.
        """
        return self._parent_object

    @property
    def child_object(self) -> str:
        """The object to place on a `parent_object`. The coffee cup in a coffee cup/table relationship.

        Returns:
            str: The child object's class name.
        """
        return self._child_object

    @parent_object.setter
    def parent_object(self, new_parent_object: str) -> None:
        if not RobotGoal.is_valid_class(new_parent_object):
            raise ValueError(
                f"Invalid parent object. {new_parent_object} is not in RobotGoal.classes!"
            )
        self._parent_object = new_parent_object

    @child_object.setter
    def child_object(self, new_child_object: str) -> None:
        if not RobotGoal.is_valid_class(new_child_object):
            raise ValueError(
                f"Invalid child object. {new_child_object} is not in RobotGoal.classes!"
            )
        self._child_object = new_child_object

    @staticmethod
    def is_valid_class(_cls: str) -> bool:
        return _cls in RobotGoal.classes

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, RobotGoal):
            return False
        return (
            self.child_object == other.child_object
            and self.parent_object == other.parent_object
        )

    def __str__(self) -> str:
        return f"Goal: Place {self.child_object} on a {self.parent_object}"

    def __repr__(self) -> str:
        return f"RobotGoal(parent_object={self.parent_object}, child_object={self.child_object})"


_states: Dict[int, str] = get_constants_from_msg(RobotState)
"""The enum value and their corresponding states from msg/RobotState.msg. Follows key/value of int: STATE.

The keys and values are accessed by getting all the non-callable int constants from the autogenerated python file. 
"""


def state_to_str(state: int) -> str:
    """Returns a string that is human-readable for a state int.

    Args:
        state (int): The state to turn human-readable.

    Returns:
        str: The state in a human-readable form.

    Example:
        >>> state_to_str(0)
        WAITING (0)
        >>> state_to_str(RobotState.WAITING)
        WAITING (0)
    """
    return f"{_states.get(state)} ({state})"


class GraceNode:
    """The main node for controlling the turtlebot. Is the link between a (as of January 23, 2025) theoretical frontend and the turtlebot. Send a goal to /grace/goal to kickstart the application.

    Attributes:
        state (int): The state of the turtlebot. Must be a valid constant in `RobotState.msg`.
        goal (RobotGoal | None): The goal of the robot. Can be None.
        verbose (bool, optional): Whether verbose output should be logged. Defaults to False.
        STATE_TOPIC (str, Final): The topic to publish the states to.
        GOAL_TOPIC (str, Final): The topic to publish the goals to.

    Publishers:
        state_publisher (RobotState): Publishes to STATE_TOPIC. Details the current state of GRACE.
        goal_publisher (RobotGoal): Publishes to GOAL_TOPIC. Contains GRACE's parent and child object.

    Subscribers:
        state_subscriber (RobotState): Subscribes to STATE_TOPIC. Used to keep `state` updated.
    """

    verbose: bool = False
    STATE_TOPIC: Final[str] = "/grace/state"
    GOAL_TOPIC: Final[str] = "/grace/goal"

    def __init__(self, verbose: bool = False) -> None:
        """Initializes the GraceNode.

        Args:
            verbose (bool, optional): Whether verbose output should be logged. Defaults to False.
        """
        self._state: int
        self._goal: Union[RobotGoal, None]
        GraceNode.verbose: bool = verbose

        # TODO: Add a user-friendly input to change the goal and display the state.
        self._state = RobotState.WAITING
        self._goal = None
        self.has_object: bool = False

        self.state_publisher = rospy.Publisher(
            name=GraceNode.STATE_TOPIC, data_class=RobotState, queue_size=5
        )

        self.goal_publisher = rospy.Publisher(
            name=GraceNode.GOAL_TOPIC, data_class=RobotGoalMsg, queue_size=5
        )
        self.state_subscriber = rospy.Subscriber(
            GraceNode.STATE_TOPIC, data_class=RobotState, callback=self.change_state
        )
        # self.goal_subscriber = rospy.Subscriber(
        #     GraceNode.GOAL_TOPIC, data_class=RobotGoalMsg, callback=self.goal_callback
        # )

        self.state = self._state  # Run the guard function in the setter
        self.state_publisher.publish(self.state)

        if GraceNode.verbose:
            rospy.loginfo(
                f"GRACE is currently {state_to_str(self.state)}{f'. {self.goal}.' if self.goal else ' with no current goal.'}"
            )

        # Local import to resolve circular importing
        from scripts.grace_navigation import GraceNavigation

        self.grace_navigation = GraceNavigation(
            done_cb=self.done_cb, verbose=GraceNode.verbose
        )

    # region State
    @property
    def state(self) -> int:
        """The state of the turtlebot. The default state is `RobotState.WAITING`.

        Returns:
            int: A value within msg/RobotState.msg's constants.
        """
        return self._state

    @state.setter
    def state(self, new_state: int) -> None:
        if new_state not in _states.keys():
            raise ValueError(
                f"Invalid state {new_state}. It is not a defined state in RobotState.msg"
            )
        self._state = new_state

    def change_state(self, new_state: Union[RobotState, int]) -> None:
        """Used for changing the state of the state machine. Will advance the state machine based on the provided state.
        This is not the `state.setter` because setting the state and setting & advancing the state should be separate.

        Args:
            new_state (RobotState | int): The state to change `self.state` to.
        """
        _temp_state: int = self.state  # Used for verbose logging
        try:
            cleaned_state: int
            if isinstance(new_state, int):
                cleaned_state = new_state
            else:
                cleaned_state = new_state.state
            self.state = cleaned_state
            if GraceNode.verbose:
                rospy.loginfo(
                    f"Changed state from {state_to_str(_temp_state)} to {state_to_str(cleaned_state)}."
                )
            self.execute_state(self.state, _temp_state)
        except ValueError as ve:
            rospy.logerr(f"Error while changing state: {ve}")

    def next_state(self) -> Tuple[int, int]:
        """Determines what `self.state` will be next according to the state machine. Note that this does not change `self.state`.

        * WAITING \3 EXPLORING
        * PICKING \3 HOMING
        * PLACING \3 HOMING
        * EXPLORING \3
            * Arrived at child \3 PICKING
            * Arrived at parent \3 PLACING
        * HOMING \3
            * Object Picked up \3 EXPLORING
            * Object Placed \3 WAITING
        * UNKNOWN \3 WAITING

        Returns:
            tuple (new_state: int, old_state: int): The new state and old state.
        """
        new_state: int
        # BUG: MIGHT cause a race condition when updating from goto
        # Not sure yet...
        if self.state == RobotState.WAITING:
            new_state = RobotState.EXPLORING
        elif self.state == RobotState.EXPLORING:
            new_state = RobotState.PLACING if self.has_object else RobotState.PICKING
        elif self.state == RobotState.PICKING:
            new_state = RobotState.HOMING
        elif self.state == RobotState.PLACING:
            new_state = RobotState.HOMING
        elif self.state == RobotState.HOMING:
            new_state = RobotState.EXPLORING if self.has_object else RobotState.WAITING
        else:
            rospy.logwarn(
                "Unknown state; cannot go to next state. Defaulting to WAITING"
            )
            new_state = RobotState.WAITING

        return new_state, self.state

    # endregion
    # region Goal
    @property
    def goal(self) -> Union[RobotGoal, None]:
        """The goal of the turtlebot. Contains a parent_object and a child_object. See `RobotGoal`'s documentation for details.

        Returns:
            RobotGoal: The turtlebot's goal.

        Usage:
            >>> grace.goal = RobotGoal(parent_object="dining table", child_object="cup") # Set the goal
            >>> grace.publish_goal() # Publish the goal
        """
        return self._goal

    @goal.setter
    def goal(self, new_goal: RobotGoal) -> None:
        _temp_goal: Union[RobotGoal, None] = self.goal
        self._goal = new_goal  # new_goal will be validated within RobotGoal's setters
        if GraceNode.verbose:
            rospy.loginfo(f"Changed goal from {_temp_goal} to {self._goal}.")

    def done_cb(
        self, state: int, result: MoveBaseResult, state_dict: Dict[int, str]
    ) -> None:
        if state_dict[state] == "SUCCEEDED":
            new_state, old_state = self.next_state()
            self.change_state(new_state)
        elif state_dict[state] == "ABORTED":
            rospy.logerr("move_base failed to go to the goal pose!")

    def publish_goal(self) -> None:
        """Publishes GraceNode's current goal using `goal_publisher`. If `goal` is None, it will log an error and do nothing.

        Usage:
            >>> grace.goal = RobotGoal(parent_object="dining table", child_object="cup") # Set the goal
            >>> grace.publish_goal() # Publish the goal
        """
        if self.goal is None:
            rospy.logerr("Attempted to publish an empty goal in GraceNode. Ignoring...")
            return

        self.goal_publisher.publish(
            self.goal.parent_object,
            self.goal.child_object,
        )
        if GraceNode.verbose:
            rospy.loginfo(f"Successfully published goal! {self.goal}.")

        # Kickstart GRACE
        threading.Thread(target=self.change_state, args=(RobotState.EXPLORING,)).start()

    # endregion
    # region State Implementation
    def execute_state(self, new_state: int, old_state: int) -> None:
        # BUG: Immediately goes through entire state machine after first goal is reached
        if new_state == RobotState.WAITING:
            self.grace_navigation.move_base.cancel_all_goals()
        elif new_state == RobotState.EXPLORING:
            # If previous state was HOMING, then it already has an object (invariant I believe)
            self.explore(go_to_child=not self.has_object)
        elif new_state == RobotState.PICKING:
            self.pick()
        elif new_state == RobotState.PLACING:
            self.place()
        elif new_state == RobotState.HOMING:
            self.home()

    def home(self) -> None:
        rospy.loginfo("Homing")
        self.grace_navigation.dummy_done_with_task()

    def explore(self, go_to_child: bool) -> None:
        assert self.goal
        EXPLORE_SECONDS = 60
        rospy.loginfo("Exploring")
        found_pose = self.grace_navigation.explore_until_found(
            self.goal,
            find_child=go_to_child,
            exploration_timeout=rospy.Duration(EXPLORE_SECONDS),
        )
        if found_pose is None:
            rospy.logwarn("Exploration timed out without finding the object!")
            self.change_state(RobotState.WAITING)
            return

        success: bool = self.grace_navigation.navigate_to_pose(
            found_pose, timeout=rospy.Duration(100)
        )
        if success:
            next_state, old_state = self.next_state()
            self.change_state(next_state)
            return

        rospy.logwarn("Failed to navigate to the object!")
        self.change_state(RobotState.WAITING)

    def pick(self) -> None:
        rospy.loginfo("Picking")
        self.has_object = True
        self.grace_navigation.dummy_done_with_task()

    def place(self) -> None:
        rospy.loginfo("Placing")
        self.has_object = False
        self.grace_navigation.dummy_done_with_task()

    # endregion
    def __call__(self) -> None:
        self.run()

    def __repr__(self) -> str:
        return f"GraceNode(state='{state_to_str(self.state)})', goal='{self.goal}')"

    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        """The callback for when the GraceNode needs to be shutdown through a ROSInterruptExecption."""
        self.grace_navigation.shutdown()
        if GraceNode.verbose:
            rospy.loginfo("Shutting down GraceNode!")


if __name__ == "__main__":
    rospy.init_node(name="GraceNode")  # type: ignore
    grace = GraceNode(verbose=True)
    rospy.on_shutdown(grace.shutdown)
    grace.goal = RobotGoal(parent_object="dining table", child_object="cup")
    rospy.sleep(3)  # Sleep for an arbitrary 3 seconds to allow sim map to load
    grace.publish_goal()
    try:
        grace.run()
    except rospy.ROSInterruptException:
        grace.shutdown()
        rospy.loginfo("GraceNode terminated GRACEfully.")  # type: ignore
