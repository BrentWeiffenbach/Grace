from typing import Dict, Final, List, Union

import actionlib
import rospy
from geometry_msgs.msg import Twist
from grace_navigation.msg import RobotGoalMsg, RobotState
from std_msgs.msg import Bool


def get_constants_from_msg(msg: type) -> Dict[int, str]:
    """
    The keys and values are accessed by getting all the non-callable int constants from the autogenerated python file.

    Args:
        msg (type): The imported msg class to get the constants from. e.g. for `from grace_navigation.msg import RobotGoalMsg`, this would be `RobotGoalMsg`.

    Returns:
        Dict (int, str): The enum value and their corresponding states from `msg`'s autogenerated python file. Follows key/value of int: STATE.
    """
    return {
        value: key
        for key, value in msg.__dict__.items()
        if not key.startswith("_") and not callable(value) and isinstance(value, int)
    }


class RobotGoal:
    """A RobotGoal consists of a `place_location` and a `pick_object`. The `place_location` is the destination to place the `pick_object`.

    An example is with a coffee cup and a table, the `place_location` would be the table and the `pick_object` is the coffee cup.
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

    def __init__(self, place_location: str, pick_object: str) -> None:
        """
        Args:
            place_location (str): The object to place the `pick_object` on. Must be a valid YOLO class.
            pick_object (str): The object to place on a `place_location`. Must be a valid YOLO class.
        """
        self.place_location = place_location
        self.pick_object = pick_object

    @property
    def place_location(self) -> str:
        """The object to place the `pick_object` on. The table in a coffee cup/table relationship.

        Returns:
            str: The place location object's class name.
        """
        return self._place_location

    @property
    def pick_object(self) -> str:
        """The object to place on a `place_location`. The coffee cup in a coffee cup/table relationship.

        Returns:
            str: The pick object's class name.
        """
        return self._pick_object

    @place_location.setter
    def place_location(self, place_location: str) -> None:
        if not RobotGoal.is_valid_class(place_location):
            raise ValueError(
                f"Invalid place location. {place_location} is not in RobotGoal.classes!"
            )
        self._place_location = place_location

    @pick_object.setter
    def pick_object(self, new_pick_object: str) -> None:
        if not RobotGoal.is_valid_class(new_pick_object):
            raise ValueError(
                f"Invalid pick object. {new_pick_object} is not in RobotGoal.classes!"
            )
        self._pick_object = new_pick_object

    @staticmethod
    def is_valid_class(_cls: str) -> bool:
        return _cls in RobotGoal.classes

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, RobotGoal):
            return False
        return (
            self.pick_object == other.pick_object
            and self.place_location == other.place_location
        )

    def __str__(self) -> str:
        return f"Goal: Place {self.pick_object} on a {self.place_location}"

    def __repr__(self) -> str:
        return f"RobotGoal(place_location={self.place_location}, pick_object={self.pick_object})"


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


# region GraceNode
class GraceNode:
    """The main node for controlling the turtlebot. Is the link between a (as of January 23, 2025) theoretical frontend and the turtlebot. Send a goal to /grace/goal to kickstart the application.

    Attributes:
        state (int): The state of the turtlebot. Must be a valid constant in `RobotState.msg`.
        goal (RobotGoal | None): The goal of the robot. Can be None.
        verbose (bool, optional): Whether verbose output should be logged. Defaults to False.
        STATE_TOPIC (str, Final): The topic to publish the states to.
        GOAL_TOPIC (str, Final): The topic to publish the goals to.
        NAV_STATUS_TOPIC (str, Final): Topic navigation status is published to.
        ARM_CONTROL_TOPIC (str, Final): Topic for arm controller status being published to.
        HAS_OBJECT_TOPIC (str, Final): Topic for if the arm has an object or not.

    Publishers:
        state_publisher (RobotState): Publishes to STATE_TOPIC. Details the current state of GRACE.
        goal_publisher (RobotGoalMsg): Publishes to GOAL_TOPIC. Contains GRACE's place loaction and pick object.
        has_object_publisher (Bool): Publishes to HAS_OBJECT_TOPIC if GRACE has an object.

    Subscribers:
        nav_status_subscriber (actionlib.GoalStatus): Subscribes to NAV_STATUS_TOPIC. Used to update state based on status of exploring/navigation.
        arm_control_status_subscriber (Bool): Subscribes to ARM_CONTROL_TOPIC. Updates state based on top level arm controller status.
    """

    verbose: bool = False

    STATE_TOPIC: Final[str] = "/grace/state"
    GOAL_TOPIC: Final[str] = "/grace/goal"
    NAV_STATUS_TOPIC: Final[str] = "/grace/nav_status"
    ARM_CONTROL_TOPIC: Final[str] = "/grace/arm_control_status"
    HAS_OBJECT_TOPIC: Final[str] = "/grace/has_object"
    
    DEFAULT_STATE: Final[int] = RobotState.WAITING
    
    nav_statuses: Dict[int, str] = get_constants_from_msg(actionlib.GoalStatus)
    """Gets all of the non-callable integer constants from actionlib.GoalStatus msg. """

    def __init__(self, verbose: bool = False) -> None:
        """Initializes the GraceNode.

        Args:
            verbose (bool, optional): Whether verbose output should be logged. Defaults to False.
        """
        self._state: int
        self._goal: Union[RobotGoal, None]
        GraceNode.verbose: bool = verbose

        # TODO: Add a user-friendly input to change the goal and display the state.
        self._state = GraceNode.DEFAULT_STATE
        self._goal = None

        self.has_object: bool = False

        # state pub
        self.state_publisher = rospy.Publisher(
            name=GraceNode.STATE_TOPIC, data_class=RobotState, queue_size=5, latch=True
        )
        # goal pub
        self.goal_publisher = rospy.Publisher(
            name=GraceNode.GOAL_TOPIC, data_class=RobotGoalMsg, queue_size=5, latch=True
        )
        # has_object pub
        self.has_object_publisher = rospy.Publisher(
            name=GraceNode.HAS_OBJECT_TOPIC, data_class=Bool, queue_size=2, latch=True
        )
        self.has_object_publisher.publish(False)
        # navigation status sub
        self.nav_status_subscriber = rospy.Subscriber(
            GraceNode.NAV_STATUS_TOPIC,
            data_class=actionlib.GoalStatus,
            callback=self.navigation_cb,
        )
        # arm control status
        self.arm_control_status_subscriber = rospy.Subscriber(
            GraceNode.ARM_CONTROL_TOPIC,
            data_class=Bool,
            callback=self.arm_control_cb,
        )

        if GraceNode.verbose:
            rospy.loginfo(
                f"GRACE is currently {state_to_str(self.state)}{f'. {self.goal}.' if self.goal else ' with no current goal.'}"
            )

    # region Setters
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
        self.state_publisher.publish(self.state)

    @property
    def goal(self) -> Union[RobotGoal, None]:
        """The goal of the turtlebot. Contains a place_location and a pick_object. See `RobotGoal`'s documentation for details.

        Returns:
            RobotGoal: The turtlebot's goal.

        Usage:
            >>> grace.goal = RobotGoal(place_location="dining table", pick_object="cup") # Set the goal
            >>> grace.publish_goal() # Publish the goal
        """
        return self._goal

    @goal.setter
    def goal(self, new_goal: RobotGoal) -> None:
        _temp_goal: Union[RobotGoal, None] = self.goal
        self._goal = new_goal  # new_goal will be validated within RobotGoal's setters
        if GraceNode.verbose:
            rospy.loginfo(f"Changed goal from {_temp_goal} to {self._goal}.")

    # endregion
    # region Status Callbacks
    def navigation_cb(self, state_msg: actionlib.GoalStatus) -> None:
        """
        Callback function for navigation status updates.
        This function is called when the navigation status changes. It updates the
        state of the robot based on the navigation result and the current state of
        the robot.
        Args:
            state_msg (int): The navigation status code.
        Returns:
            None
        """
        # only update state if we are exploring
        if self.state != RobotState.EXPLORING:
            return

        if GraceNode.nav_statuses[state_msg.status] in ["SUCCEEDED"]:
            rospy.loginfo(
                f"Move base has status: {GraceNode.nav_statuses[state_msg.status]}"
            )

        elif GraceNode.nav_statuses[state_msg.status] in ["PREEMPTED"]:
            rospy.logwarn(
                f"Move base has status: {GraceNode.nav_statuses[state_msg.status]}"
            )

        else:
            rospy.logerr(
                f"Move base has status: {GraceNode.nav_statuses[state_msg.status]}"
            )
        if GraceNode.nav_statuses[state_msg.status] == "SUCCEEDED":
            if self.has_object:
                self.state = RobotState.PLACING
            else:
                self.state = RobotState.PICKING
        elif GraceNode.nav_statuses[state_msg.status] == "ABORTED":
            self.state = RobotState.WAITING
            rospy.logerr("move_base failed to go to the goal pose!")
        # LOST represents the node exploration timing out without finding the object
        elif GraceNode.nav_statuses[state_msg.status] == "LOST":
            self.state = RobotState.WAITING
            rospy.logerr("move_base timed out without finding the goal object!")

    def arm_control_cb(self, is_completed: Bool) -> None:
        """
        Callback function to control the state of the robot's arm.
        This function updates the robot's state based on the completion status of
        arm movements such as picking, placing, and homing. The state transitions
        are as follows:
        - If the robot is in the WAITING or EXPLORING state, the function returns immediately.
        - If the robot is in the PICKING state and the action is completed, the state changes to HOMING and the robot is marked as having an object.
        - If the robot is in the PLACING state and the action is completed, the state changes to HOMING.
        - If the robot is in the HOMING state:
            - If the robot has an object and the action is completed, the object is released and the state changes to WAITING.
            - If the action is completed without having an object, the state changes to EXPLORING.
        Args:
            is_completed (Bool): A flag indicating whether the arm movement action is completed.
        """

        # only update arm state from picking / placing / homeing
        if self.state == RobotState.WAITING or self.state == RobotState.EXPLORING:
            return
        if self.state == RobotState.PICKING and is_completed.data:
            self.state = RobotState.HOMING
            self.has_object = True
            self.has_object_publisher.publish(self.has_object)
        elif self.state == RobotState.PLACING and is_completed.data:
            self.has_object = False
            self.has_object_publisher.publish(self.has_object)
            self.state = RobotState.HOMING
        elif self.state == RobotState.HOMING:
            if self.has_object and is_completed.data:
                self.state = RobotState.EXPLORING
            elif is_completed.data:
                self.state = RobotState.WAITING

    # endregion
    # region Node Functions

    # endregion
    def __call__(self) -> None:
        self.run()

    def __repr__(self) -> str:
        return f"GraceNode(state='{state_to_str(self.state)})', goal='{self.goal}')"

    def run(self) -> None:
        rospy.spin()

    def shutdown(self) -> None:
        """The callback for when the GraceNode needs to be shutdown through a ROSInterruptExecption."""
        if GraceNode.verbose:
            rospy.loginfo("Shutting down GraceNode!")

    def publish_goal(self) -> None:
        """Publishes GraceNode's current goal using `goal_publisher`. If `goal` is None, it will log an error and do nothing.

        Usage:
            >>> grace.goal = RobotGoal(place_location="dining table", pick_object="cup") # Set the goal
            >>> grace.publish_goal() # Publish the goal
        """
        if self.goal is None:
            rospy.logerr("Attempted to publish an empty goal in GraceNode. Ignoring...")
            return

        self.goal_publisher.publish(
            self.goal.place_location,
            self.goal.pick_object,
        )

        rospy.loginfo(f"Successfully published goal! {self.goal}.")

        # Start exploring!
        self.state = RobotState.EXPLORING


def rotate_360() -> None:
    rotate_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=1)
    rotate_msg = Twist()
    rotate_msg.angular.z = 1.0  # Rotate at 1 rad/s

    # Rotate for 2*pi seconds to complete a full rotation
    rotate_duration = rospy.Duration(int(2 * 3.14159))
    rotate_end_time = rospy.Time.now() + rotate_duration

    while rospy.Time.now() < rotate_end_time:
        rotate_pub.publish(rotate_msg)
        rospy.sleep(0.1)

    # Stop rotation
    rotate_msg.angular.z = 0.0
    rotate_pub.publish(rotate_msg)
    # Rotate 360 degrees before doing exploration


if __name__ == "__main__":
    rospy.init_node(name="GraceNode")  # type: ignore
    verbose = rospy.get_param("~verbose", False)
    assert type(verbose) is bool
    grace = GraceNode(verbose=verbose)
    rospy.on_shutdown(grace.shutdown)
    rospy.wait_for_message("/map", rospy.AnyMsg) # Wait for map before starting
    rospy.sleep(5)
    rotate_360()
    grace.state = GraceNode.DEFAULT_STATE
    grace.goal = RobotGoal(place_location="dining table", pick_object="elephant")
    rospy.sleep(5)  # Sleep for an arbitrary 3 seconds to allow sim map to load
    grace.publish_goal()
    try:
        grace.run()
    except rospy.ROSInterruptException:
        grace.shutdown()
        rospy.loginfo("GraceNode terminated GRACEfully.")  # type: ignore
