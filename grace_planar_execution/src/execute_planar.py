#!/usr/bin/env python
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "scripts"))
import math

import rospy
from actionlib import SimpleActionServer
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import Twist
from moveit_msgs.msg import MoveGroupActionResult
from nav_msgs.msg import Odometry
from planar_pure_pursuit import find_lookahead_point, pure_pursuit_control
from std_msgs.msg import Bool
from trajectory_utils import extract_path_from_trajectory, quaternion_to_yaw
from grace_navigation.msg import RobotState

# Pure Pursuit Controller Parameters (tunable)
LOOKAHEAD_DISTANCE = 0.1  # Lookahead distance (meters)
DESIRED_LINEAR_VELOCITY = 0.1  # Constant linear velocity (m/s)
POSITION_THRESHOLD = 0.022  # Position tolerance (meters) for goal achievement
ROTATION_THRESHOLD = 0.01  # Orientation tolerance (radians) for final rotation
Kp_rotation = 0.55  # Proportional gain for rotation correction
FINAL_ROT_KP = 5.3  # The amount angular_velocity gets multiplied by
MIN_VELOCITY = 0.6  # Minimum angular velocity to ensure movement
TIME_TO_MOVE_FORWARD = 2.5 # time to move forward after arm is finished

# Global variable to store current odometry
current_odom = None


def odom_callback(data):
    """
    Callback for the /odom topic.
    Updates the global current_odom variable.
    """
    global current_odom
    current_odom = data.pose.pose


def rotate_to_final_orientation(final_yaw):
    """
    Rotates the robot in place until its orientation matches the final desired yaw within a tolerance.

    Args:
        final_yaw: The desired final yaw (radians).
    """
    rate = rospy.Rate(30)  # 30 Hz control loop
    rospy.loginfo("Rotating to final orientation: {:.2f} rad".format(final_yaw))

    if current_odom is None:
        rospy.logwarn("Odometry data is not available. Skipping rotation.")
        return

    while not rospy.is_shutdown():
        if current_odom is None:
            rospy.logwarn("Waiting for odometry data for rotation...")
            rate.sleep()
            continue

        # Calculate the current yaw and error
        current_yaw = quaternion_to_yaw(current_odom.orientation)
        error_yaw = math.atan2(
            math.sin(final_yaw - current_yaw), math.cos(final_yaw - current_yaw)
        )
        # rospy.loginfo("Current yaw: {:.2f}, Target yaw: {:.2f}, Error yaw: {:.2f}".format(
        #     current_yaw, final_yaw, error_yaw
        # ))

        # Check if the error is within the threshold
        if abs(error_yaw) < ROTATION_THRESHOLD:
            rospy.loginfo("Final orientation reached within tolerance.")
            break

        # Publish angular velocity command
        twist_msg = Twist()
        angular_velocity = Kp_rotation * error_yaw * FINAL_ROT_KP
        # Set a minimum angular velocity to ensure movement
        if abs(angular_velocity) < MIN_VELOCITY:
            angular_velocity = MIN_VELOCITY if angular_velocity > 0 else -MIN_VELOCITY
        twist_msg.angular.z = angular_velocity
        twist_msg.linear.x = 0.0
        cmd_vel_pub.publish(twist_msg)
        # rospy.loginfo("Published angular velocity: {:.2f}".format(twist_msg.angular.z))
        rate.sleep()

    # Stop rotation
    twist_msg = Twist()
    twist_msg.angular.z = 0.0
    twist_msg.linear.x = 0.0
    cmd_vel_pub.publish(twist_msg)
    rospy.loginfo("Rotation complete; robot stopped.")


def execute_trajectory_pure_pursuit(points, initial_pose):
    """
    Executes the trajectory on a differential drive robot using pure pursuit control.
    Trajectory points are relative to the start of the motion, so they are transformed
    using the provided initial_pose.

    Args:
        points: List of trajectory points from MoveIt.
        initial_pose: geometry_msgs/Pose representing the robot's pose when the trajectory was received.
    """
    global current_odom, cmd_vel_pub

    # Transform relative trajectory points to global coordinates.
    path_full = extract_path_from_trajectory(points, initial_pose)
    if not path_full:
        rospy.logwarn("No valid path extracted from trajectory.")
        return

    # Extract (x, y) for path following and get final desired orientation.
    path_xy = [(p[0], p[1]) for p in path_full]
    final_yaw = path_full[-1][2]

    last_index = 0
    rate = rospy.Rate(20)  # 20 Hz control loop
    rospy.loginfo("Starting pure pursuit control on transformed trajectory.")

    while not rospy.is_shutdown():
        if current_odom is None:
            rospy.logwarn("Waiting for odometry data...")
            rate.sleep()
            continue

        # Get current global pose.
        current_x = current_odom.position.x
        current_y = current_odom.position.y
        current_yaw = quaternion_to_yaw(current_odom.orientation)
        current_pose = (current_x, current_y, current_yaw)

        # Find the lookahead point along the global path.
        lookahead_point, last_index = find_lookahead_point(
            path_xy, (current_x, current_y), LOOKAHEAD_DISTANCE, last_index
        )

        # Compute angular velocity using pure pursuit.
        angular_velocity = pure_pursuit_control(
            current_pose, lookahead_point, LOOKAHEAD_DISTANCE, DESIRED_LINEAR_VELOCITY
        )

        twist_msg = Twist()
        twist_msg.linear.x = DESIRED_LINEAR_VELOCITY
        twist_msg.angular.z = angular_velocity
        cmd_vel_pub.publish(twist_msg)

        rospy.logdebug(
            "Current Pose: ({:.2f}, {:.2f}, {:.2f}) | Lookahead: ({:.2f}, {:.2f}) | Cmd: v: {:.2f}, w: {:.2f}".format(
                current_x,
                current_y,
                current_yaw,
                lookahead_point[0],
                lookahead_point[1],
                DESIRED_LINEAR_VELOCITY,
                angular_velocity,
            )
        )

        # Check if the final position is reached.
        final_goal = path_xy[-1]
        goal_distance = math.hypot(final_goal[0] - current_x, final_goal[1] - current_y)
        if goal_distance < POSITION_THRESHOLD:
            arrived_pub = rospy.Publisher("/grace/planar_arrived", Bool, queue_size=10)
            rospy.sleep(1)
            arrived_msg = Bool()
            arrived_msg.data = True
            arrived_pub.publish(arrived_msg)
            rospy.loginfo(
                "Published True to /grace/planar_arrived to indicate arrival at final goal."
            )
            rospy.loginfo(
                "Final goal position reached: distance {:.2f} m.".format(goal_distance)
            )
            break

        rate.sleep()

    # Stop the robot before rotating.
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    cmd_vel_pub.publish(twist_msg)

    # Rotate in place to achieve the final orientation.
    rotate_to_final_orientation(final_yaw)
    # Check if /grace/state is 2 before proceeding

    completion_pub = rospy.Publisher("/grace/planar_execution", Bool, queue_size=10)
    rospy.sleep(1)
    try:
        state_msg = rospy.wait_for_message("/grace/state", RobotState, timeout=5.0)
        assert type(state_msg) is RobotState
        if state_msg.state != RobotState.PICKING:
            rospy.loginfo("State is not Picking. Skipping forward motion.")
            rospy.sleep(5)
            completion_pub.publish(Bool(True))
            return
        rospy.loginfo("State is Picking. Proceeding with forward motion.")
    except rospy.ROSException:
        rospy.logwarn("Timeout waiting for message on /grace/state.")
        return

    # Publish a small forward twist to grab the object.
    # Wait for a message on /grace/arm_status or timeout after 5 seconds
    try:
        arm_status_msg = rospy.wait_for_message("/grace/arm_status", Bool, timeout=5.0)
        assert type(arm_status_msg) is Bool
        if arm_status_msg.data:
            rospy.loginfo("Received message on /grace/arm_status.")
        else:
            rospy.logwarn("Received message on /grace/arm_status, but data is False.")
    except rospy.ROSException:
        rospy.logwarn("Timeout waiting for message on /grace/arm_status.")

    twist_msg = Twist()
    twist_msg.linear.x = 0.09  # Slow forward motion
    twist_msg.angular.z = 0.0
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < TIME_TO_MOVE_FORWARD:  # time to move
        cmd_vel_pub.publish(twist_msg)
        rospy.sleep(0.1)  # Small delay to maintain control loop
    cmd_vel_pub.publish(Twist())  # Stop the robot
    rospy.loginfo("Published small forward twist to grab the object.")
    rospy.loginfo("Pure pursuit trajectory execution complete.")
    # Publish a True boolean to /grace/planar_execution to indicate completion.
    completion_pub.publish(Bool(True))
    rospy.loginfo(
        "Published True to /grace/planar_execution to indicate trajectory execution completion."
    )


def execute_actioncb(data):
    """
    Callback for MoveGroupActionResult messages.
    Extracts the relative trajectory and executes it using pure pursuit control.
    The current odometry when the trajectory is received is treated as the origin.
    """
    rospy.loginfo("Received MoveGroupActionResult message.")
    move_group_action = data.result
    planned_trajectory = move_group_action.planned_trajectory
    multi_dof_joint_trajectory = planned_trajectory.multi_dof_joint_trajectory
    points = multi_dof_joint_trajectory.points

    if not points:
        rospy.logwarn("Trajectory has no points.")
        return

    if current_odom is None:
        rospy.logwarn("No odometry available at trajectory start!")
        return

    # Use the current odometry as the origin (0,0,0) for the trajectory.
    initial_pose = current_odom
    rospy.loginfo(
        "Executing trajectory with {} points using pure pursuit control.".format(
            len(points)
        )
    )
    execute_trajectory_pure_pursuit(points, initial_pose)


def execute_trajectory_goal(goal):
    rospy.loginfo("Tricked ya!")


if __name__ == "__main__":
    rospy.init_node("moveit_pure_pursuit_controller", anonymous=True)
    rospy.loginfo("moveit_pure_pursuit_controller node initialized.")

    rospy.Subscriber("/move_group/result", MoveGroupActionResult, execute_actioncb)
    rospy.loginfo("Subscribed to /move_group/result topic.")

    cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
    rospy.loginfo("Publisher for /cmd_vel_mux/input/navi topic created.")

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.loginfo("Subscribed to /odom topic.")

    action_server = SimpleActionServer(
        "moveit_pure_pursuit_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction,
        execute_trajectory_goal,
        False,
    )
    action_server.start()
    rospy.loginfo("FollowJointTrajectory action server started.")

    rospy.spin()
