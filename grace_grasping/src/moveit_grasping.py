#!/usr/bin/env python2.7
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, PoseStamped, Transform, Point, Quaternion  # noqa: F401
from moveit_commander.move_group import MoveGroupCommander  # type: ignore
from visualization_msgs.msg import Marker  # noqa: F401
import math

from grace_grasping.srv import (  # noqa: F401
    GetObjectPose,
    ObjectDetection,
    ObjectDetectionResponse,
)
from grace_navigation.msg import RangeBearing, RobotState, RobotGoalMsg  # noqa: F401


class MoveItGrasping:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.state = None
        self.obj_pose_srv = None
        self.offset_posestamped = PoseStamped()
        self.base_goal_pose = Pose()
        self.arm_relative_goal = None
        self.place_location = "suitcase"
        self.pick_object_goal = "cup"
        rospy.sleep(5)  # Wait for move_group to initialize
        self.arm_group = MoveGroupCommander("arm_group", wait_for_servers=20)
        self.base_group = MoveGroupCommander("base_group", wait_for_servers=20)
        rospy.Subscriber("/grace/state", RobotState, self.state_callback)
        rospy.Subscriber("/grace/goal", RobotGoalMsg, self.goal_callback)
        self.tf_listener = tf.TransformListener()

        # Publish a marker for the object pose
        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )
        rospy.sleep(1)

    def get_current_pose(self):
        """
        Gets the current robot pose from TF instead of odometry.
        Returns a Pose object or None if the transform isn't available.
        """
        assert self.tf_listener is not None
        try:
            self.tf_listener.waitForTransform(
                "map", "base_link", rospy.Time(0), rospy.Duration(0.5) # type: ignore
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0)
            )

            pose = Pose()
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]

            return pose
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.logwarn("Failed to get transform: {}".format(e))
            return None

    def connect_to_pose_service(self):
        # type: () -> None
        try:
            rospy.wait_for_service("get_object_pose", timeout=5)
            self.obj_pose_srv = rospy.ServiceProxy("get_object_pose", GetObjectPose)
            rospy.loginfo("Connected to GetObjectPose service!")
        except rospy.ROSException as e:
            rospy.logerr("Failed to connect to GetObjectPose service! {}".format(e))
            self.obj_pose_srv = None

    def ensure_pose_service_connection(self):
        # type: () -> bool
        if self.obj_pose_srv is None:
            self.connect_to_pose_service()
            if self.obj_pose_srv is None:
                rospy.logerr("Failed to connect to GetObjectPose service!")
                return False
        return self.obj_pose_srv is not None

    def get_pose_of_obj(self, obj):
        # type: (str) -> PoseStamped | None
        assert self.arm_group is not None and self.base_group is not None

        if not self.ensure_pose_service_connection():
            pass

        assert self.obj_pose_srv is not None

        obj_pose_srv_result = self.obj_pose_srv(obj, False)

        if obj_pose_srv_result.success:
            return obj_pose_srv_result.pose

        return None

    # region Offsets
    def get_xy_offset(self, pose, offset_distance):
        current_pose = self.get_current_pose()
        if current_pose is None:
            rospy.logerr("Current pose was none when planning")
            return

        current_position = current_pose.position
        # Calculate the direction vector from robot to object
        dx = pose.pose.position.x - current_position.x
        dy = pose.pose.position.y - current_position.y

        # Calculate the distance and normalize the direction vector
        distance = math.sqrt(dx * dx + dy * dy)
        if distance > 0:
            dx = dx / distance
            dy = dy / distance
        else:
            dx, dy = 1.0, 0.0  # Default to positive x if robot is at object position

        # Apply offset along this direction vector
        x_offset = dx * offset_distance
        y_offset = dy * offset_distance
        return (x_offset, y_offset)

    def offset_object(self, object_posestamped, offset_distance, z_offset):
        # type: (PoseStamped, float, float) -> None
        # Create offset transformation matrix
        offsets = self.get_xy_offset(object_posestamped, offset_distance)
        if offsets is None:
            return
        (x_offset, y_offset) = offsets

        offset_matrix = tf.transformations.translation_matrix(
            [x_offset, y_offset, z_offset]
        )

        # Create object pose transformation matrix
        obj_pose_matrix = tf.transformations.compose_matrix(
            translate=[
                object_posestamped.pose.position.x,
                object_posestamped.pose.position.y,
                object_posestamped.pose.position.z,
            ],
            angles=tf.transformations.euler_from_quaternion(
                [
                    object_posestamped.pose.orientation.x,
                    object_posestamped.pose.orientation.y,
                    object_posestamped.pose.orientation.z,
                    object_posestamped.pose.orientation.w,
                ]
            ),
        )

        # Apply the offset to the object pose
        adjusted_pose_matrix = tf.transformations.concatenate_matrices(
            obj_pose_matrix, offset_matrix
        )

        # Extract translation and quaternion from the adjusted pose matrix
        adjusted_translation = tf.transformations.translation_from_matrix(
            adjusted_pose_matrix
        )
        adjusted_quaternion = tf.transformations.quaternion_from_matrix(
            adjusted_pose_matrix
        )

        # Update the offset_posestamped with the adjusted position
        self.offset_posestamped = PoseStamped(
            header=object_posestamped.header,
            pose=Pose(
                position=Point(*adjusted_translation),
                orientation=Quaternion(*adjusted_quaternion),
            ),
        )

        # Publish a marker for the adjusted object pose if verbose
        if self.verbose:
            self.publish_marker("map", self.offset_posestamped.pose, "adjusted_obj")

    # endregion
    # region Pick
    def pick_object(self):
        assert self.obj_pose_srv is not None
        try:
            object_posestamped = self.get_pose_of_obj(self.pick_object_goal)
            if object_posestamped is None:
                return

            if self.verbose:
                self.publish_marker("map", object_posestamped.pose, "actual_obj")

            OFFSET_DISTANCE = (
                -0.22
            )  # Tunable value. Lower is farther away from the object.
            Z_OFFSET = 0.00  # Tunable value. Lower is more down
            self.offset_object(object_posestamped, OFFSET_DISTANCE, Z_OFFSET)

            self.plan_base()
            # rospy.wait_for_message("/grace/planar_arrived", Bool)
            self.plan_arm()

        except rospy.ServiceException as se:
            rospy.logerr("Failed to publish MoveIt! goal! {}".format(se))

    # endregion
    # region Place
    def place_object(self):
        assert self.obj_pose_srv is not None
        try:
            object_posestamped = self.get_pose_of_obj(self.place_location)
            if object_posestamped is None:
                return

            if self.verbose:
                self.publish_marker("map", object_posestamped.pose, "actual_obj")

            OFFSET_DISTANCE = (
                0.0  # Tunable value. Lower is farther away from the object.
            )
            Z_OFFSET = (
                object_posestamped.pose.position.z * 1.55
            )  # Double the current Z of the object
            self.offset_object(object_posestamped, OFFSET_DISTANCE, Z_OFFSET)

            self.plan_base()
            rospy.sleep(13)
            self.plan_arm()

        except rospy.ServiceException as se:
            rospy.logerr("Failed to publish MoveIt! goal! {}".format(se))

    # endregion

    def publish_goal(self):
        assert self.arm_group is not None and self.base_group is not None

        if not self.ensure_pose_service_connection():
            pass

        assert self.obj_pose_srv is not None

        # If we are picking, use get_object_pose service
        if self.state == RobotState.PICKING:
            self.pick_object()

        if self.state == RobotState.PLACING:
            self.place_object()

    # region plan_base
    def plan_base(self):
        BASE_OFFSET = -0.275 # Tunable value. The smaller this value, the further away from the object the base will stop.
        # Adjust the pose for base_group
        offsets = self.get_xy_offset(self.offset_posestamped, BASE_OFFSET)
        if offsets is None:
            return
        (x_offset, y_offset) = offsets

        self.base_goal_pose.position.x = (
            self.offset_posestamped.pose.position.x + x_offset
        )
        self.base_goal_pose.position.y = (
            self.offset_posestamped.pose.position.y + y_offset
        )
        self.base_goal_pose.position.z = 0.0
        try:
            # Get the current position of the base_link
            tf_listener = tf.TransformListener()
            tf_listener.waitForTransform(
                "map", "base_link", rospy.Time(0), rospy.Duration(4)
            )
            (trans, rot) = tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0)
            )

            # Calculate the angle between the current base_link position and the offset pose
            delta_x = self.offset_posestamped.pose.position.x - trans[0]
            delta_y = self.offset_posestamped.pose.position.y - trans[1]
            yaw_angle = math.atan2(delta_y, delta_x)

            # Convert yaw angle to quaternion
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)
            self.base_goal_pose.orientation = Quaternion(*quaternion)
        except Exception as e:
            rospy.logerr("Failed to calculate orientation for base_link: {}".format(e))
            return False

        # Debug information
        rospy.loginfo("Base group joints: %s", self.base_group.get_active_joints())
        rospy.loginfo(
            "Base group planning frame: %s", self.base_group.get_planning_frame()
        )
        rospy.loginfo(
            "Base group end effector: %s", self.base_group.get_end_effector_link()
        )

        self.publish_marker("map", self.base_goal_pose, "base_goal")
        # Extract yaw from quaternion
        euler = tf.transformations.euler_from_quaternion(
            [
                self.base_goal_pose.orientation.x,
                self.base_goal_pose.orientation.y,
                self.base_goal_pose.orientation.z,
                self.base_goal_pose.orientation.w,
            ]
        )
        yaw = euler[2]  # Get the yaw component

        # Set the target using joint values
        try:
            self.base_group.clear_pose_targets()
            joint_values = self.base_group.get_current_joint_values()
            rospy.loginfo("Current joint values: %s", joint_values)

            # Update joint values for x, y, and theta
            if len(joint_values) == 3:  # x, y, theta
                joint_values[0] = self.base_goal_pose.position.x
                joint_values[1] = self.base_goal_pose.position.y
                joint_values[2] = yaw
                self.base_group.set_joint_value_target(joint_values)

                rospy.loginfo("Trying with joint values: %s", joint_values)
                self.base_group.set_planning_time(3.0)
                plan_success, plan, _, _ = self.base_group.plan()

                if plan_success:
                    rospy.loginfo(
                        "Base planning successful with joint values approach!"
                    )
                    return True
                else:
                    rospy.logerr("Base planning failed with joint values approach!")
            else:
                rospy.logerr(
                    "Unexpected joint configuration for base, cannot set joint values"
                )

            return False

        except Exception as e:
            rospy.logerr("Error planning base movement: %s", str(e))
            return False

    # endregion
    # region plan_arm
    def plan_arm(self):
        self.arm_relative_goal = self.transform_goal_to_relative_baselink()
        if self.arm_relative_goal is None:
            rospy.logerr("Error finding relative goal")
            return

        # Set goal tolerances for the arm group
        self.arm_group.set_goal_position_tolerance(0.01)  # 1 cm tolerance
        # self.arm_group.set_goal_tolerance(0.01)  # General tolerance


        self.arm_group.set_pose_target(self.arm_relative_goal)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_planning_time(2.5)

        success, plan, _, _ = self.arm_group.plan()

        if success:
            rospy.loginfo("Arm planning (to object from relative base) successful!")
        else:
            rospy.logerr("Arm planning (from relative base) failed!")
            rospy.sleep(20)
            self.publish_marker("base_link", self.arm_relative_goal.pose, "real_arm_goal_Final") # type: ignore

    # endregion

    def state_callback(self, msg):
        self.state = msg.state
        if self.state in [RobotState.PLACING, RobotState.PICKING]:
            self.publish_goal()

    def goal_callback(self, msg):
        self.place_location = msg.place_location
        self.pick_object_goal = msg.pick_object

    def transform_goal_to_relative_baselink(self):
        try:
            # Convert base_goal_pose to transformation matrix
            trans = [
                self.base_goal_pose.position.x,
                self.base_goal_pose.position.y,
                self.base_goal_pose.position.z,
            ]
            rot = [
                self.base_goal_pose.orientation.x,
                self.base_goal_pose.orientation.y,
                self.base_goal_pose.orientation.z,
                self.base_goal_pose.orientation.w,
            ]
            T_goal_base = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(trans),
                tf.transformations.quaternion_matrix(rot),
            )

            # Get inverse (transformation from goal_base to map (was map))
            T_goal_base_inv = tf.transformations.inverse_matrix(T_goal_base)

            # Convert offset pose to matrix
            obj_trans = [
                self.offset_posestamped.pose.position.x,
                self.offset_posestamped.pose.position.y,
                self.offset_posestamped.pose.position.z,
            ]
            obj_rot = [
                self.offset_posestamped.pose.orientation.x,
                self.offset_posestamped.pose.orientation.y,
                self.offset_posestamped.pose.orientation.z,
                self.offset_posestamped.pose.orientation.w,
            ]
            T_obj = tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix(obj_trans),
                tf.transformations.quaternion_matrix(obj_rot),
            )

            # Transform: object pose relative to goal base position
            # This gives us where the object will be relative to where the robot will be after moving
            # We apply the inverse transformation to convert from map frame to the future base_link frame
            T_obj_rel_goal_base = tf.transformations.concatenate_matrices(
                T_goal_base_inv, T_obj
            )

            # Extract position and orientation
            trans_obj = tf.transformations.translation_from_matrix(T_obj_rel_goal_base)
            # quat_obj = tf.transformations.quaternion_from_matrix(T_obj_rel_goal_base)

            # Build transformed PoseStamped
            transformed_pose = PoseStamped()
            transformed_pose.header.frame_id = "base_link"  # named base link for moveit to use properly (imaginary base_link in the future)
            transformed_pose.header.stamp = rospy.Time.now()
            transformed_pose.pose.position.x = trans_obj[0]
            transformed_pose.pose.position.y = trans_obj[1]
            transformed_pose.pose.position.z = trans_obj[2]
            transformed_pose.pose.orientation.x = 0
            transformed_pose.pose.orientation.y = 0
            transformed_pose.pose.orientation.z = 0
            transformed_pose.pose.orientation.w = 1 # 0 0 0 1 for flat joint 6 orientation

            if self.verbose:
                rospy.loginfo("Transformed object pose to goal base frame:")
                rospy.loginfo(
                    "Position: x=%f, y=%f, z=%f",
                    trans_obj[0],
                    trans_obj[1],
                    trans_obj[2],
                )

            self.publish_marker("base_link", transformed_pose.pose, "real_arm_goal")

            return transformed_pose

        except Exception as e:
            rospy.logerr("Error finding relative goal: {}".format(e))
            return None

    def publish_marker(self, frame_id, pose, namespace):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("moveit_grasping")
    verbose = rospy.get_param("~verbose", False)
    assert type(verbose) is bool
    rospy.loginfo("MoveIt Goal Node Initalized")
    moveit_grasping = MoveItGrasping(verbose=verbose)
    rospy.spin()
