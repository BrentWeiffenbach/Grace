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
        self.place_location = "suitcase"
        self.pick_object_goal = "cup"
        rospy.sleep(5)  # Wait for move_group to initialize
        self.arm_group = MoveGroupCommander("arm_group", wait_for_servers=20)
        self.base_group = MoveGroupCommander("base_group", wait_for_servers=20)
        rospy.Subscriber("/grace/state", RobotState, self.state_callback)
        rospy.Subscriber("/grace/goal", RobotGoalMsg, self.goal_callback)

        # Publish a marker for the object pose
        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )
        rospy.sleep(1)

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

    def offset_object(self, object_posestamped, offset_distance, z_offset):
        # type: (PoseStamped, float, float) -> None
        # Create offset transformation matrix
        offset_matrix = tf.transformations.translation_matrix(
            [offset_distance, 0, z_offset]
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
            self.publish_marker(
                "base_footprint", self.offset_posestamped.pose, "adjusted_obj"
            )

    # endregion
    # region Pick
    def pick_object(self):
        assert self.obj_pose_srv is not None
        try:
            object_posestamped = self.get_pose_of_obj(self.pick_object_goal)
            if object_posestamped is None:
                return

            if self.verbose:
                self.publish_marker(
                    "base_footprint", object_posestamped.pose, "actual_obj"
                )

            OFFSET_DISTANCE = -0.24  # Tunable value. Lower is farther away from the object.
            Z_OFFSET = -0.04 # Tunable value. Lower is more down
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
                self.publish_marker(
                    "base_footprint", object_posestamped.pose, "actual_obj"
                )

            OFFSET_DISTANCE = 0.0  # Tunable value. Lower is farther away from the object.
            Z_OFFSET = object_posestamped.pose.position.z * 1.5  # Double the current Z of the object
            self.offset_object(object_posestamped, OFFSET_DISTANCE, Z_OFFSET)

            self.plan_base()
            # rospy.wait_for_message("/grace/planar_arrived", Bool)
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

        # If we are placing, .... uhhhh TODO: This is just a manual set point for now
        if self.state == RobotState.PLACING:
            self.place_object()

    # region plan_base
    def plan_base(self):
        BASE_X_OFFSET = -0.32  # Tunable value. The smaller this value, the further away from the object the base will stop.
        # Adjust the pose for base_group
        self.base_goal_pose.position.x = (
            self.offset_posestamped.pose.position.x + BASE_X_OFFSET
        )
        self.base_goal_pose.position.y = self.offset_posestamped.pose.position.y
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

        # Transform to map frame
        try:
            tf_listener = tf.TransformListener()
            tf_listener.waitForTransform(
                "map", "base_footprint", rospy.Time(0), rospy.Duration(4)
            )
            base_goal_stamped = PoseStamped()
            base_goal_stamped.header.frame_id = "base_footprint"
            base_goal_stamped.header.stamp = rospy.Time(0)
            base_goal_stamped.pose = self.base_goal_pose
            map_goal = tf_listener.transformPose("map", base_goal_stamped)
            if self.verbose:
                self.publish_marker("map", map_goal.pose, "map_goal")
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.logerr("Failed to transform pose: {}".format(e))
            return False

        # Extract yaw from quaternion
        euler = tf.transformations.euler_from_quaternion(
            [
                map_goal.pose.orientation.x,
                map_goal.pose.orientation.y,
                map_goal.pose.orientation.z,
                map_goal.pose.orientation.w,
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
                joint_values[0] = map_goal.pose.position.x
                joint_values[1] = map_goal.pose.position.y
                joint_values[2] = yaw
                self.base_group.set_joint_value_target(joint_values)

                rospy.loginfo("Trying with joint values: %s", joint_values)
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
        arm_relative_goal = self.transform_goal_to_relative_baselink()
        if arm_relative_goal is None:
            rospy.logerr("Error finding relative goal")
            return

        # Set goal tolerances for the arm group
        self.arm_group.set_goal_position_tolerance(0.01)  # 1 cm tolerance
        self.arm_group.set_goal_tolerance(0.01)  # General tolerance

        self.arm_group.set_pose_target(arm_relative_goal)
        self.arm_group.set_num_planning_attempts(5)
        self.arm_group.set_planning_time(5.0)
        success, plan, _, _ = self.arm_group.plan()

        if success:
            rospy.loginfo("Arm planning (to object from relative base) successful!")
        else:
            rospy.logerr("Arm planning (from relative base) failed!")

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

            # Get inverse (transformation from goal_base to base_footprint)
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
            T_obj_rel_goal_base = tf.transformations.concatenate_matrices(
                T_goal_base_inv, T_obj
            )

            # Extract position and orientation
            trans_obj = tf.transformations.translation_from_matrix(T_obj_rel_goal_base)
            quat_obj = tf.transformations.quaternion_from_matrix(T_obj_rel_goal_base)

            # Build transformed PoseStamped
            transformed_pose = PoseStamped()
            transformed_pose.header.frame_id = (
                "base_link"  # Relative to where base_link will be
            )
            transformed_pose.header.stamp = rospy.Time.now()
            transformed_pose.pose.position.x = trans_obj[0]
            transformed_pose.pose.position.y = trans_obj[1]
            transformed_pose.pose.position.z = trans_obj[2]
            transformed_pose.pose.orientation.x = quat_obj[0]
            transformed_pose.pose.orientation.y = quat_obj[1]
            transformed_pose.pose.orientation.z = quat_obj[2]
            transformed_pose.pose.orientation.w = quat_obj[3]

            if self.verbose:
                rospy.loginfo("Transformed object pose to goal base frame:")
                rospy.loginfo(
                    "Position: x=%f, y=%f, z=%f",
                    trans_obj[0],
                    trans_obj[1],
                    trans_obj[2],
                )

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
