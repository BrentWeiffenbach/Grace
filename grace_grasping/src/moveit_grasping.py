#!/usr/bin/env python2.7
from moveit_commander.move_group import MoveGroupCommander  # type: ignore
from grace_navigation.msg import RobotState, RangeBearing  # noqa: F401
from grace_grasping.srv import GetObjectPose, ObjectDetection, ObjectDetectionResponse  # noqa: F401
from geometry_msgs.msg import PoseStamped, Pose, Transform  # noqa: F401
import tf.transformations
from visualization_msgs.msg import Marker  # noqa: F401
import rospy
import tf
from std_msgs.msg import Bool


class MoveItGrasping:
    def __init__(self, verbose=False):
        self.verbose = verbose
        rospy.sleep(5)  # Wait for move_group to initialize
        self.arm_group = MoveGroupCommander("arm_group", wait_for_servers=20)
        self.base_group = MoveGroupCommander("base_group", wait_for_servers=20)
        rospy.Subscriber("/grace/state", RobotState, self.state_callback)

        # Publish a marker for the object pose
        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10
        )
        rospy.sleep(1)
        self.state = None
        self.obj_pose_srv = None
        self.object_detection_srv = None
        self.object_posestamped = PoseStamped()
        self.base_goal_pose = Pose()

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

    def connect_to_detection_service(self):
        # type: () -> None
        try:
            rospy.wait_for_service("object_detection", timeout=5)
            self.object_detection_srv = rospy.ServiceProxy(
                "object_detection", ObjectDetection
            )
            rospy.loginfo("Connected to ObjectDetection service!")
        except rospy.ROSException as e:
            rospy.logerr("Failed to connect to ObjectDetection service! {}".format(e))
            self.obj_pose_srv = None

    def ensure_detection_service_connection(self):
        # type: () -> bool
        if self.object_detection_srv is None:
            self.connect_to_detection_service()
            if self.object_detection_srv is None:
                rospy.logerr("Failed to connect to ObjectDetection service!")
                return False
        return self.object_detection_srv is not None
    
    def get_pose_of_obj(self, obj):
        # type: (str) -> PoseStamped | None
        assert self.arm_group is not None and self.base_group is not None

        if not self.ensure_pose_service_connection():
            pass

        if not self.ensure_detection_service_connection():
            pass
        
        assert self.obj_pose_srv is not None
        assert self.object_detection_srv is not None
        
        TRY_AGAIN_MAX = 5
        obj_pose_srv_result = None
        for i in range(0, TRY_AGAIN_MAX):
            object_detection_result = self.object_detection_srv(obj)  # type: ObjectDetectionResponse
            if not object_detection_result.success:
                if i < TRY_AGAIN_MAX - 1:
                    rospy.loginfo(
                        "The service did not find the object! Trying again..."
                    )
                    continue
                else:
                    rospy.logerr(
                        "The service could not find the object even after trying again!"
                    )
                    return
            range_bearing = object_detection_result.range_bearing  # type: RangeBearing

            obj_pose_srv_result = self.obj_pose_srv(range_bearing)
            if not obj_pose_srv_result.success:
                # TODO: Deal with failures
                if i < TRY_AGAIN_MAX - 1:
                    rospy.logwarn("Pose is unknown. Trying again...")
                    continue
                else:
                    rospy.logerr("The pose is unknown even after trying again.")
                    return
            assert obj_pose_srv_result.pose
            assert isinstance(obj_pose_srv_result.pose, PoseStamped)
            # Loop is only here to allow it to run multiple times
            # If it can get this far, just break out of the loop
            break

        assert obj_pose_srv_result
        return obj_pose_srv_result.pose
        
    def publish_goal(self):
        assert self.arm_group is not None and self.base_group is not None

        if not self.ensure_pose_service_connection():
            pass

        if not self.ensure_detection_service_connection():
            pass
        
        assert self.obj_pose_srv is not None
        assert self.object_detection_srv is not None

        # If we are picking, use get_object_pose service
        if self.state == RobotState.PICKING:
            assert self.obj_pose_srv is not None
            assert self.object_detection_srv is not None

            try:
                object_posestamped = self.get_pose_of_obj("cup")
                if object_posestamped is None:
                    return
                
                self.object_posestamped = object_posestamped  # type: PoseStamped
                self.publish_marker(
                    "base_footprint", self.object_posestamped.pose, "actual_obj"
                )

                self.plan_base()
                rospy.wait_for_message("/grace/planar_arrived", Bool)
                self.plan_arm()

            except rospy.ServiceException as se:
                rospy.logerr("Failed to publish MoveIt! goal! {}".format(se))

        # If we are placing, .... uhhhh TODO: This is just a manual set point for now
        if self.state == RobotState.PLACING:
            self.place_object()

    def plan_base(self):
        # Adjust the pose for base_group
        self.base_goal_pose.position.x = self.object_posestamped.pose.position.x + -0.47
        self.base_goal_pose.position.y = self.object_posestamped.pose.position.y
        self.base_goal_pose.position.z = 0.0
        self.base_goal_pose.orientation = self.object_posestamped.pose.orientation

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

    def plan_arm(self):
        virtual_obj_pose = self.transform_pose_to_virtual_baselink()

        # Offset the pose to avoid collision with the object
        OFFSET_DISTANCE = -0.145  # Move 18 cm towards the robot
        offset_vector = tf.transformations.translation_matrix([OFFSET_DISTANCE, 0, 0])
        obj_pose_matrix = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix([
                virtual_obj_pose.pose.position.x,
                virtual_obj_pose.pose.position.y,
                virtual_obj_pose.pose.position.z + 0.01
            ]),
            tf.transformations.quaternion_matrix([
                virtual_obj_pose.pose.orientation.x,
                virtual_obj_pose.pose.orientation.y,
                virtual_obj_pose.pose.orientation.z,
                virtual_obj_pose.pose.orientation.w
            ])
        )
        adjusted_pose_matrix = tf.transformations.concatenate_matrices(
            obj_pose_matrix, offset_vector
        )
        adjusted_translation = tf.transformations.translation_from_matrix(adjusted_pose_matrix)
        adjusted_quaternion = tf.transformations.quaternion_from_matrix(adjusted_pose_matrix)

        # Update the virtual_obj_pose with the adjusted position
        virtual_obj_pose.pose.position.x = adjusted_translation[0]
        virtual_obj_pose.pose.position.y = adjusted_translation[1]
        virtual_obj_pose.pose.position.z = adjusted_translation[2]
        virtual_obj_pose.pose.orientation.x = adjusted_quaternion[0]
        virtual_obj_pose.pose.orientation.y = adjusted_quaternion[1]
        virtual_obj_pose.pose.orientation.z = adjusted_quaternion[2]
        virtual_obj_pose.pose.orientation.w = adjusted_quaternion[3]

        self.publish_marker("base_link", virtual_obj_pose.pose, "virtual_obj_pose")

        # Set goal tolerances for the arm group
        self.arm_group.set_goal_position_tolerance(0.01)  # 1 cm tolerance
        # self.arm_group.set_goal_orientation_tolerance(0.08)  # ~2.87 degrees tolerance
        self.arm_group.set_goal_tolerance(0.02)  # General tolerance

        self.arm_group.set_pose_target(virtual_obj_pose)
        self.arm_group.set_num_planning_attempts(5)
        self.arm_group.set_planning_time(5.0)
        success, plan, _, _ = self.arm_group.plan()

        if success:
            rospy.loginfo("Arm planning (to object from virtual base) successful!")
        else:
            rospy.logerr("Arm planning (from virtual base) failed!")

    def place_object(self):
        assert self.obj_pose_srv is not None
        assert self.object_detection_srv is not None
        goal_joint_values = [
            0.08059417813617288,
            0.95,
            -0.3932249476604554,
            0.18421584162174223,
            0.45491917923620506,
            0.16590019448519736,
        ]
        
        obj_posestamped = self.get_pose_of_obj("suitcase")
        
        if obj_posestamped is None:
            return
        
        self.object_posestamped = obj_posestamped

        self.arm_group.set_joint_value_target(goal_joint_values)

        # Plan to the goal
        success, plan, _, _ = self.arm_group.plan()
        if success:
            rospy.loginfo("Planning successful!")
        else:
            rospy.logerr("Planning failed!")

    def state_callback(self, msg):
        self.state = msg.state
        if self.state in [RobotState.PLACING, RobotState.PICKING]:
            self.publish_goal()

    def transform_pose_to_virtual_baselink(self):
        # Convert the base goal pose to transformation matrix
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
        T_virtual_base = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans),
            tf.transformations.quaternion_matrix(rot),
        )

        # Now get the inverse (transform from world to virtual base)
        T_virtual_base_inv = tf.transformations.inverse_matrix(T_virtual_base)

        # Convert object pose to matrix
        obj_trans = [
            self.object_posestamped.pose.position.x - 0.1,
            self.object_posestamped.pose.position.y,
            self.object_posestamped.pose.position.z,
        ]
        obj_rot = [
            self.object_posestamped.pose.orientation.x,
            self.object_posestamped.pose.orientation.y,
            self.object_posestamped.pose.orientation.z,
            self.object_posestamped.pose.orientation.w,
        ]
        T_obj = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(obj_trans),
            tf.transformations.quaternion_matrix(obj_rot),
        )

        # Transform the object pose into the virtual base frame
        T_obj_in_virtual_base = tf.transformations.concatenate_matrices(
            T_virtual_base_inv, T_obj
        )
        trans_obj = tf.transformations.translation_from_matrix(T_obj_in_virtual_base)
        quat_obj = tf.transformations.quaternion_from_matrix(T_obj_in_virtual_base)

        # Build the transformed PoseStamped
        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = "base_link"
        transformed_pose.pose.position.x = trans_obj[0]
        transformed_pose.pose.position.y = trans_obj[1]
        transformed_pose.pose.position.z = trans_obj[2]
        transformed_pose.pose.orientation.x = quat_obj[0]
        transformed_pose.pose.orientation.y = quat_obj[1]
        transformed_pose.pose.orientation.z = quat_obj[2]
        transformed_pose.pose.orientation.w = quat_obj[3]
        return transformed_pose

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
