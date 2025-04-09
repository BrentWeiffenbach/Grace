#!/usr/bin/env python2.7
from moveit_commander.move_group import MoveGroupCommander # type: ignore
from grace_navigation.msg import RobotState, RangeBearing  # noqa: F401
from grace_grasping.srv import GetObjectPose, ObjectDetection, ObjectDetectionResponse  # noqa: F401
from geometry_msgs.msg import PoseStamped, Pose, Transform  # noqa: F401
import tf.transformations
from visualization_msgs.msg import Marker  # noqa: F401
import rospy
import tf

class MoveItGrasping:
    def __init__(self):
        rospy.sleep(20)  # Wait for move_group to initialize
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

    def publish_goal(self):
        assert self.arm_group is not None and self.base_group is not None

        if not self.ensure_pose_service_connection():
            # TODO: Handle case of not connected service?
            pass

        if not self.ensure_detection_service_connection():
            # TODO: Handle case of not connected service?
            pass

        # If we are picking, use get_object_pose service
        if self.state == RobotState.PICKING:
            assert self.obj_pose_srv is not None
            assert self.object_detection_srv is not None

            try:
                TRY_AGAIN_MAX = 2
                obj_pose_srv_result = None
                for i in range(0, TRY_AGAIN_MAX):
                    object_detection_result = self.object_detection_srv("cup")  # type: ObjectDetectionResponse
                    if not object_detection_result.success:
                        if i < TRY_AGAIN_MAX - 1:
                            rospy.loginfo("The service did not find the object! Trying again...")
                            continue
                        else:
                            rospy.logerr("The service could not find the object even after trying again!")
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
                
                assert obj_pose_srv_result
                self.object_posestamped = obj_pose_srv_result.pose # type: PoseStamped
                self.publish_marker("base_footprint", self.object_posestamped.pose, "actual_obj")

                self.plan_base()
                rospy.sleep(3)
                self.plan_arm()

            except rospy.ServiceException as se:
                rospy.logerr("Failed to publish MoveIt! goal! {}".format(se))

        # If we are placing, .... uhhhh TODO: This is just a manual set point for now
        if self.state == RobotState.PLACING:
            self.place_object()
    

    def plan_base(self):
        # Adjust the pose for base_group
        self.base_goal_pose.position.x = self.object_posestamped.pose.position.x + -0.3
        self.base_goal_pose.position.y = self.object_posestamped.pose.position.y
        self.base_goal_pose.position.z = 0.0
        self.base_goal_pose.orientation = self.object_posestamped.pose.orientation
        
        # Debug information
        rospy.loginfo("Base group joints: %s", self.base_group.get_active_joints())
        rospy.loginfo("Base group planning frame: %s", self.base_group.get_planning_frame())
        rospy.loginfo("Base group end effector: %s", self.base_group.get_end_effector_link())
        
        # Transform to map frame
        try:
            tf_listener = tf.TransformListener()
            tf_listener.waitForTransform("map", "base_footprint", rospy.Time(0), rospy.Duration(4))
            base_goal_stamped = PoseStamped()
            base_goal_stamped.header.frame_id = "base_footprint"
            base_goal_stamped.header.stamp = rospy.Time(0)
            base_goal_stamped.pose = self.base_goal_pose
            map_goal = tf_listener.transformPose("map", base_goal_stamped)
            self.publish_marker("map", map_goal.pose, "map_goal")
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Failed to transform pose: {}".format(e))
            return False
        
        # For a differential drive base, we should use the joint values approach
        # Extract yaw from quaternion
        euler = tf.transformations.euler_from_quaternion([
            map_goal.pose.orientation.x,
            map_goal.pose.orientation.y,
            map_goal.pose.orientation.z,
            map_goal.pose.orientation.w
        ])
        yaw = euler[2]  # Get the yaw component
        
        # Set the target using position and yaw directly
        try:
            # Directly set position reference for a planar joint
            self.base_group.clear_pose_targets()
            
            # The following works for a planar or floating joint representation
            reference_point = PoseStamped()
            reference_point.header.frame_id = "map"
            reference_point.header.stamp = rospy.Time.now()
            reference_point.pose.position.x = map_goal.pose.position.x
            reference_point.pose.position.y = map_goal.pose.position.y
            reference_point.pose.position.z = 0.0
            reference_point.pose.orientation.x = 0.0
            reference_point.pose.orientation.y = 0.0
            reference_point.pose.orientation.z = 0.0
            reference_point.pose.orientation.w = 1.0
            
            # Try setting position target instead of full pose
            self.base_group.set_position_target(
                [map_goal.pose.position.x, map_goal.pose.position.y, 0.0]
            )
            
            # Planning settings
            self.base_group.set_num_planning_attempts(10)
            self.base_group.set_planning_time(7.0)
            
            rospy.loginfo("Planning for base with position target")
            plan_success, plan, _, _ = self.base_group.plan()
            
            if plan_success:
                rospy.loginfo("Base planning successful!")
                return True
            else:
                rospy.logerr("Base planning failed with position target approach!")
                
                # Try with joint value target as fallback
                self.base_group.clear_pose_targets()
                joint_values = self.base_group.get_current_joint_values()
                rospy.loginfo("Current joint values: %s", joint_values)
                
                # Try to find the appropriate joints - common names for base joints
                if len(joint_values) == 3:  # x, y, theta
                    joint_values[0] = map_goal.pose.position.x
                    joint_values[1] = map_goal.pose.position.y
                    joint_values[2] = yaw
                    self.base_group.set_joint_value_target(joint_values)
                    
                    rospy.loginfo("Trying with joint values: %s", joint_values)
                    plan_success, plan, _, _ = self.base_group.plan()
                    
                    if plan_success:
                        rospy.loginfo("Base planning successful with joint values approach!")
                        return True
                    else:
                        rospy.logerr("Base planning also failed with joint values approach!")
                else:
                    rospy.logerr("Unexpected joint configuration for base, cannot set joint values")
                
                return False
                
        except Exception as e:
            rospy.logerr("Error planning base movement: %s", str(e))
            return False
    def plan_arm(self):
        virtual_obj_pose = self.transform_pose_to_virtual_baselink(self.object_posestamped, self.base_goal_pose)
        
        # Adjust the x position by 3.5 inches (converted to meters)
        virtual_obj_pose.pose.position.x -= 8.0 * 0.0254  # 1 inch = 0.0254 meters
        
        self.publish_marker("base_link", virtual_obj_pose.pose, "virtual_obj_pose")

        self.arm_group.set_pose_target(virtual_obj_pose)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_planning_time(20.0)
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
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = rospy.Time.now()
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

    def transform_pose_to_virtual_baselink(self, object_pose, base_goal_pose):
        # Convert the base goal pose to transformation matrix
        trans = [self.base_goal_pose.position.x, self.base_goal_pose.position.y, self.base_goal_pose.position.z]
        rot = [
            self.base_goal_pose.orientation.x,
            self.base_goal_pose.orientation.y,
            self.base_goal_pose.orientation.z,
            self.base_goal_pose.orientation.w
        ]
        T_virtual_base = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))

        # Now get the inverse (transform from world to virtual base)
        T_virtual_base_inv = tf.transformations.inverse_matrix(T_virtual_base)

        # Convert object pose to matrix
        obj_trans = [
            object_pose.pose.position.x,
            object_pose.pose.position.y,
            object_pose.pose.position.z
        ]
        obj_rot = [
            object_pose.pose.orientation.x,
            object_pose.pose.orientation.y,
            object_pose.pose.orientation.z,
            object_pose.pose.orientation.w
        ]
        T_obj = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(obj_trans), tf.transformations.quaternion_matrix(obj_rot))

        # Transform the object pose into the virtual base frame
        T_obj_in_virtual_base = tf.transformations.concatenate_matrices(T_virtual_base_inv, T_obj)
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
    rospy.loginfo("MoveIt Goal Node Initalized")
    moveit_grasping = MoveItGrasping()
    rospy.spin()
