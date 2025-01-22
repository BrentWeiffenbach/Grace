#!/home/alex/catkin_ws/src/Grace/yolovenv/bin/python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Pose
from gazebo_msgs.srv import SpawnModel, SetModelState
from gazebo_msgs.msg import ModelState
import rospkg

class AddMarker:
    def __init__(self) -> None:
        rospy.init_node('add_marker', anonymous=True)

        self.marker_pub: rospy.Publisher = rospy.Publisher(name="/visualization_marker", data_class=Marker, queue_size=10)

        self.point_sub = rospy.Subscriber(name="transform_node/base_link_frame_point", data_class=PointStamped, callback=self.point_callback)
        self.point: PointStamped
        self.spawned: bool = False


    def point_callback(self, point: PointStamped) -> None:
        self.point: PointStamped = point
        if not self.spawned:
            self.spawn_model()
        else:
            self.move_model()

    def add_marker(self) -> None:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = self.point.point

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.header.frame_id = self.point.header.frame_id
        marker.header.stamp = rospy.Time.now()

        self.marker_pub.publish(marker)
        rospy.sleep(1)

    def get_pose(self) -> Pose:
        pose = Pose()
        pose.position.x = self.point.point.x
        pose.position.y = self.point.point.y
        pose.position.z = self.point.point.z
        return pose

    def spawn_model(self) -> None:
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        try:
            model_path = rospkg.RosPack().get_path("grace") + "/../aws-robomaker-small-house-world/models/aws_robomaker_residential_Ball_01/model.sdf"
            with open(model_path, 'r') as f:
                model_xml = f.read()
            
            
            spawn_pose: Pose = self.get_pose()
            spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            spawn_model("cf_to_wf_marker", model_xml, "", spawn_pose, "world")
            self.spawned = True
        except rospy.ServiceException as err:
            rospy.logerr("Failed to call service: ", err)

    def move_model(self) -> None:
        model_state = ModelState()
        model_state.model_name = "cf_to_wf_marker"

        model_state.pose = self.get_pose()

        try:
            set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            set_model_state(model_state)
        except rospy.ServiceException as err:
            rospy.logerr("Failed to call service: ", err)

    def run(self) -> None:
        rospy.spin()

if __name__ == "__main__":
    try:
        add_marker = AddMarker()
        add_marker.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("AddMarker node terminated.")

    