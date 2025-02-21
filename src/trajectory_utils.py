import math
from geometry_msgs.msg import Pose, Transform

def quaternion_to_yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def transform_relative_pose(initial_pose, relative_transform):
    init_x = initial_pose.position.x
    init_y = initial_pose.position.y
    init_yaw = quaternion_to_yaw(initial_pose.orientation)
    
    rel_x = relative_transform.translation.x
    rel_y = relative_transform.translation.y
    global_x = init_x + (rel_x * math.cos(init_yaw) - rel_y * math.sin(init_yaw))
    global_y = init_y + (rel_x * math.sin(init_yaw) + rel_y * math.cos(init_yaw))
    
    rel_yaw = quaternion_to_yaw(relative_transform.rotation)
    global_yaw = init_yaw + rel_yaw
    
    return (global_x, global_y, global_yaw)

def extract_path_from_trajectory(points, initial_pose):
    path = []
    for pt in points:
        global_pose = transform_relative_pose(initial_pose, pt.transforms[0])
        path.append(global_pose)
    return path