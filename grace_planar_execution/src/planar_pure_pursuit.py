import math

def find_lookahead_point(path, current_position, lookahead_distance, last_index):
    """
    Finds the lookahead point on the path given the robot's current position.
    
    Args:
        path: List of (x, y) tuples representing the global path.
        current_position: Tuple (x, y) for the current robot position.
        lookahead_distance: The desired lookahead distance.
        last_index: The index in the path from which to begin the search.
    
    Returns:
        A tuple (lookahead_point, new_index) where lookahead_point is (x, y).
        If no point is found at the required distance, the final point is returned.
    """
    for i in range(last_index, len(path)):
        p = path[i]
        d = math.hypot(p[0] - current_position[0], p[1] - current_position[1])
        if d >= lookahead_distance:
            return (p[0], p[1]), i
    final_point = path[-1]
    return (final_point[0], final_point[1]), len(path) - 1

def pure_pursuit_control(current_pose, target_point, lookahead_distance, linear_velocity):
    """
    Computes the angular velocity command using the pure pursuit algorithm.
    
    Args:
        current_pose: Tuple (x, y, yaw) representing current robot pose.
        target_point: Tuple (x, y) representing the lookahead target.
        lookahead_distance: Lookahead distance (meters).
        linear_velocity: Constant linear velocity (m/s).
    
    Returns:
        Angular velocity command (radians/s).
    """
    dx = target_point[0] - current_pose[0]
    dy = target_point[1] - current_pose[1]
    
    # Transform the error into the robot's frame.
    error_y = -math.sin(current_pose[2]) * dx + math.cos(current_pose[2]) * dy
    
    curvature = 2 * error_y / (lookahead_distance ** 2)
    angular_velocity = linear_velocity * curvature
    return angular_velocity