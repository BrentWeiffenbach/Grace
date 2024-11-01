#!/usr/bin/env python2.7
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult
import numpy as np
from scipy.ndimage import label, center_of_mass, generate_binary_structure

class FrontierSearch:
    def __init__(self):
        rospy.init_node('frontier_search', anonymous=True)
        rospy.loginfo("FrontierSearch node initialized")
        
        self.map_data = None
        self.closest_centroid = None
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.frontiers_pub = rospy.Publisher('/frontier_search/frontiers', OccupancyGrid, queue_size=10)
        self.frontier_centroid_pub = rospy.Publisher('/frontier_search/centroid', OccupancyGrid, queue_size=10)
        self.nav_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.publish_nav_goal_callback)
        
    def map_callback(self, data):
        rospy.loginfo("Map data received")
        self.map_data = data

    def find_frontiers(self, map_data):
        unexplored_cells = []
        width = map_data.info.width
        height = map_data.info.height
        data = map_data.data

        def is_explored_and_free(x, y):
            index = x + y * width
            return data[index] == 0  # 0 indicates free space in OccupancyGrid

        for y in range(height):
            for x in range(width):
                index = x + y * width
                if data[index] == -1:  # -1 indicates unexplored cell in OccupancyGrid
                    # Check if any adjacent cell is explored and free
                    if (x > 0 and is_explored_and_free(x - 1, y)) or \
                       (x < width - 1 and is_explored_and_free(x + 1, y)) or \
                       (y > 0 and is_explored_and_free(x, y - 1)) or \
                       (y < height - 1 and is_explored_and_free(x, y + 1)):
                            unexplored_cells.append(index)

        # Create an OccupancyGrid message for the frontiers
        frontiers_grid = OccupancyGrid()
        frontiers_grid.header = map_data.header
        frontiers_grid.info = map_data.info
        frontiers_grid.data = [-1] * (width * height)  # Initialize with -1 (unexplored)

        for index in unexplored_cells:
            frontiers_grid.data[index] = 0  # Mark frontier cells as 0 (free space)
        # if frontiers_grid.data is greater than 10 return it
        if len(unexplored_cells) > 10:
            return frontiers_grid

    def find_centroids(self, mapdata, frontiers):
        width = frontiers.info.width
        height = frontiers.info.height
        data = np.array(frontiers.data).reshape((height, width))

        # Create a 3x3 structuring element for 8-connectivity
        structuring_element = generate_binary_structure(2, 2)

        # Label connected components using the structuring element
        labeled_array, num_features = label(data == 0, structure=structuring_element)

        # Calculate centroids
        centroids = center_of_mass(data == 0, labeled_array, range(1, num_features + 1))

        # Create an OccupancyGrid message for the centroids
        centroids_grid = OccupancyGrid()
        centroids_grid.header = frontiers.header
        centroids_grid.info = frontiers.info
        centroids_grid.data = [-1] * (width * height)  # Initialize with -1 (unexplored)

        for centroid in centroids:
            x, y = int(centroid[1]), int(centroid[0])
            index = x + y * width
            centroids_grid.data[index] = 0  # Mark centroid cells as 0 (free space)

        # Use BFS to find the closest free space for each centroid
        def bfs_find_free_space(start_x, start_y):
            queue = [(start_x, start_y)]
            visited = set()
            visited.add((start_x, start_y))

            while queue:
                x, y = queue.pop(0)
                index = x + y * width
                if mapdata.data[index] == 0:  # Free space
                    return x, y

                # Check all 8 possible directions (N, S, E, W, NE, NW, SE, SW)
                directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
                for dx, dy in directions:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                        queue.append((nx, ny))
                    visited.add((nx, ny))

            return start_x, start_y  # Return the original position if no free space is found

        # Translate centroids to the closest free space
        translated_centroids = []
        for centroid in centroids:
            x, y = int(centroid[1]), int(centroid[0])
            free_x, free_y = bfs_find_free_space(x, y)
            translated_centroids.append((free_y, free_x))  # Append as (row, col) for consistency

        # Update centroids_grid with translated centroids
        centroids_grid.data = [-1] * (width * height)  # Reinitialize with -1 (unexplored)
        for centroid in translated_centroids:
            x, y = centroid[1], centroid[0]
            index = x + y * width
            centroids_grid.data[index] = 0  # Mark translated centroid cells as 0 (free space)
        
        return centroids_grid, translated_centroids

    def perform_360_rotation(self):
        twist = Twist()
        twist.angular.z = 0.5  # Adjust the angular speed as needed
        duration = 2 * np.pi / twist.angular.z  # Time to complete 360 degrees

        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Stop the rotation
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Completed 360-degree rotation")

    def find_closest_centroid(self, centroids):
        # Assuming the robot's current position is at the center of the map
        robot_x = self.map_data.info.origin.position.x + (self.map_data.info.width * self.map_data.info.resolution) / 2
        robot_y = self.map_data.info.origin.position.y + (self.map_data.info.height * self.map_data.info.resolution) / 2

        closest_centroid = None
        min_distance = float('inf')

        for centroid in centroids:
            x, y = centroid[1] * self.map_data.info.resolution, centroid[0] * self.map_data.info.resolution
            distance = np.sqrt((x - robot_x) ** 2 + (y - robot_y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_centroid = (x, y)

        return closest_centroid

    def publish_nav_goal_callback(self, event):
        if self.closest_centroid:
            # Convert grid cell coordinates to world coordinates
            map_origin_x = self.map_data.info.origin.position.x
            map_origin_y = self.map_data.info.origin.position.y
            resolution = self.map_data.info.resolution

            goal_x = map_origin_x + self.closest_centroid[0]
            goal_y = map_origin_y + self.closest_centroid[1]

            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.position.z = 0
            goal.pose.orientation.w = 1.0  # No rotation

            self.nav_goal_pub.publish(goal)
            # rospy.loginfo("Published navigation goal: ({}, {})".format(goal_x, goal_y))

    def run(self):
        # self.perform_360_rotation()  # Perform the 360-degree rotation before starting the frontier search
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.map_data:
                frontiers = self.find_frontiers(self.map_data)
                if frontiers:
                    centroids_grid, centroids = self.find_centroids(self.map_data, frontiers)
                    self.frontiers_pub.publish(frontiers)
                    self.frontier_centroid_pub.publish(centroids_grid)
                    self.closest_centroid = self.find_closest_centroid(centroids)
                # self.publish_nav_goal(closest_centroid)
                # rospy.loginfo("Frontiers, centroids, and navigation goal published")
            rate.sleep()

if __name__ == '__main__':
    try:
        frontier_search = FrontierSearch()
        frontier_search.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("FrontierSearch node terminated.")