#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

class FrontierSearch:
    def __init__(self):
        rospy.init_node('frontier_search', anonymous=True)
        rospy.loginfo("FrontierSearch node initialized")
        
        self.map_data = None
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.frontiers_pub = rospy.Publisher('/frontier_search/frontiers', OccupancyGrid, queue_size=10)
        
    def map_callback(self, data):
        rospy.loginfo("Map data received")
        self.map_data = data

    def find_frontiers(self, map_data):
        # Implement your frontier search algorithm here
        # For now, we will just return the input map_data as a placeholder
        rospy.loginfo("Finding frontiers")
        return map_data

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.map_data:
                frontiers = self.find_frontiers(self.map_data)
                self.frontiers_pub.publish(frontiers)
                rospy.loginfo("Frontiers published")
            rate.sleep()

if __name__ == '__main__':
    try:
        frontier_search = FrontierSearch()
        frontier_search.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("FrontierSearch node terminated.")