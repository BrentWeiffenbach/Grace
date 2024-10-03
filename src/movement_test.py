#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

class movement_test:
    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'movement_test'
        rospy.init_node("movement_test")

        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
        ### When a message is received, call self.update_odometry
        rospy.Subscriber('/odom', Odometry, self.update_odometry)

        ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
        ### When a message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

        # Initialize odometery variables
        self.px = 0
        self.py = 0
        self.yaw = 0

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        ### Make a new Twist message
        twist = Twist(
            linear = Vector3(x = linear_speed),
            angular = Vector3(z = angular_speed)
        )
        ### Publish the message
        self.cmd_vel.publish(twist)
    
    def stop(self):
        self.send_speed(0, 0)
        
    def drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # Save last position
        start_x = self.px
        start_y = self.py

        # Keep driving until distance is reached
        TOLERANCE = 0.05 # m
        K_P = 0.25
        while True:
            current_distance = math.dist([start_x, start_y], [self.px, self.py])
            error = distance - current_distance
            if error < TOLERANCE:
                self.stop()
                break
            else:
                output = K_P * error
                self.send_speed(min(linear_speed, output), 0)
                rospy.sleep(0.05)


    def rotate(self, angle: float, aspeed: float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # Save last yaw
        start_yaw = self.yaw
        target_yaw = start_yaw + angle
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        elif target_yaw < -math.pi:
            target_yaw += 2 * math.pi

        # Keep turning until angle is reached
        TOLERANCE = 0.05 # rad
        K_P = 1
        while True:
            error = target_yaw - self.yaw
            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
                error += 2 * math.pi

            if abs(error) < TOLERANCE:
                self.stop()
                break
            else:
                output = K_P * error
                self.send_speed(0, min(aspeed, output))
                rospy.sleep(0.05)

    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        # Create target pose x, y, and yaw
        target_pose = msg.pose
        target_x = target_pose.position.x
        target_y = target_pose.position.y
        (target_roll, target_pitch, target_yaw) = euler_from_quaternion([
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w,
        ])

        TURN_SPEED = 1.0 # rad / s
        DRIVE_SPEED = 0.5 # m / s

        # Turn to face target
        angle_to_target = math.atan2(target_y - self.py, target_x - self.px)
        angle_to_turn = angle_to_target - self.yaw
        self.rotate(math.atan2(math.sin(angle_to_turn), math.cos(angle_to_turn)), TURN_SPEED)

        # Drive to target
        distance_to_target = math.dist([target_x, target_y], [self.px, self.py])
        self.smooth_drive(distance_to_target, DRIVE_SPEED)

        # Turn to align with target pose
        self.rotate(target_yaw - self.yaw, TURN_SPEED) 

    def update_odometry(self, msg: Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        # update odemetry variables
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(quat_list)

    def smooth_drive(self, distance: float, linear_speed: float):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        # Save last position
        start_x = self.px
        start_y = self.py

        # Keep driving until distance is reached
        TOLERANCE = 0.05 # m
        K_P = 0.4
        MAX_ACCELERATION = 0.1 # m/s^2

        # Calculate the time it takes to accelerate to max velocity
        acceleration_dt = linear_speed / MAX_ACCELERATION

        # If we can't reach max velocity in the given distance, accelerate as much as possible
        halfway_distance = distance / 2
        acceleration_distance = 0.5 * MAX_ACCELERATION * acceleration_dt * acceleration_dt
        if acceleration_distance > halfway_distance:
            acceleration_dt = math.sqrt(halfway_distance / (0.5 * MAX_ACCELERATION))

        start_time = rospy.get_rostime().to_sec()
        while True:
            elapsed_time = (rospy.get_rostime().to_sec() - start_time)
            if elapsed_time < acceleration_dt:
                max_velocity = elapsed_time * MAX_ACCELERATION
            else:
                max_velocity = linear_speed
            
            current_distance = math.dist([start_x, start_y], [self.px, self.py])
            error = distance - current_distance
            if error < TOLERANCE:
                self.stop()
                break
            else:
                output = K_P * error
                self.send_speed(min(max_velocity, output), 0)
                rospy.sleep(0.05)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    movement_test().run()