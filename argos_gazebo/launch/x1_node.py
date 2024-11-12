#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import sqrt
import random  # Fire detection position simulation (if no YOLOv8 integrated)

class X1Node(Node):
    def __init__(self):
        super().__init__('x1_node')
        self.get_logger().info("X1 Config5 Node has started.")

        # Set up the velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Set up the goal_position publisher to send coordinates to drones
        self.goal_publisher = self.create_publisher(Point, '/quadrotor/goal_position', 10)

        # Set up the odometry subscriber to update robot's current position
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer for controlling robot's movement
        self.timer = self.create_timer(0.1, self.move_robot)

        # Robot's current position information
        self.current_position = Point()
        self.goal_position = None
        self.fire_detected = False

        # Movement control variables
        self.linear_speed = 0.2  # Linear speed
        self.angular_speed = 0.0  # Angular speed
        self.stop = False

    def odom_callback(self, msg):
        # Update robot's current position
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = msg.pose.pose.position.z
        self.get_logger().info(f"Current Position - x: {self.current_position.x}, y: {self.current_position.y}")

        # Fire detection simulation
        if not self.fire_detected:
            self.detect_fire()

    def move_robot(self):
        # Set robot speed
        twist = Twist()

        if not self.stop:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Publishing velocity - linear: {twist.linear.x}, angular: {twist.angular.z}")

    def detect_fire(self):
        # Fire detection simulation
        if random.random() < 0.05:  # Simulate fire detection with a probability (remove when YOLO is integrated)
            fire_position = Point(
                x=self.current_position.x + random.uniform(-5, 5),
                y=self.current_position.y + random.uniform(-5, 5),
                z=0.0  # Ground level
            )
            self.publish_fire_location(fire_position)

    def publish_fire_location(self, fire_position):
        # Publish fire location to the goal_position topic for the drones
        self.get_logger().info(f"Fire detected at - x: {fire_position.x}, y: {fire_position.y}")
        self.goal_publisher.publish(fire_position)
        self.fire_detected = True
        self.stop_robot()

    def stop_robot(self):
        # Stop the robot when fire is detected
        self.stop = True
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        self.get_logger().info("Robot stopped due to fire detection.")

def main(args=None):
    rclpy.init(args=args)
    x1_node = X1Node()
    rclpy.spin(x1_node)
    x1_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
