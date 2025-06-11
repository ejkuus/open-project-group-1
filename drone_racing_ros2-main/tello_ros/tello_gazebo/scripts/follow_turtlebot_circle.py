#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time

class CircleAroundTurtleBot(Node):
    def __init__(self):
        super().__init__('circle_around_turtlebot')

        # Publisher to drone velocity command topic
        self.drone_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)

        # Subscriber to TurtleBot odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber to drone odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/drone1/odom', self.drone_odom_callback, qos)


        self.tb_position = None      # (x, y)
        self.drone_position = None   # (x, y)
        self.start_time = time.time()

        self.timer = self.create_timer(0.1, self.move_in_circle)  # 10 Hz
        self.get_logger().info('Node initialized: Drone will circle around TurtleBot.')

    def odom_callback(self, msg):
        """Callback to update TurtleBot's position."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.tb_position = (x, y)
        # self.get_logger().info(f"TurtleBot position: x={x:.2f}, y={y:.2f}")

    def drone_odom_callback(self, msg):
        """Callback to update drone's own position."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.drone_position = (x, y)
        # self.get_logger().info(f"Drone position: x={x:.2f}, y={y:.2f}")

    def move_in_circle(self):
        """Control logic to move the drone in a circle around the TurtleBot."""
        if self.tb_position is None or self.drone_position is None:
            return

        radius = 0.3  # meters
        angular_speed = 0.6  # radians/sec
        gain = 1  # Proportional control gain

        time_elapsed = time.time() - self.start_time
        angle = angular_speed * time_elapsed

        # Circle target point around TurtleBot
        center_x, center_y = self.tb_position
        target_x = center_x + radius * math.cos(angle)
        target_y = center_y + radius * math.sin(angle)

        # Vector from drone to TurtleBot (radial vector)
        drone_x, drone_y = self.drone_position
        dx = drone_x - center_x
        dy = drone_y - center_y

        current_radius = math.hypot(dx, dy)
        error_radius = current_radius - radius

        # Normalize radial vector
        if current_radius > 0.001:
            nx = dx / current_radius
            ny = dy / current_radius
        else:
            nx, ny = 0.0, 0.0

        # Tangent vector (perpendicular to radial)
        tx = -ny
        ty = nx

        # Compose final velocity: tangent + radial correction
        cmd = Twist()
        cmd.linear.x = angular_speed * radius * tx - gain * error_radius * nx
        cmd.linear.y = angular_speed * radius * ty - gain * error_radius * ny
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0

        self.drone_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CircleAroundTurtleBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
