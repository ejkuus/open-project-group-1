#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time

class CircleAroundTurtleBot(Node):
    def __init__(self):
        super().__init__('circle_around_turtlebot')

        self.drone_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/drone1/odom', self.drone_odom_callback, qos)

        # Subscribe to /threat (assuming geometry_msgs/Pose)
        self.create_subscription(Pose, '/threat', self.threat_callback, 10)

        self.tb_position = None
        self.drone_position = None
        self.threat_position = None
        self.start_time = time.time()
        self.mode = "circle"  # or "approach"

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Node initialized: Drone will circle around TurtleBot.')

    def odom_callback(self, msg):
        self.tb_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def drone_odom_callback(self, msg):
        self.drone_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def threat_callback(self, msg):
        self.threat_position = (msg.position.x, msg.position.y)
        self.mode = "approach"
        self.get_logger().info(f'Threat detected at ({msg.position.x:.2f}, {msg.position.y:.2f}). Switching to approach mode.')

    def control_loop(self):
        if self.tb_position is None or self.drone_position is None:
            return

        if self.mode == "circle":
            self.move_in_circle()
        elif self.mode == "approach":
            self.move_toward_threat()

    def move_in_circle(self):
        radius = 0.3
        angular_speed = 0.6
        gain = 0.8

        time_elapsed = time.time() - self.start_time
        angle = angular_speed * time_elapsed

        center_x, center_y = self.tb_position
        target_x = center_x + radius * math.cos(angle)
        target_y = center_y + radius * math.sin(angle)

        drone_x, drone_y = self.drone_position
        dx = drone_x - center_x
        dy = drone_y - center_y

        current_radius = math.hypot(dx, dy)
        error_radius = current_radius - radius

        if current_radius > 0.001:
            nx = dx / current_radius
            ny = dy / current_radius
        else:
            nx, ny = 0.0, 0.0

        tx = -ny
        ty = nx

        cmd = Twist()
        cmd.linear.x = angular_speed * radius * tx - gain * error_radius * nx
        cmd.linear.y = angular_speed * radius * ty - gain * error_radius * ny
        self.drone_vel_pub.publish(cmd)

    def move_toward_threat(self):
        if self.threat_position is None:
            return

        tb_x, tb_y = self.tb_position
        threat_x, threat_y = self.threat_position
        drone_x, drone_y = self.drone_position

        # Direction vector from TurtleBot to Threat
        dx = threat_x - tb_x
        dy = threat_y - tb_y
        dist = math.hypot(dx, dy)

        if dist < 1e-2:
            self.get_logger().warn('Threat too close to TurtleBot. Holding position.')
            return

        # Normalize and scale to 2m behind TurtleBot
        unit_dx = dx / dist
        unit_dy = dy / dist
        target_x = tb_x + unit_dx * 2.0
        target_y = tb_y + unit_dy * 2.0

        # Move towards target point
        error_x = target_x - drone_x
        error_y = target_y - drone_y
        error_dist = math.hypot(error_x, error_y)

        cmd = Twist()
        if error_dist > 0.1:
            gain = 0.8
            cmd.linear.x = gain * error_x
            cmd.linear.y = gain * error_y
        else:
            self.get_logger().info('Drone reached 2m offset from TurtleBot toward threat.')
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

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
