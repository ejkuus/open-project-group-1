#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import time


class CircleAroundTurtleBot(Node):
    def __init__(self):
        super().__init__('circle_around_turtlebot')

        # Velocity commands sent to the Tello drone (linear x/y in drone frame)
        self.drone_vel_pub = self.create_publisher(
            Twist, '/drone1/cmd_vel', 10)

        # TurtleBot pose (used as the orbit centre)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Drone pose 
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry, '/drone1/odom', self.drone_odom_callback, qos)

        laser_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan, '/scan', self.laserscan_callback, laser_qos)

        self.tb_position = None          # Latest TurtleBot (x, y)
        self.drone_position = None       # Latest Drone (x, y)

        self.start_time = time.time()    # Used to make the orbit angle progress
        self.state = "orbit"             # Finite-state machine mode
        self.state_start_time = None     # Timestamp when current state started
        self.intruder_handled = False    # Only intimidate once per run

        # Intruder detection parameters 
        self.intruder_detected = False
        self.intruder_position = None
        self.detection_range = 1.5       # Laser hit distance to trigger (m)
        self.hover_duration = 3.0        # How long to hover at intruder (s)

        # Main control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            'Drone follower node started with LaserScan-based intruder logic.')

    def odom_callback(self, msg):
        """Store latest TurtleBot position (x, y) in world frame."""
        self.tb_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def drone_odom_callback(self, msg):
        """Store latest drone position (x, y) in world frame."""
        self.drone_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def laserscan_callback(self, msg):
        if self.tb_position is None:
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges

        cone = math.radians(60)          # Total cone width
        found = False
        min_r = float('inf')
        best_angle = 0.0

        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_inc
            if -cone / 2 <= angle <= cone / 2 and 0.05 < r < self.detection_range:
                found = True
                if r < min_r:
                    min_r = r
                    best_angle = angle

        if found:
            # Rough world-frame estimate of intruder position
            tb_x, tb_y = self.tb_position
            ix = tb_x + min_r * math.cos(best_angle)
            iy = tb_y + min_r * math.sin(best_angle)
            self.intruder_detected = True
            self.intruder_position = (ix, iy)
            self.get_logger().info(
                f"LaserScan: intruder at ≈({ix:.2f}, {iy:.2f})")

    # Main control loop                                                     #
    
    def control_loop(self):
        if self.tb_position is None or self.drone_position is None:
            return  # Wait until both positions are known

        drone_x, drone_y = self.drone_position

        if self.state == "orbit":
            # If a new intruder is detected and hasn’t been handled yet
            if (self.intruder_detected and not self.intruder_handled
                    and self.intruder_position):
                ix, iy = self.intruder_position
                dist = math.hypot(ix - drone_x, iy - drone_y)
                if dist <= self.detection_range + 3.0:  # Allow some margin
                    self.get_logger().info("Switching to INTERCEPT.")
                    self.state = "intercept"
                    self.state_start_time = time.time()
                    self.intruder_detected = False  # Reset flag
                    # Momentarily stop to avoid big overshoot
                    self.send_velocity(0.0, 0.0)
                    return
            # Normal circular motion
            self.orbit_turtlebot()

        elif self.state == "intercept":
            self.fly_to_target(self.intruder_position)
            ix, iy = self.intruder_position
            dist = math.hypot(ix - drone_x, iy - drone_y)
            if dist < 0.3:
                self.get_logger().info("Arrived at intruder – HOVER.")
                self.state = "hover"
                self.state_start_time = time.time()

        elif self.state == "hover":
            self.send_velocity(0.0, 0.0)
            if time.time() - self.state_start_time >= self.hover_duration:
                self.get_logger().info("Hover done – RETURN to orbit.")
                self.state = "return"
                self.state_start_time = time.time()
                self.intruder_handled = True  # Only once per run


        elif self.state == "return":
            # Create a temporary orbit point to head back to
            t = time.time() - self.start_time
            angle = 0.6 * t
            cx, cy = self.tb_position
            orbit_x = cx + 0.4 * math.cos(angle)
            orbit_y = cy + 0.4 * math.sin(angle)

            self.fly_to_target((orbit_x, orbit_y))

            dist = math.hypot(orbit_x - drone_x, orbit_y - drone_y)
            if dist < 0.3:
                self.get_logger().info("Back on circular path.")
                self.state = "orbit"


    def orbit_turtlebot(self):
        """Generate smooth circular motion around the TurtleBot."""
        radius = 0.6
        omega = 0.4
        gain = 0.2

        t = time.time() - self.start_time
        angle = omega * t

        cx, cy = self.tb_position
        target_x = cx + radius * math.cos(angle)
        target_y = cy + radius * math.sin(angle)

        dx, dy = (self.drone_position[0] - cx,
                  self.drone_position[1] - cy)
        curr_r = math.hypot(dx, dy)
        err_r = curr_r - radius

        if curr_r > 1e-3:
            nx, ny = dx / curr_r, dy / curr_r
        else:
            nx, ny = 0.0, 0.0

        tx, ty = -ny, nx  # Tangent unit vector

        vx = omega * radius * tx - gain * err_r * nx
        vy = omega * radius * ty - gain * err_r * ny
        self.send_velocity(vx, vy)

    def fly_to_target(self, target_pos):
        """Simple proportional controller to reach (x, y)."""
        tx, ty = target_pos
        dx = tx - self.drone_position[0]
        dy = ty - self.drone_position[1]

        gain = 0.1
        self.send_velocity(gain * dx, gain * dy)

    def send_velocity(self, vx, vy):
        """Publish a Twist with horizontal velocity (vx, vy)."""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0
        self.drone_vel_pub.publish(cmd)


# Boiler-plate ROS 2 node start-up                                          #
def main(args=None):
    rclpy.init(args=args)
    node = CircleAroundTurtleBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
