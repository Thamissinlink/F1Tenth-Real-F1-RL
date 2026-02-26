#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

class CenterFollower(Node):
    def __init__(self):
        super().__init__('center_follower')
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Tuned for centering on Vegas
        self.desired_center = 0.0  # 0 = center (steer to balance left/right distances)
        self.speed = 3.0
        self.kp = 0.3
        self.kd = 0.8
        self.look_angle_deg = 90   # cone for each side (forward-left and forward-right)
        self.min_valid_beams = 20
        self.deadband = 0.2

        self.previous_error = 0.0

        self.get_logger().info('Center Follower started — staying between walls on Vegas')

    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_inc = msg.angle_increment
        min_angle = msg.angle_min

        # Left cone (0° to -90°)
        left_start_angle = 0.0
        left_end_angle = -math.radians(self.look_angle_deg)
        left_start_idx = int((left_start_angle - min_angle) / angle_inc)
        left_end_idx = int((left_end_angle - min_angle) / angle_inc)
        left_ranges = ranges[left_start_idx:left_end_idx]
        left_valid = [r for r in left_ranges if msg.range_min < r < msg.range_max]
        left_dist = min(left_valid) if len(left_valid) >= self.min_valid_beams else math.inf

        # Right cone (0° to +90°)
        right_start_angle = 0.0
        right_end_angle = math.radians(self.look_angle_deg)
        right_start_idx = int((right_start_angle - min_angle) / angle_inc)
        right_end_idx = int((right_end_angle - min_angle) / angle_inc)
        right_ranges = ranges[right_start_idx:right_end_idx]
        right_valid = [r for r in right_ranges if msg.range_min < r < msg.range_max]
        right_dist = min(right_valid) if len(right_valid) >= self.min_valid_beams else math.inf

        # Error = left_dist - right_dist (positive = steer right, negative = steer left)
        error = left_dist - right_dist
        if abs(error) < self.deadband:
            error = 0.0
        derivative = error - self.previous_error
        self.previous_error = error
        steering_angle = self.kp * error + self.kd * derivative

        steering_angle = max(min(steering_angle, 0.34), -0.34)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle
        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CenterFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
