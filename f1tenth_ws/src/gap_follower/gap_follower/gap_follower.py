import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower')
        # Publisher to /drive
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # Subscriber to /scan
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # --- Adjustable parameters ---
        self.speed = 4.0            # Speed of the car
        self.bubble_radius = 60     # Rays to clear around closest obstacle
        self.forward_angle = 135    # Degrees of forward cone to consider

    # Preprocess scan: clear a bubble around closest obstacle
    def preprocess_scan(self, ranges):
        scan = np.array(ranges)
        min_idx = np.argmin(scan)

        start = max(0, min_idx - self.bubble_radius)
        end = min(len(scan), min_idx + self.bubble_radius + 1)
        scan[start:end] = 0.0

        return scan

    # Find the largest free gap
    def find_max_gap(self, free_space):
        masked = np.where(free_space > 0.5, 1, 0)
        padded = np.concatenate(([0], masked, [0]))
        diffs = np.diff(padded)
        starts = np.where(diffs == 1)[0]
        ends = np.where(diffs == -1)[0]

        if len(starts) == 0:
            return None, None

        lengths = ends - starts
        max_idx = np.argmax(lengths)
        return starts[max_idx], ends[max_idx]

    # Find best point inside the gap
    def find_best_point(self, start, end, full_scan):
        if start is None:
            return None
        gap_scan = full_scan[start:end]
        if len(gap_scan) == 0:
            return start
        return start + np.argmax(gap_scan)

    # Scan callback
    def scan_callback(self, msg):
        processed = self.preprocess_scan(msg.ranges)

        num_beams = len(msg.ranges)
        center = num_beams // 2
        half_cone = int((self.forward_angle / 270.0) * num_beams / 2)
        forward_scan = processed[center - half_cone : center + half_cone]
        forward_start = center - half_cone

        start_i, end_i = self.find_max_gap(forward_scan)

        if start_i is None:
            steering_angle = 0.0
        else:
            best_i = self.find_best_point(start_i, end_i, forward_scan)
            global_i = forward_start + best_i
            angle = msg.angle_min + global_i * msg.angle_increment
            steering_angle = angle

        # Limit steering
        steering_angle = max(min(steering_angle, 0.34), -0.34)

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = steering_angle
        self.publisher.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GapFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
