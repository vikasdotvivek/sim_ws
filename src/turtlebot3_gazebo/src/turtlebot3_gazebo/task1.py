import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import math
import random

class AutonomousMapper(Node):
    def __init__(self):
        super().__init__('autonomous_mapper')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Initialize variables
        self.laser_data = None
        self.map_data = None
        self.covered_cells = set()
        self.state = 'wall_following'

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.control_loop)

    def laser_callback(self, msg):
        self.laser_data = msg

    def map_callback(self, msg):
        self.map_data = msg
        self.update_coverage(msg)

    def update_coverage(self, map_msg):
        """Track covered grid cells using the map's occupancy grid."""
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        width = map_msg.info.width
        height = map_msg.info.height

        for y in range(height):
            for x in range(width):
                idx = x + y * width
                if map_msg.data[idx] == 0:  # Free space
                    grid_x = origin_x + x * resolution
                    grid_y = origin_y + y * resolution
                    self.covered_cells.add((grid_x, grid_y))

    def control_loop(self):
        """Main control loop for autonomous navigation."""
        if self.laser_data is None:
            return

        twist = Twist()

        # Wall following logic
        if self.state == 'wall_following':
            if min(self.laser_data.ranges) < 0.5:  # Too close to wall
                twist.angular.z = 0.5
            else:
                twist.linear.x = 0.2

            if random.random() < 0.1:  # Occasionally switch to random bouncing
                self.state = 'random_bouncing'

        # Random bouncing logic
        elif self.state == 'random_bouncing':
            twist.linear.x = 0.2
            twist.angular.z = random.uniform(-1.0, 1.0)

            if random.random() < 0.1:  # Occasionally switch to wall following
                self.state = 'wall_following'

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

