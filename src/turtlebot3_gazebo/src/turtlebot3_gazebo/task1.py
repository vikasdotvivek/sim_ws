#Vikas Vivek
#vvikasvi@purdue.edu
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
        self.state = 'exploring'

        # Timer for periodic updates
        self.timer = self.create_timer(0.3, self.control_loop)

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
        if self.laser_data is None or self.map_data is None:
            return

        twist = Twist()
        SAFE_DISTANCE = 0.50 # threshold for walls or obstacles

        # Divide laser ranges into regions
        front_distance = min(min(self.laser_data.ranges[0:15] + self.laser_data.ranges[-15:]), float('inf'))
        left_distance = min(self.laser_data.ranges[70:114])  # Find the minimum distance in the left region
        right_distance = min(self.laser_data.ranges[-114:-70])  # Find the minimum distance in the right region

        #self.get_logger().info(f"{front_distance:.2f} F")
        #self.get_logger().info(f"{left_distance:.2f} L")
        #self.get_logger().info(f"{right_distance:.2f} R")

        # Wall avoidance logic
        if front_distance < SAFE_DISTANCE:
            #twist.linear.x = -0.3
            
            if left_distance > right_distance:
                twist.angular.z = 2.25*(right_distance - SAFE_DISTANCE)  # Turn left
                self.get_logger().info(f"Obstacle ahead, turn left: {(right_distance - SAFE_DISTANCE):.2f}")
                if  ((right_distance - SAFE_DISTANCE) < 0.1):
                    twist.angular.z = 0.2
                #twist.linear.x = -0.1
            else:
                twist.angular.z = -2.25*(left_distance - SAFE_DISTANCE)  # Turn right
                self.get_logger().info(f"Obstacle ahead, turn right: {(left_distance - SAFE_DISTANCE):.2f}")
                #twist.linear.x = -0.1
                if  ((left_distance - SAFE_DISTANCE) < 0.1):
                    twist.angular.z = -0.2

        elif left_distance < SAFE_DISTANCE:
            
            twist.angular.z = -1.0*(SAFE_DISTANCE - left_distance) #turn right
            twist.linear.x = 0.1
            self.get_logger().info(f"Too close to the left: {left_distance:.2f} meters")

        elif right_distance < SAFE_DISTANCE:
            
            twist.angular.z = 1.0*(SAFE_DISTANCE - right_distance)     #turn left
            twist.linear.x = 0.1
            self.get_logger().info(f"Too close to the right: {right_distance:.2f} meters")
            
        else:
            # Free to move forward

            twist.linear.x = 0.7*(front_distance - SAFE_DISTANCE)   #p-controller for dynamic speed control
            #twist.linear.x = 0.4
            twist.angular.z = 0.0
            self.get_logger().info("Moving forward")

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()