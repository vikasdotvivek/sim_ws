#!/usr/bin/env python3
#Vikas Vivek
#vvikasvi@purdue.edu
#Roll Number 60
import sys
import os
import numpy as np
import time
import math
import rclpy
from rclpy.node import Node as RCLPyNode
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
from copy import copy, deepcopy

from PIL import Image, ImageOps
import yaml

class MapProcessor():
    def __init__(self):
        self.map_im, self.map_df, self.limits = self.__open_map()
        self.image_array = self.__get_obstacle_map(self.map_im, self.map_df)
        self.inf_map_img_array = np.zeros(self.image_array.shape)

    def __open_map(self):
        yaml_path = '/home/me597/final_project/sim_ws/src/turtlebot3_gazebo/maps/sync_classroom_map'
        f = open(yaml_path +'.yaml', 'r')
        map_df = yaml.safe_load(f)

        map_image_path = map_df['image']
        full_image_path = os.path.join(os.path.dirname(yaml_path + '.yaml'), map_image_path)
        im = Image.open(full_image_path)
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        xmin = map_df['origin'][0]
        xmax = map_df['origin'][0] + im.size[0] * map_df['resolution']
        ymin = map_df['origin'][1]
        ymax = map_df['origin'][1] + im.size[1] * map_df['resolution']
        return im, map_df, [xmin, xmax, ymin, ymax]

    def __get_obstacle_map(self, map_im, map_df):
        img_array = np.reshape(list(map_im.getdata()), (map_im.size[1], map_im.size[0]))
        up_thresh = map_df['occupied_thresh'] * 255
        low_thresh = map_df['free_thresh'] * 255

        for j in range(map_im.size[0]):
            for i in range(map_im.size[1]):
                if img_array[i, j] > up_thresh:
                    img_array[i, j] = 255
                else:
                    img_array[i, j] = 0
        return img_array

    def inflate_map(self, kernel_size):
        kernel = np.ones((kernel_size, kernel_size))
        inflated_map = np.zeros_like(self.image_array)

        for i in range(self.image_array.shape[0]):
            for j in range(self.image_array.shape[1]):
                if self.image_array[i, j] == 255:
                    for dx in range(-kernel_size//2, kernel_size//2 + 1):
                        for dy in range(-kernel_size//2, kernel_size//2 + 1):
                            if 0 <= i+dx < self.image_array.shape[0] and 0 <= j+dy < self.image_array.shape[1]:
                                inflated_map[i+dx, j+dy] = 255
        self.inf_map_img_array = inflated_map

    def draw_path(self, path):
        path_array = copy(self.inf_map_img_array)
        for point in path:
            x, y = point
            path_array[y, x] = 128
        return path_array

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.error_sum = 0
        self.last_error = 0

    def update(self, error):
        p_term = self.kp * error
        self.error_sum += error * self.dt
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.last_error) / self.dt
        output = p_term + i_term + d_term
        self.last_error = error
        return output

    def reset(self):
        self.error_sum = 0
        self.last_error = 0

class DynamicObstacleNavigation(RCLPyNode):
    def __init__(self, node_name='DynamicObstacleNavigation'):
        super().__init__(node_name)
        self.get_logger().info("Initializing Task 3 Navigation Node")
        
        self.map_processor = MapProcessor()
        self.map_processor.inflate_map(kernel_size=5)

        self.path = Path()
        self.goal_pose = None
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.orientation.w = 1.0

        self.pid_controller = PIDController(kp=1.0, ki=0.0, kd=0.1)
        self.obstacle_threshold = 0.5
        self.current_waypoint_index = 0

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        self.create_subscription(LaserScan, '/scan', self.__laser_scan_cbk, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacles_detected = []
        self.scan_data = None

    def __goal_pose_cbk(self, msg):
        self.get_logger().info("Received new goal pose")
        self.goal_pose = msg
        if self.current_pose is not None:
            self.plan_path(self.current_pose)

    def __ttbot_pose_cbk(self, data):
        self.current_pose = data.pose.pose

    def __laser_scan_cbk(self, scan):
        self.scan_data = scan.ranges
        self.obstacles_detected = self.__detect_obstacles(scan)

    def __detect_obstacles(self, scan):
        obstacles = []
        for i, distance in enumerate(scan.ranges):
            if distance < self.obstacle_threshold:
                angle = scan.angle_min + i * scan.angle_increment
                obstacles.append((distance, angle))
        return obstacles

    def plan_path(self, current_pose):
        if self.goal_pose is not None:
            self.get_logger().info("Planning path with obstacle avoidance")
            path = self.dynamic_path_planner(current_pose, self.goal_pose)
            if path:
                self.path = path
                self.current_waypoint_index = 0
                self.get_logger().info("Path planned successfully")

    def dynamic_path_planner(self, current_pose, goal_pose):
        # Reusing A* logic from task2
        self.get_logger().info("Planning path using A* with dynamic obstacle avoidance")

        def heuristic(node, goal):
            return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

        start = (int(current_pose.position.x * 10), int(current_pose.position.y * 10))
        goal = (int(goal_pose.pose.position.x * 10), int(goal_pose.pose.position.y * 10))

        open_set = [start]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            if current == goal:
                self.get_logger().info("Path found!")
                return self.__reconstruct_path(came_from, current)

            open_set.remove(current)

            neighbors = self.__get_neighbors(current)
            for neighbor in neighbors:
                tentative_g_score = g_score[current] + heuristic(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.append(neighbor)

        self.get_logger().warn("No path found")
        return None

    def __reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return self.map_processor.draw_path(path)

    def __get_neighbors(self, node):
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (1, 1), (-1, 1), (1, -1)
        ]
        neighbors = []
        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            if not self.__is_in_collision(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def __is_in_collision(self, node):
        x, y = node
        if x < 0 or y < 0 or x >= self.map_processor.image_array.shape[1] or y >= self.map_processor.image_array.shape[0]:
            return True
        return self.map_processor.inf_map_img_array[y, x] == 255

    def avoid_obstacles(self, vehicle_pose):
        for obstacle in self.obstacles_detected:
            distance, angle = obstacle
            if distance < self.obstacle_threshold:
                self.get_logger().warn(f"Obstacle detected at distance {distance}, angle {angle}.")
                return True
        return False

    def compute_velocity(self, vehicle_pose, target_pose):
        dx = target_pose.pose.position.x - vehicle_pose.position.x
        dy = target_pose.pose.position.y - vehicle_pose.position.y

        distance = math.sqrt(dx ** 2 + dy ** 2)
        heading = math.atan2(dy, dx)

        current_heading = self.__get_yaw_from_quaternion(vehicle_pose.orientation)
        heading_error = heading - current_heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        linear_speed = self.pid_controller.update(distance)
        angular_speed = max(-1.0, min(1.0, 2.0 * heading_error))

        return linear_speed, angular_speed

    def move_ttbot(self, linear_speed, angular_speed):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.path.poses:
                vehicle_pose = self.current_pose

                if self.current_waypoint_index < len(self.path.poses):
                    target_pose = self.path.poses[self.current_waypoint_index]
                    
                    if self.avoid_obstacles(vehicle_pose):
                        self.get_logger().info("Replanning due to obstacles")
                        self.plan_path(vehicle_pose)
                        continue

                    linear_speed, angular_speed = self.compute_velocity(vehicle_pose, target_pose)
                    self.move_ttbot(linear_speed, angular_speed)

                    distance = self.__calculate_distance(vehicle_pose.position, target_pose.pose.position)
                    if distance < 0.1:
                        self.current_waypoint_index += 1

                else:
                    self.get_logger().info("Path complete")
                    self.move_ttbot(0.0, 0.0)
                    break

    def __calculate_distance(self, position1, position2):
        dx = position1.x - position2.x
        dy = position1.y - position2.y
        return math.sqrt(dx**2 + dy**2)

    def __get_yaw_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    print("Task3 Dynamic Obstacle Navigation is running!")

    rclpy.init(args=args)
    nav = DynamicObstacleNavigation()

    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
