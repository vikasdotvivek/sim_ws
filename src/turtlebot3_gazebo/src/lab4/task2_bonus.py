#!/usr/bin/env python3
#Vikas Vivek
#vvikasvi@purdue.edu
#Roll Number 60
import sys
import pandas as pd

import os
import numpy as np
import time
import math
import rclpy
from rclpy.node import Node as RCLPyNode
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from copy import copy, deepcopy

from PIL import Image, ImageOps
import yaml
from nav_msgs.msg import OccupancyGrid

class Map():
    def __init__(self):
        self.map_im, self.map_df, self.limits = self.__open_map()
        self.image_array = self.__get_obstacle_map(self.map_im, self.map_df)

    def __open_map(self):
        yaml_path = '/home/me597/final_project/sim_ws/src/turtlebot3_gazebo/maps/sync_classroom_map'  # hardcoded path
        f = open(yaml_path +'.yaml', 'r')
        map_df = pd.json_normalize(yaml.safe_load(f))
        
        map_image_path = map_df.image[0]
        full_image_path = os.path.join(os.path.dirname(yaml_path + '.yaml'), map_image_path)
        im = Image.open(full_image_path)
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]
        return im, map_df, [xmin, xmax, ymin, ymax]

    def __get_obstacle_map(self, map_im, map_df):
        img_array = np.reshape(list(self.map_im.getdata()), (self.map_im.size[1], self.map_im.size[0]))
        up_thresh = self.map_df.occupied_thresh[0] * 255 * 2
        low_thresh = self.map_df.free_thresh[0] * 255 * 2

        for j in range(self.map_im.size[0]):
            for i in range(self.map_im.size[1]):
                if img_array[i, j] > up_thresh:
                    img_array[i, j] = 255 * 2
                else:
                    img_array[i, j] = 0
        return img_array

class RRT():
    def __init__(self, start, goal, map_array, map_limits, max_iterations=1000, step_size=5):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.map_array = map_array
        self.map_limits = map_limits
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.tree = [self.start]
        self.parents = [-1]

    def __is_in_collision(self, point):
        x, y = int(point[0]), int(point[1])
        return self.map_array[y, x] != 0

    def __get_random_point(self):
        x = np.random.uniform(self.map_limits[0], self.map_limits[1])
        y = np.random.uniform(self.map_limits[2], self.map_limits[3])
        return np.array([x, y])

    def __nearest_neighbor(self, point):
        distances = [np.linalg.norm(point - node) for node in self.tree]
        nearest_index = np.argmin(distances)
        return nearest_index, self.tree[nearest_index]

    def __steer(self, from_point, to_point):
        direction = to_point - from_point
        length = np.linalg.norm(direction)
        if length > self.step_size:
            direction = direction / length * self.step_size
        return from_point + direction

    def plan(self):
        for _ in range(self.max_iterations):
            random_point = self.__get_random_point()
            nearest_index, nearest_point = self.__nearest_neighbor(random_point)
            new_point = self.__steer(nearest_point, random_point)

            if not self.__is_in_collision(new_point):
                self.tree.append(new_point)
                self.parents.append(nearest_index)

                if np.linalg.norm(new_point - self.goal) <= self.step_size:
                    self.tree.append(self.goal)
                    self.parents.append(len(self.tree) - 2)
                    return self.__reconstruct_path()

        return None  # Return None if planning fails

    def __reconstruct_path(self):
        path = []
        current_index = len(self.tree) - 1
        while current_index != -1:
            path.append(self.tree[current_index])
            current_index = self.parents[current_index]
        path.reverse()
        return path

class Navigation(RCLPyNode):
    def __init__(self, node_name='Navigation'):
        super().__init__(node_name)
        self.get_logger().info("Starting Navigation initialization")
        self.current_waypoint_index = 0
        self.path = Path()
        self.goal_pose = None
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.orientation.w = 1.0
        self.map = Map()

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def __goal_pose_cbk(self, msg):
        self.get_logger().info("Received new goal pose")
        self.goal_pose = msg
        if self.current_pose is not None:
            self.plan_path(self.current_pose)

    def __ttbot_pose_cbk(self, data):
        self.current_pose = data.pose.pose

    def plan_path(self, current_pose):
        if self.goal_pose is not None:
            self.get_logger().info("Planning path")
            path = self.rrt_path_planner(current_pose, self.goal_pose)
            if path:
                self.path = path
                self.current_waypoint_index = 0
                self.get_logger().info("Path planned successfully")

    def rrt_path_planner(self, current_pose, goal_pose):
        self.get_logger().info("Converting poses to grid coordinates...")
        start = self.__pose_to_grid(current_pose)
        goal = self.__pose_to_grid(goal_pose)

        self.get_logger().info(f"Start: {start}, Goal: {goal}")

        rrt = RRT(start, goal, self.map.image_array, self.map.limits)
        path = rrt.plan()

        if path is None:
            self.get_logger().warn("Path planning failed.")
            return None

        path_msg = Path()
        for point in path:
            pose = self.__grid_to_pose(point)
            path_msg.poses.append(pose)

        return path_msg

    def __pose_to_grid(self, pose):
        if hasattr(pose, 'pose'):
            x_grid = int((pose.pose.position.x - self.map.limits[0]) / self.map.map_df.resolution[0])
            y_grid = int((pose.pose.position.y - self.map.limits[2]) / self.map.map_df.resolution[0])
        else:
            x_grid = int((pose.position.x - self.map.limits[0]) / self.map.map_df.resolution[0])
            y_grid = int((pose.position.y - self.map.limits[2]) / self.map.map_df.resolution[0])
        return [x_grid, y_grid]

    def __grid_to_pose(self, grid_point):
        pose = PoseStamped()
        pose.pose.position.x = grid_point[0] * self.map.map_df.resolution[0] + self.map.limits[0]
        pose.pose.position.y = grid_point[1] * self.map.map_df.resolution[0] + self.map.limits[2]
        pose.pose.orientation.w = 1.0
        return pose

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.path.poses:
                vehicle_pose = self.current_pose

                if self.current_waypoint_index < len(self.path.poses):
                    target_pose = self.path.poses[self.current_waypoint_index]
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

    def compute_velocity(self, vehicle_pose, target_pose):
        dx = target_pose.pose.position.x - vehicle_pose.position.x
        dy = target_pose.pose.position.y - vehicle_pose.position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        heading = math.atan2(dy, dx)

        current_heading = self.__get_yaw_from_quaternion(vehicle_pose.orientation)
        heading_error = heading - current_heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        linear_speed = min(0.2, 0.2 * distance)
        angular_speed = max(-1.0, min(1.0, 2.0 * heading_error))

        return linear_speed, angular_speed

    def move_ttbot(self, linear_speed, angular_speed):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_speed
        cmd_vel.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd_vel)

    def __get_yaw_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    print("Task2 Bonus from task2 package is running!")

    rclpy.init(args=args)
    nav = Navigation()

    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
