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
from copy import copy, deepcopy

from PIL import Image, ImageOps
#import matplotlib.pyplot as plt
#import matplotlib.cm as cm
import yaml
from nav_msgs.msg import OccupancyGrid
from PIL import Image, ImageOps
import pandas as pd     #not rep installed - install it here
from geometry_msgs.msg import Quaternion


class Map():
    def __init__(self):
        self.map_im, self.map_df, self.limits = self.__open_map()
        self.image_array = self.__get_obstacle_map(self.map_im, self.map_df)

    def __open_map(self):
        #yaml_path = '/home/me597/lab4/src/task_7/maps/sync_classroom_map'
        #yaml_path = '/home/me597/ros2_ws/src/task_7/maps/classroom_map'
        yaml_path = '/home/me597/final_project/sim_ws/src/turtlebot3_gazebo/maps/map'     #hard-coding it, taking no chances
        f = open(yaml_path +'.yaml', 'r')
        map_df = pd.json_normalize(yaml.safe_load(f))
       
        #map_image_path = map_df.image[0]  # Path to the image
        map_image_path = map_df.image[0]
        full_image_path = os.path.join(os.path.dirname(yaml_path + '.yaml'), map_image_path)
        im = Image.open(full_image_path)
        #im = Image.open('/home/me597/sim_ws/src/turtlebot3_gazebo/maps/map.pgm')   #tried hard-coding. guess I'll be taking chances
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]
        return im, map_df, [xmin,xmax,ymin,ymax]

    def __get_obstacle_map(self,map_im, map_df):
        img_array = np.reshape(list(self.map_im.getdata()),(self.map_im.size[1],self.map_im.size[0]))
        up_thresh = self.map_df.occupied_thresh[0]*255*2
        low_thresh = self.map_df.free_thresh[0]*255*2

        for j in range(self.map_im.size[0]):
            for i in range(self.map_im.size[1]):
                if img_array[i,j] > up_thresh:
                    img_array[i,j] = 255*2
                else:
                    img_array[i,j] = 0
        return img_array

class Queue():
    def __init__(self, init_queue = []):
        self.queue = copy(init_queue)
        self.start = 0
        self.end = len(self.queue)-1

    def __len__(self):
        numel = len(self.queue)
        return numel

    def __repr__(self):
        q = self.queue
        tmpstr = ""
        for i in range(len(self.queue)):
            flag = False
            if(i == self.start):
                tmpstr += "<"
                flag = True
            if(i == self.end):
                tmpstr += ">"
                flag = True

            if(flag):
                tmpstr += '| ' + str(q[i]) + '|\n'
            else:
                tmpstr += ' | ' + str(q[i]) + '|\n'

        return tmpstr

    def __call__(self):
        return self.queue

    def initialize_queue(self,init_queue = []):
        self.queue = copy(init_queue)

    def sort(self,key=str.lower):
        self.queue = sorted(self.queue,key=key)

    def push(self,data):
        self.queue.append(data)
        self.end += 1

    def pop(self):
        p = self.queue.pop(self.start)
        self.end = len(self.queue)-1
        return p

class Node():
    def __init__(self,name):
        self.name = name
        self.children = []
        self.weight = []

    def __repr__(self):
        return self.name

    def add_children(self,node,w=None):
        if w == None:
            w = [1]*len(node)
        self.children.extend(node)
        self.weight.extend(w)

class Tree():
    def __init__(self, name):
        self.name = name
        self.name = '0,0'
        self.end = '0,0'
        #below for holding nodes in a dict
        self.g = {} 

    def add_node(self, node, start=False, end=False):
        """Add a node to the tree."""
        self.g[node.name] = node
        if start:
            self.root = node.name
        elif end:
            self.end = node.name

    def set_as_root(self, node):
        """Set the specified node as the root."""
        self.root = node.name
        #resetting end
        self.end = None

    def set_as_end(self, node):
        """Set the specified node as the end."""
        self.end = node.name
        #resetting root
        self.root = None 

    def get_children(self, node_name):
        """Return the children of the specified node."""
        if node_name in self.g:
            return self.g[node_name].children
        return None

    def get_weight(self, node_name):
        """Return the weights of the edges from the specified node."""
        if node_name in self.g:
            return self.g[node_name].weight
        return None

    def __str__(self):
        """String representation of the tree."""
        return f"Tree(name={self.name}, root={self.root}, end={self.end}, nodes={list(self.g.keys())})"

import heapq
import numpy as np  

class AStar:
    #same old a* from google colab
    def __init__(self, in_tree):
        self.in_tree = in_tree
        self.q = []
        self.dist = {name: np.inf for name in in_tree.g}
        self.h = {}
        self.via = {name: None for name in in_tree.g}

        end = tuple(map(int, self.in_tree.end.split(',')))
        for name in in_tree.g:
            start = tuple(map(int, name.split(',')))
            self.h[name] = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)

    def __get_f_score(self, name):
        return self.dist[name] + self.h[name]

    def solve(self, sn, en):
        self.dist[sn.name] = 0
        heapq.heappush(self.q, (self.__get_f_score(sn.name), sn.name))
       
        while self.q:
            _, current_name = heapq.heappop(self.q)
            current_node = self.in_tree.g[current_name]
           
            if current_name == en.name:
                break
           
            for child, weight in zip(current_node.children, current_node.weight):
                new_dist = self.dist[current_name] + weight
                if new_dist < self.dist[child.name]:
                    self.dist[child.name] = new_dist
                    self.via[child.name] = current_name
                    heapq.heappush(self.q, (self.__get_f_score(child.name), child.name))

    def reconstruct_path(self, sn, en):
        path = []
        current = en.name
        while current != sn.name:
            path.append(current)
            current = self.via[current]
        path.append(sn.name)
        path.reverse()
        return path, self.dist[en.name]

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
        """Reset the accumulated error and last error."""
        self.error_sum = 0
        self.last_error = 0


class MapProcessor():
    def __init__(self):
        # initialize map without passing arguments
        self.map = Map()
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)  # Initialize based on the image_array shape
        self.map_graph = Tree('map')

    def __modify_map_pixel(self,map_array,i,j,value,absolute):
        if( (i >= 0) and
            (i < map_array.shape[0]) and
            (j >= 0) and
            (j < map_array.shape[1]) ):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value

    def __inflate_obstacle(self,kernel,map_array,i,j,absolute):
        dx = int(kernel.shape[0]//2)
        dy = int(kernel.shape[1]//2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array,i,j,kernel[0][0],absolute)
        else:
            for k in range(i-dx,i+dx):
                for l in range(j-dy,j+dy):
                    self.__modify_map_pixel(map_array,k,l,kernel[k-i+dx][l-j+dy],absolute)

    def inflate_map(self,kernel,absolute=True):
        # Perform an operation like dilation, such that the small wall found during the mapping process
        # are increased in size, thus forcing a safer path.
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.map.image_array[i][j] == 0:
                    self.__inflate_obstacle(kernel,self.inf_map_img_array,i,j,absolute)
        r = np.max(self.inf_map_img_array)-np.min(self.inf_map_img_array)
        if r == 0:
            r = 1
        self.inf_map_img_array = (self.inf_map_img_array - np.min(self.inf_map_img_array))/r

    def get_graph_from_map(self):
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    node = Node('%d,%d'%(i,j))
                    self.map_graph.add_node(node)
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.inf_map_img_array[i][j] == 0:
                    if (i > 0):
                        if self.inf_map_img_array[i-1][j] == 0:
                            # add an edge up
                            child_up = self.map_graph.g['%d,%d'%(i-1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up],[1])
                    if (i < (self.map.image_array.shape[0] - 1)):
                        if self.inf_map_img_array[i+1][j] == 0:
                            # add an edge down
                            child_dw = self.map_graph.g['%d,%d'%(i+1,j)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw],[1])
                    if (j > 0):
                        if self.inf_map_img_array[i][j-1] == 0:
                            # add an edge to the left
                            child_lf = self.map_graph.g['%d,%d'%(i,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_lf],[1])
                    if (j < (self.map.image_array.shape[1] - 1)):
                        if self.inf_map_img_array[i][j+1] == 0:
                            # add an edge to the right
                            child_rg = self.map_graph.g['%d,%d'%(i,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_rg],[1])
                    if ((i > 0) and (j > 0)):
                        if self.inf_map_img_array[i-1][j-1] == 0:
                            # add an edge up-left
                            child_up_lf = self.map_graph.g['%d,%d'%(i-1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_lf],[np.sqrt(2)])
                    if ((i > 0) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i-1][j+1] == 0:
                            # add an edge up-right
                            child_up_rg = self.map_graph.g['%d,%d'%(i-1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_up_rg],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j > 0)):
                        if self.inf_map_img_array[i+1][j-1] == 0:
                            # add an edge down-left
                            child_dw_lf = self.map_graph.g['%d,%d'%(i+1,j-1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_lf],[np.sqrt(2)])
                    if ((i < (self.map.image_array.shape[0] - 1)) and (j < (self.map.image_array.shape[1] - 1))):
                        if self.inf_map_img_array[i+1][j+1] == 0:
                            # add an edge down-right
                            child_dw_rg = self.map_graph.g['%d,%d'%(i+1,j+1)]
                            self.map_graph.g['%d,%d'%(i,j)].add_children([child_dw_rg],[np.sqrt(2)])

    def gaussian_kernel(self, size, sigma=1):
        size = int(size) // 2
        x, y = np.mgrid[-size:size+1, -size:size+1]
        normal = 1 / (2.0 * np.pi * sigma**2)
        g =  np.exp(-((x**2 + y**2) / (2.0*sigma**2))) * normal
        r = np.max(g)-np.min(g)
        sm = (g - np.min(g))*1/r
        return sm

    def rect_kernel(self, size, value):
        m = np.ones(shape=(size,size))
        return m

    def draw_path(self,path):
        path_tuple_list = []
        path_array = copy(self.inf_map_img_array)
        for idx in path:
            tup = tuple(map(int, idx.split(',')))
            path_tuple_list.append(tup)
            path_array[tup] = 0.5
        return path_array





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
        self.map_processor = MapProcessor()
        self.map_processor.inflate_map(self.map_processor.rect_kernel(5, 1), absolute=True)
        self.map_processor.get_graph_from_map()
        self.astar = AStar(self.map_processor.map_graph)

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.__ttbot_pose_cbk, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
       
        # Pure Pursuit parameters
        self.lookahead_distance = 0.5
        self.max_linear_speed = 0.2
        self.max_angular_speed = 1.0
        self.last_update_time = self.get_clock().now()



        self.obstacle_tolerance = 0.5  # Tolerance distance for obstacle detection



    def detect_obstacle(self, current_pose):
        # Placeholder for actual obstacle detection logic
        # For simplicity, assume we know obstacle locations
        obstacles = [
            (2.0, 2.0),  # Example obstacle coordinates (x, y)
            (4.0, 3.0)
        ]
        
        for obs in obstacles:
            distance = math.sqrt(
                (current_pose.pose.position.x - obs[0])**2 +
                (current_pose.pose.position.y - obs[1])**2
            )
            if distance < self.obstacle_tolerance:
                return obs  # Return the obstacle position if detected
        return None

    def generate_bypass_path(self, current_pose, obstacle_position, original_path):
        # Generate a simple local bypass path
        bypass_path = []

        # Create intermediate points to bypass the obstacle
        direction_vector = np.array([
            current_pose.pose.position.x - obstacle_position[0],
            current_pose.pose.position.y - obstacle_position[1]
        ])
        direction_vector = direction_vector / np.linalg.norm(direction_vector)  # Normalize

        # Add points around the obstacle
        offset_distance = self.obstacle_tolerance + 0.5
        bypass_point_1 = (
            obstacle_position[0] + direction_vector[1] * offset_distance,
            obstacle_position[1] - direction_vector[0] * offset_distance
        )
        bypass_point_2 = (
            obstacle_position[0] - direction_vector[1] * offset_distance,
            obstacle_position[1] + direction_vector[0] * offset_distance
        )

        for point in [bypass_point_1, bypass_point_2]:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            bypass_path.append(pose)

        return bypass_path + original_path.poses  # Return bypass path merged with original   




        

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
            path = self.a_star_path_planner(current_pose, self.goal_pose)
            if path:
                self.path = path
                self.current_waypoint_index = 0
                self.get_logger().info("Path planned successfully")

    def a_star_path_planner(self, current_pose, goal_pose):
        self.get_logger().info("Converting poses to grid coordinates...")
        start_node_name = self.__pose_to_grid(current_pose)
        end_node_name = self.__pose_to_grid(goal_pose)

        self.get_logger().info(f"Start node: {start_node_name}, End node: {end_node_name}")

        if start_node_name not in self.map_processor.map_graph.g or end_node_name not in self.map_processor.map_graph.g:
            self.get_logger().warn("Start or end node not found in the graph.")
            return None

        self.get_logger().info("Running A* algorithm...")
        start_node = self.map_processor.map_graph.g[start_node_name]
        end_node = self.map_processor.map_graph.g[end_node_name]

        try:
            self.astar.solve(start_node, end_node)
        except Exception as e:
            self.get_logger().error(f"Error during A* execution: {str(e)}")
            return None

        self.get_logger().info("Reconstructing path...")
        try:
            path_nodes, _ = self.astar.reconstruct_path(start_node, end_node)
        except Exception as e:
            self.get_logger().error(f"Error during path reconstruction: {str(e)}")
            return None

        path = Path()
        for node in path_nodes:
            pose = self.__grid_to_pose(node)
            path.poses.append(pose)

        self.get_logger().info(f'Original waypoints: {len(path.poses)}')

        smoothed_path = self.smooth_and_segment_waypoints(path.poses)
        path.poses = smoothed_path

        self.get_logger().info(f'Smoothed waypoints: {len(path.poses)}')

        return path

    def smooth_and_segment_waypoints(self, waypoints, threshold=0.25):
        smoothed_waypoints = []
        temp_waypoints = []
        accumulated_distance = 0.0

        if not waypoints:
            self.get_logger().warn("No waypoints provided for smoothing.")
            return smoothed_waypoints  # Return empty list if no waypoints

        for i in range(len(waypoints) - 1):
            current_wp = waypoints[i].pose.position
            next_wp = waypoints[i + 1].pose.position

            distance = math.sqrt((next_wp.x - current_wp.x) ** 2 + (next_wp.y - current_wp.y) ** 2)

            if accumulated_distance + distance < threshold:
                temp_waypoints.append(waypoints[i])
                accumulated_distance += distance
            else:
                if temp_waypoints:
                    smoothed_waypoint = self.average_waypoints(temp_waypoints)
                    smoothed_waypoints.append(smoothed_waypoint)
                temp_waypoints = [waypoints[i]]  # Start new segment
                accumulated_distance = distance  # Reset to current distance

        # Handle any remaining waypoints
        if temp_waypoints:
            smoothed_waypoint = self.average_waypoints(temp_waypoints)
            smoothed_waypoints.append(smoothed_waypoint)

        return smoothed_waypoints

    def average_waypoints(self, waypoints):
        num_waypoints = len(waypoints)
       
        if num_waypoints == 0:
            raise ValueError("No waypoints to average.")
        elif num_waypoints == 1:
            # If there's only one waypoint, return it as the smoothed waypoint
            return waypoints[0]

        smoothed_waypoint = PoseStamped()
       
        x = sum(wp.pose.position.x for wp in waypoints) / num_waypoints
        y = sum(wp.pose.position.y for wp in waypoints) / num_waypoints
       
        smoothed_waypoint.pose.position.x = x
        smoothed_waypoint.pose.position.y = y
        smoothed_waypoint.pose.orientation.w = 1.0  # Default orientation

        return smoothed_waypoint

    def get_yaw_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def __pose_to_grid(self, pose):
        if hasattr(pose, 'pose'):
            x_grid = int((pose.pose.position.x - self.map_processor.map.limits[0]) / self.map_processor.map.map_df.resolution[0])
            y_grid = int((pose.pose.position.y - self.map_processor.map.limits[2]) / self.map_processor.map.map_df.resolution[0])
        else:
            x_grid = int((pose.position.x - self.map_processor.map.limits[0]) / self.map_processor.map.map_df.resolution[0])
            y_grid = int((pose.position.y - self.map_processor.map.limits[2]) / self.map_processor.map.map_df.resolution[0])
        return f"{x_grid},{y_grid}"

    def __grid_to_pose(self, grid_node_name):
        grid_x, grid_y = map(int, grid_node_name.split(','))
        pose = PoseStamped()
        pose.pose.position.x = grid_x * self.map_processor.map.map_df.resolution[0] + self.map_processor.map.limits[0]
        pose.pose.position.y = grid_y * self.map_processor.map.map_df.resolution[0] + self.map_processor.map.limits[2]
        pose.pose.orientation.w = 1.0
        return pose

    def calculate_segment_heading_and_distance(self, vehicle_pose, target_pose):
        dx = target_pose.pose.position.x - vehicle_pose.position.x
        dy = target_pose.pose.position.y - vehicle_pose.position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        heading = math.atan2(dy, dx)
        return heading, distance


    def calculate_segment_heading_and_distance(self, vehicle_pose, target_pose):
        try:
            dx = target_pose.pose.position.x - vehicle_pose.position.x
            dy = target_pose.pose.position.y - vehicle_pose.position.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            heading = math.atan2(dy, dx)
            return heading, distance
        except Exception as e:
            self.get_logger().error(f"Error calculating segment: {str(e)}")
            return 0.0, 0.0

    def adjust_heading(self, target_heading, current_heading):
        # Calculate the angular difference
        angular_difference = target_heading - current_heading
        angular_difference = (angular_difference + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

        # Use PID controller to calculate angular speed
       

        angular_threshold = 0.13
        linear_speed = 0.1

        if abs(angular_difference) > angular_threshold:
            angular_speed = self.angular_pid.update(angular_difference)
            angular_speed = max(min(angular_speed, 0.5), -0.5)  # Limit max angular speed
            return 0.0, angular_speed  # Turn based on PID output
        else:
            self.angular_pid.reset()
            return linear_speed, 0.0  # Move forward without turning

    def find_target_point(self, vehicle_pose):
        if not self.path.poses:
            return None

        # Start looking from current waypoint index
        for i in range(self.current_waypoint_index, len(self.path.poses)):
            target = self.path.poses[i]
            dx = target.pose.position.x - vehicle_pose.position.x
            dy = target.pose.position.y - vehicle_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
           
            if distance >= self.lookahead_distance:
                return target, i

        # If no point found, return the last point
        return self.path.poses[-1], len(self.path.poses) - 1

    def compute_velocity(self, vehicle_pose, target_pose):
        dx = target_pose.pose.position.x - vehicle_pose.position.x
        dy = target_pose.pose.position.y - vehicle_pose.position.y
       
        # Calculate distance and heading to target
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        current_heading = self.get_yaw_from_quaternion(vehicle_pose.orientation)
       
        # Calculate heading error
        heading_error = target_heading - current_heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
       
        # Calculate velocities
        linear_velocity = self.max_linear_speed * math.cos(heading_error)
        angular_velocity = self.max_angular_speed * heading_error
       
        # Adjust speeds based on distance and heading error
        if distance < 0.3:
            linear_velocity *= (distance / 0.3)
        if abs(heading_error) > math.pi/4:
            linear_velocity *= 0.5
           
        # Clamp velocities
        linear_velocity = max(min(linear_velocity, self.max_linear_speed), -self.max_linear_speed)
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)
       
        return linear_velocity, angular_velocity

    def move_ttbot(self, speed, angular_speed):
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.path.poses:
                vehicle_pose = self.current_pose
               
                if self.current_waypoint_index < len(self.path.poses):
                    # Find target point along path
                    target_pose, new_index = self.find_target_point(vehicle_pose)






                    # Detect obstacle
                    obstacle = self.detect_obstacle(vehicle_pose)
                    if obstacle:
                        self.get_logger().info(f"Obstacle detected at {obstacle}, generating bypass path.")
                        bypass_path = self.generate_bypass_path(vehicle_pose, obstacle, self.path)
                        self.path.poses = bypass_path
                        self.current_waypoint_index = 0

                    # Follow the updated path
                    target_pose, new_index = self.find_target_point(vehicle_pose)


                







                    if target_pose is None:
                        continue
                       
                    self.current_waypoint_index = new_index
                   
                    # Compute and apply velocities
                    linear_speed, angular_speed = self.compute_velocity(vehicle_pose, target_pose)
                    self.move_ttbot(linear_speed, angular_speed)
                   
                    # Check if reached final goal
                    if self.current_waypoint_index == len(self.path.poses) - 1:
                        dx = target_pose.pose.position.x - vehicle_pose.position.x
                        dy = target_pose.pose.position.y - vehicle_pose.position.y
                        distance = math.sqrt(dx**2 + dy**2)
                        if distance < 0.1:
                            self.current_waypoint_index += 1
                            print('Way point: ',self.current_waypoint_index,'reached')
               
                elif self.current_waypoint_index == len(self.path.poses):
                    # Final orientation adjustment
                    if self.goal_pose is not None:
                        target_heading = self.get_yaw_from_quaternion(self.goal_pose.pose.orientation)
                        current_heading = self.get_yaw_from_quaternion(vehicle_pose.orientation)
                        heading_error = target_heading - current_heading
                        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
                       
                        if abs(heading_error) > 0.1:
                            self.move_ttbot(0.0, self.max_angular_speed * heading_error)
                        else:
                            self.current_waypoint_index += 1
                            self.move_ttbot(0.0, 0.0)
                    else:
                        self.get_logger().warn("Goal pose is not set.")
               
                else:
                    self.get_logger().info("Path complete")
                    self.move_ttbot(0.0, 0.0)
                    break


def main(args=None):

    print("Task2 from task2 package is running!")

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

