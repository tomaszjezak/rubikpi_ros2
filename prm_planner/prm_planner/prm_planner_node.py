#!/usr/bin/env python3
"""
PRM Planner ROS2 Node
Main path planning node that integrates PRM algorithm with ROS2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
from scipy.spatial import KDTree
import numpy as np

# import prm core
from .prm_core import (
        sample_points, generate_road_map, dijkstra_planning,
        N_SAMPLE, N_KNN, MAX_EDGE_LEN
    )

class PRMPlannerNode(Node):
    def __init__(self):
        super().__init__('prm_planner_node')

        # Declare ROS2 parameters
        self.declare_parameter('n_sample', 1000)
        self.declare_parameter('n_knn', 10)
        self.declare_parameter('max_edge_len', 14.1)
        self.declare_parameter('robot_radius', 0.5)
        self.declare_parameter('workspace_bounds', [0.0, 10.0, 0.0, 10.0])  # [min_x, max_x, min_y, max_y]
        self.declare_parameter('obstacle_bounds', [4.75, 5.25, 4.75, 5.25])  # [min_x, max_x, min_y, max_y]
        # these numbers are dum ^ because i should be thinking in meters, we can measure and decide tomorrow

        # Get parameters
        self.n_sample = self.get_parameter('n_sample').get_parameter_value().integer_value
        self.n_knn = self.get_parameter('n_knn').get_parameter_value().integer_value
        self.max_edge_len = self.get_parameter('max_edge_len').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        workspace_bounds = self.get_parameter('workspace_bounds').get_parameter_value().double_array_value
        obstacle_bounds = self.get_parameter('obstacle_bounds').get_parameter_value().double_array_value