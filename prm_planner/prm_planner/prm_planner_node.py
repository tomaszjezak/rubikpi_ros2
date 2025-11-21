#!/usr/bin/env python3
"""
PRM Planner ROS2 Node
Main path planning node that integrates PRM algorithm with ROS2
"""

import rclpy
from rclpy.node import Node

# import prm core - no KDTree needed!
from prm_planner.prm_core import sample_points, generate_road_map, dijkstra_planning

# import service
from prm_planner.srv import GetPRMPath

class PRMPlannerNode(Node):
    def __init__(self):
        super().__init__('prm_planner_node')

        # Declare ROS2 parameters
        self.declare_parameter('n_sample', 1000)
        self.declare_parameter('n_knn', 10)
        self.declare_parameter('max_edge_len', 3.5)  # Max edge ~diagonal of arena
        self.declare_parameter('robot_radius', 0.11)  # 11cm robot radius
        self.declare_parameter('safety_margin', 0.4)  # 40cm safety margin for safe mode
        self.declare_parameter('workspace_bounds', [0.0, 2.8067, 0.0, 2.6544])  # Actual arena bounds
        self.declare_parameter('obstacle_bounds', [1.336675, 1.654175, 1.003300, 1.416000])  # Box at 52 5/8" X, 39.5" Y, 12.5"×16.25"

        # Get parameters
        self.n_sample = self.get_parameter('n_sample').get_parameter_value().integer_value
        self.n_knn = self.get_parameter('n_knn').get_parameter_value().integer_value
        self.max_edge_len = self.get_parameter('max_edge_len').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.safety_margin = self.get_parameter('safety_margin').get_parameter_value().double_value
        workspace_bounds = self.get_parameter('workspace_bounds').get_parameter_value().double_array_value
        obstacle_bounds = self.get_parameter('obstacle_bounds').get_parameter_value().double_array_value

        self.workspace_bounds = tuple(workspace_bounds)
        self.obstacle_bounds = tuple(obstacle_bounds)

        # Create service server
        self.service = self.create_service(
            GetPRMPath,
            'get_prm_path',
            self.plan_path_callback
        )

        self.get_logger().info('PRM Planner Node initialized')
        self.get_logger().info(f'Workspace: {self.workspace_bounds}, Obstacle: {self.obstacle_bounds}')
        self.get_logger().info('Service available at: get_prm_path')

    def plan_path_callback(self, request, response):
        """Service callback to plan a path from start to goal"""
        sx = request.start_x
        sy = request.start_y
        gx = request.goal_x
        gy = request.goal_y
        
        # Get planning mode (default to "distance" if not provided)
        planning_mode = request.planning_mode.lower() if request.planning_mode else "distance"
        
        # Calculate effective radius based on planning mode
        if planning_mode == "safe":
            effective_radius = self.robot_radius + self.safety_margin
        else:
            effective_radius = self.robot_radius
            if planning_mode != "distance":
                self.get_logger().warn(f'Unknown planning_mode "{request.planning_mode}", defaulting to "distance"')
                planning_mode = "distance"

        self.get_logger().info(
            f'Service request: Planning from ({sx:.2f}, {sy:.2f}) to ({gx:.2f}, {gy:.2f}) '
            f'[Mode: {planning_mode}, Effective radius: {effective_radius:.4f}m]'
        )

        try:
            # Call prm_core functions to plan path with effective radius
            rx, ry = self._plan_path(sx, sy, gx, gy, effective_radius)

            if len(rx) == 0:
                response.success = False
                response.message = "No path found"
                self.get_logger().warn('No path found')
                return response

            # Convert rx, ry to response format (already in goal->start order, reverse for start->goal)
            response.path_x = list(reversed(rx))
            response.path_y = list(reversed(ry))
            response.success = True
            response.message = f"Path found with {len(rx)} waypoints"

            self.get_logger().info(f'Path planned successfully: {len(rx)} waypoints')
            return response

        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Planning error: {str(e)}"
            return response

    def _plan_path(self, sx, sy, gx, gy, effective_radius):
        """
        Call prm_core functions to plan path.
        
        Args:
            sx: Start x coordinate
            sy: Start y coordinate
            gx: Goal x coordinate
            gy: Goal y coordinate
            effective_radius: Effective robot radius for collision checking
        
        Returns:
            Tuple (rx, ry) - path coordinates in goal->start order, or ([], []) if no path
        """
        # Call prm_core functions - obstacle_bounds is enough, no KDTree!
        sample_x, sample_y = sample_points(
            sx, sy, gx, gy, effective_radius,
            self.workspace_bounds, self.obstacle_bounds,
            rng=None, n_sample=self.n_sample
        )

        road_map = generate_road_map(
            sample_x, sample_y, effective_radius,
            self.obstacle_bounds, n_knn=self.n_knn, max_edge_len=self.max_edge_len
        )

        rx, ry = dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y)

        return rx, ry


def main(args=None):
    rclpy.init(args=args)
    node = PRMPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
