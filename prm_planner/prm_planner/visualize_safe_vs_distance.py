#!/usr/bin/env python3
"""
Visualize Safe vs Distance path planning modes
Calls PRM planner service for both modes and plots them together
"""

import rclpy
from rclpy.node import Node
from prm_planner.srv import GetPRMPath
import matplotlib.pyplot as plt
import numpy as np

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # Create service client
        self.client = self.create_client(GetPRMPath, 'get_prm_path')
        
        # Wait for service
        self.get_logger().info('Waiting for PRM planner service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('PRM planner service not available!')
            self.get_logger().error('Make sure prm_planner_node is running: ros2 run prm_planner prm_planner_node')
            raise RuntimeError('Service not available')
        self.get_logger().info('PRM planner service found!')
    
    def request_path(self, start_x, start_y, goal_x, goal_y, planning_mode):
        """Request a path from PRM planner service"""
        request = GetPRMPath.Request()
        request.start_x = start_x
        request.start_y = start_y
        request.goal_x = goal_x
        request.goal_y = goal_y
        request.planning_mode = planning_mode
        
        self.get_logger().info(f'Requesting {planning_mode} path from ({start_x:.3f}, {start_y:.3f}) to ({goal_x:.3f}, {goal_y:.3f})')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{planning_mode.capitalize()} path: {len(response.path_x)} waypoints')
                return response.path_x, response.path_y
            else:
                self.get_logger().error(f'{planning_mode.capitalize()} path failed: {response.message}')
                return None, None
        else:
            self.get_logger().error(f'{planning_mode.capitalize()} service call failed')
            return None, None

def visualize_paths(start_x, start_y, goal_x, goal_y):
    """Visualize safe vs distance paths"""
    rclpy.init()
    
    try:
        visualizer = PathVisualizer()
        
        # Request both paths
        distance_path_x, distance_path_y = visualizer.request_path(start_x, start_y, goal_x, goal_y, 'distance')
        safe_path_x, safe_path_y = visualizer.request_path(start_x, start_y, goal_x, goal_y, 'safe')
        
        # Create visualization
        fig, ax = plt.subplots(figsize=(12, 10))
        
        # Workspace and obstacle bounds (from PRM planner)
        workspace_bounds = [0.0, 2.8067, 0.0, 2.6544]
        obstacle_bounds = [1.336675, 1.654175, 1.003300, 1.416000]
        
        ws_min_x, ws_max_x, ws_min_y, ws_max_y = workspace_bounds
        obs_min_x, obs_max_x, obs_min_y, obs_max_y = obstacle_bounds
        
        # Draw workspace
        ax.add_patch(plt.Rectangle((ws_min_x, ws_min_y), 
                                  ws_max_x - ws_min_x, ws_max_y - ws_min_y,
                                  fill=False, edgecolor='gray', linestyle='--', linewidth=1, label='Workspace'))
        
        # Draw obstacle box
        ax.add_patch(plt.Rectangle((obs_min_x, obs_min_y),
                                  obs_max_x - obs_min_x, obs_max_y - obs_min_y,
                                  fill=True, facecolor='red', alpha=0.3, edgecolor='red', linewidth=2, label='Obstacle'))
        
        # Plot distance path (shorter, closer to obstacle)
        if distance_path_x and len(distance_path_x) > 0:
            ax.plot(distance_path_x, distance_path_y, 'b-o', linewidth=2.5, markersize=5, 
                   label='Distance Path', zorder=3, alpha=0.8)
        
        # Plot safe path (longer, further from obstacle)
        if safe_path_x and len(safe_path_x) > 0:
            ax.plot(safe_path_x, safe_path_y, 'g-s', linewidth=2.5, markersize=5, 
                   label='Safe Path', zorder=3, alpha=0.8)
        
        # Mark start and goal
        ax.plot(start_x, start_y, 'ko', markersize=12, label='Start', zorder=5, markerfacecolor='yellow', markeredgewidth=2)
        ax.plot(goal_x, goal_y, 'k*', markersize=20, label='Goal', zorder=5, markerfacecolor='orange', markeredgewidth=2)
        
        # Calculate and display path lengths
        if distance_path_x and len(distance_path_x) > 1:
            dist_length = sum([np.sqrt((distance_path_x[i+1] - distance_path_x[i])**2 + 
                                      (distance_path_y[i+1] - distance_path_y[i])**2) 
                              for i in range(len(distance_path_x)-1)])
            ax.text(0.02, 0.98, f'Distance Path Length: {dist_length:.3f}m', 
                   transform=ax.transAxes, fontsize=10, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
        
        if safe_path_x and len(safe_path_x) > 1:
            safe_length = sum([np.sqrt((safe_path_x[i+1] - safe_path_x[i])**2 + 
                                      (safe_path_y[i+1] - safe_path_y[i])**2) 
                              for i in range(len(safe_path_x)-1)])
            ax.text(0.02, 0.92, f'Safe Path Length: {safe_length:.3f}m', 
                   transform=ax.transAxes, fontsize=10, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Safe vs Distance Path Planning Comparison', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.set_xlim(ws_min_x - 0.1, ws_max_x + 0.1)
        ax.set_ylim(ws_min_y - 0.1, ws_max_y + 0.1)
        
        plt.tight_layout()
        output_path = '/tmp/safe_vs_distance_paths.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'\nVisualization saved to: {output_path}')
        print('To view: eog /tmp/safe_vs_distance_paths.png or xdg-open /tmp/safe_vs_distance_paths.png\n')
        
        plt.close(fig)
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    
    # Default start/goal from hw4 launch file, or use command line args
    if len(sys.argv) == 5:
        start_x, start_y, goal_x, goal_y = map(float, sys.argv[1:5])
    else:
        # Default values from hw4 launch
        start_x, start_y = 1.9685, 0.7620
        goal_x, goal_y = 0.5588, 2.1256
        print(f'Using default start/goal: ({start_x:.3f}, {start_y:.3f}) -> ({goal_x:.3f}, {goal_y:.3f})')
        print('Or provide custom: python visualize_safe_vs_distance.py <start_x> <start_y> <goal_x> <goal_y>\n')
    
    visualize_paths(start_x, start_y, goal_x, goal_y)

