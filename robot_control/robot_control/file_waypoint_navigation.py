#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import time
import os
from typing import List, Tuple


def normalize_angle(theta):
    """Wrap angle to [-π, π]"""
    return math.atan2(math.sin(theta), math.cos(theta))


def load_waypoints(filepath):
    """
    Load waypoints from text file.
    
    Format: x,y,yaw (one per line, yaw in radians)
    Returns: numpy array of shape (n, 3)
    """
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Waypoint file not found: {filepath}")
    
    waypoints = np.loadtxt(filepath, delimiter=',')
    
    # Ensure 2D array even for single waypoint
    if waypoints.ndim == 1:
        waypoints = waypoints.reshape(1, -1)
    
    return waypoints


class FileWaypointNavigator(Node):
    def __init__(self, waypoint_file='waypoints.txt'):
        super().__init__('file_waypoint_navigator')
        
        # Publisher for motor commands
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'motor_commands',
            10
        )
        
        # Motion parameters
        self.LINEAR_VELOCITY = 0.20  # m/s
        self.ANGULAR_VELOCITY = 0.14  # rad/s
        self.DT = 0.05  # 20 Hz (50 ms per tick)
        
        # Motor primitives
        self.FORWARD_CMD = [0.20, 0.155]
        self.TURN_LEFT_CMD = [-0.30, 0.30]
        self.TURN_RIGHT_CMD = [0.30, -0.30]
        self.STOP_CMD = [0.0, 0.0]
        
        # State
        self.current_yaw = 0.0
        self.waypoints = None
        self.trajectory = []  # List of (command, ticks) tuples
        
        self.get_logger().info('File-based Waypoint Navigator started')
        self.get_logger().info(f'Linear velocity: {self.LINEAR_VELOCITY} m/s')
        self.get_logger().info(f'Angular velocity: {self.ANGULAR_VELOCITY} rad/s')
        self.get_logger().info(f'Control rate: {1/self.DT} Hz')
        
        # Load waypoints
        try:
            self.waypoint_file = waypoint_file
            self.waypoints = load_waypoints(waypoint_file)
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {waypoint_file}')
            
            # Generate trajectory
            self.trajectory = self.generate_trajectory_from_waypoints()
            self.get_logger().info(f'Generated trajectory with {len(self.trajectory)} segments')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            raise
    
    def generate_trajectory_from_waypoints(self) -> List[Tuple[List[float], int]]:
        """
        Generate trajectory from waypoints.
        
        Returns:
            List of (motor_command, num_ticks) tuples
        """
        if self.waypoints is None or len(self.waypoints) == 0:
            return []
        
        trajectory = []
        current_yaw = 0.0  # Start facing 0 radians (along +X axis)
        
        for i in range(len(self.waypoints) - 1):
            # Current and next waypoint
            x1, y1, yaw1 = self.waypoints[i]
            x2, y2, yaw2 = self.waypoints[i + 1]
            
            # Compute delta position
            dx = x2 - x1
            dy = y2 - y1
            distance = math.sqrt(dx**2 + dy**2)
            
            # Compute target heading to next waypoint
            target_heading = math.atan2(dy, dx)
            
            # Compute yaw change needed
            delta_yaw = normalize_angle(target_heading - current_yaw)
            
            # --- Turn phase ---
            if abs(delta_yaw) > 0.01:  # Threshold: ~0.5 degrees
                turn_time = abs(delta_yaw) / self.ANGULAR_VELOCITY
                n_turn_ticks = int(turn_time / self.DT)
                
                if n_turn_ticks > 0:
                    # Choose turn direction
                    if delta_yaw > 0:
                        turn_cmd = self.TURN_LEFT_CMD  # CCW
                    else:
                        turn_cmd = self.TURN_RIGHT_CMD  # CW
                    
                    trajectory.append((turn_cmd, n_turn_ticks))
                    self.get_logger().info(
                        f'Segment {i}: Turn {math.degrees(delta_yaw):.1f}° '
                        f'for {n_turn_ticks} ticks ({turn_time:.2f}s)'
                    )
            
            # Update current yaw after turn
            current_yaw = target_heading
            
            # --- Forward phase ---
            if distance > 0.01:  # Threshold: 1 cm
                forward_time = distance / self.LINEAR_VELOCITY
                n_forward_ticks = int(forward_time / self.DT)
                
                if n_forward_ticks > 0:
                    trajectory.append((self.FORWARD_CMD, n_forward_ticks))
                    self.get_logger().info(
                        f'Segment {i}: Forward {distance:.2f}m '
                        f'for {n_forward_ticks} ticks ({forward_time:.2f}s)'
                    )
        
        # Final rotation to match last waypoint orientation (optional)
        final_yaw = self.waypoints[-1, 2]
        final_delta_yaw = normalize_angle(final_yaw - current_yaw)
        
        if abs(final_delta_yaw) > 0.01:
            turn_time = abs(final_delta_yaw) / self.ANGULAR_VELOCITY
            n_turn_ticks = int(turn_time / self.DT)
            
            if n_turn_ticks > 0:
                if final_delta_yaw > 0:
                    turn_cmd = self.TURN_LEFT_CMD
                else:
                    turn_cmd = self.TURN_RIGHT_CMD
                
                trajectory.append((turn_cmd, n_turn_ticks))
                self.get_logger().info(
                    f'Final rotation: {math.degrees(final_delta_yaw):.1f}° '
                    f'for {n_turn_ticks} ticks'
                )
        
        # Add stop command at end
        trajectory.append((self.STOP_CMD, 1))
        
        return trajectory
    
    def execute_trajectory(self):
        """Execute the pre-computed trajectory"""
        if not self.trajectory:
            self.get_logger().warn('No trajectory to execute')
            return
        
        self.get_logger().info('Starting trajectory execution...')
        
        for segment_idx, (motor_cmd, num_ticks) in enumerate(self.trajectory):
            self.get_logger().info(
                f'Executing segment {segment_idx + 1}/{len(self.trajectory)}: '
                f'cmd={motor_cmd}, ticks={num_ticks}'
            )
            
            for tick in range(num_ticks):
                # Publish motor command
                msg = Float32MultiArray()
                msg.data = motor_cmd
                self.publisher.publish(msg)
                
                # Wait for next tick (20 Hz)
                time.sleep(self.DT)
        
        # Final stop
        stop_msg = Float32MultiArray()
        stop_msg.data = self.STOP_CMD
        self.publisher.publish(stop_msg)
        
        self.get_logger().info('✅ Trajectory execution complete!')
    
    def run(self):
        """Main execution function"""
        # Give ROS some time to initialize
        time.sleep(0.5)
        
        # Execute trajectory
        self.execute_trajectory()


def main(args=None):
    import argparse
    
    parser = argparse.ArgumentParser(description='File-based Waypoint Navigator')
    parser.add_argument(
        '--file',
        type=str,
        default='waypoints.txt',
        help='Path to waypoint file (default: waypoints.txt)'
    )
    
    cli_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    try:
        node = FileWaypointNavigator(waypoint_file=cli_args.file)
        
        # Run trajectory
        node.run()
        
        # Keep node alive briefly for final messages
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        print('\n⚠️  Interrupted by user')
    except Exception as e:
        print(f'❌ Error: {str(e)}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

