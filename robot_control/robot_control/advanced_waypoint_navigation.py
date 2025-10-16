#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose2D
import math
import argparse
import sys
import threading
import time
import numpy as np
from typing import List, Tuple


class TrajectoryGenerator:
    """Generate smooth trajectories with interpolation between waypoints"""
    
    def __init__(self, max_step_distance=0.05, max_step_angle=0.1):
        """
        Initialize trajectory generator
        
        Args:
            max_step_distance: Maximum distance for interpolation steps (meters)
            max_step_angle: Maximum angle for interpolation steps (radians)
        """
        self.max_step_distance = max_step_distance
        self.max_step_angle = max_step_angle
        
    def generate_trajectory(self, start_pose: Pose2D, end_pose: Pose2D) -> List[Tuple[str, float]]:
        """
        Generate interpolated trajectory from start to end pose
        
        Returns:
            List of (action_type, value) tuples
        """
        trajectory = []
        
        # Calculate total distance and angle
        dx = end_pose.x - start_pose.x
        dy = end_pose.y - start_pose.y
        total_distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angle difference
        angle_diff = end_pose.theta - start_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize
        
        # If distance is very small, just rotate
        if total_distance < 0.01:
            if abs(angle_diff) > 0.01:
                trajectory.append(('rotate', angle_diff))
            return trajectory
            
        # Calculate required heading
        required_heading = math.atan2(dy, dx)
        
        # Calculate rotation needed to face target
        heading_diff = required_heading - start_pose.theta
        heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))
        
        # Add initial rotation
        if abs(heading_diff) > 0.01:
            trajectory.append(('rotate', heading_diff))
            
        # Generate interpolated forward motion
        if total_distance > self.max_step_distance:
            num_steps = int(total_distance / self.max_step_distance) + 1
            step_distance = total_distance / num_steps
            
            for _ in range(num_steps):
                trajectory.append(('forward', step_distance))
        else:
            trajectory.append(('forward', total_distance))
            
        # Add final rotation
        final_rotation = end_pose.theta - required_heading
        final_rotation = math.atan2(math.sin(final_rotation), math.cos(final_rotation))
        
        if abs(final_rotation) > 0.01:
            trajectory.append(('rotate', final_rotation))
            
        return trajectory
        
    def generate_multi_waypoint_trajectory(self, waypoints: List[Tuple[float, float, float]]) -> List[Tuple[str, float]]:
        """
        Generate trajectory through multiple waypoints
        
        Args:
            waypoints: List of (x, y, theta) tuples
            
        Returns:
            Complete trajectory as list of (action_type, value) tuples
        """
        if not waypoints:
            return []
            
        trajectory = []
        current_pose = Pose2D()
        
        for x, y, theta in waypoints:
            target_pose = Pose2D()
            target_pose.x = x
            target_pose.y = y
            target_pose.theta = math.radians(theta) if abs(theta) > 2*math.pi else theta
            
            segment = self.generate_trajectory(current_pose, target_pose)
            trajectory.extend(segment)
            
            # Update current pose
            for action, value in segment:
                if action == 'forward':
                    current_pose.x += value * math.cos(current_pose.theta)
                    current_pose.y += value * math.sin(current_pose.theta)
                elif action == 'rotate':
                    current_pose.theta += value
                    current_pose.theta = math.atan2(math.sin(current_pose.theta), math.cos(current_pose.theta))
                    
        return trajectory


class AdvancedWaypointNavigationNode(Node):
    def __init__(self):
        super().__init__('advanced_waypoint_navigation_node')
        
        # Create publisher for motor commands
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'motor_commands',
            10
        )
        
        # Robot state
        self.current_pose = Pose2D()  # x, y, theta (in meters and radians)
        self.is_navigating = False
        self.running = True
        
        # Motion parameters (calibrated for your robot)
        self.FORWARD_DISTANCE_PER_COMMAND = 0.034  # 3.4 cm per forward command
        self.ROTATION_ANGLE_PER_COMMAND = math.radians(21)  # 21 degrees per rotation command
        self.FORWARD_SPEED = 0.3  # Motor speed for forward motion
        self.ROTATION_SPEED = 0.5  # Motor speed for rotation
        
        # Command timing
        self.COMMAND_DURATION = 0.1  # Duration to hold each command (seconds)
        
        # Initialize trajectory generator
        self.trajectory_generator = TrajectoryGenerator(
            max_step_distance=0.05,  # 5cm interpolation steps
            max_step_angle=math.radians(10)  # 10 degree interpolation steps
        )
        
        self.get_logger().info('Advanced Waypoint Navigation Node started')
        self.get_logger().info(f'Motion parameters:')
        self.get_logger().info(f'  Forward: {self.FORWARD_DISTANCE_PER_COMMAND*100:.1f} cm per command')
        self.get_logger().info(f'  Rotation: {math.degrees(self.ROTATION_ANGLE_PER_COMMAND):.1f} degrees per command')
        self.get_logger().info(f'  Interpolation: {self.trajectory_generator.max_step_distance*100:.1f} cm steps')
        
    def send_motor_command(self, left_speed, right_speed, duration=None):
        """Send motor command for specified duration"""
        if duration is None:
            duration = self.COMMAND_DURATION
            
        msg = Float32MultiArray()
        msg.data = [left_speed, right_speed]
        
        self.publisher.publish(msg)
        self.get_logger().debug(f'Sending motor command: L={left_speed:.2f}, R={right_speed:.2f}')
        
        if duration > 0:
            time.sleep(duration)
            
        # Send stop command
        stop_msg = Float32MultiArray()
        stop_msg.data = [0.0, 0.0]
        self.publisher.publish(stop_msg)
        
    def move_forward(self, distance):
        """Move forward by specified distance (in meters)"""
        if distance <= 0:
            return
            
        steps = int(distance / self.FORWARD_DISTANCE_PER_COMMAND)
        self.get_logger().info(f'Moving forward {distance*100:.1f} cm in {steps} steps')
        
        for _ in range(steps):
            if not self.running:
                break
            self.send_motor_command(self.FORWARD_SPEED, self.FORWARD_SPEED)
            
        # Update current pose
        self.current_pose.x += distance * math.cos(self.current_pose.theta)
        self.current_pose.y += distance * math.sin(self.current_pose.theta)
        self.get_logger().info(f'Current pose: ({self.current_pose.x:.3f}, {self.current_pose.y:.3f}, {math.degrees(self.current_pose.theta):.1f}°)')
        
    def rotate(self, angle):
        """Rotate by specified angle (in radians, positive = counterclockwise)"""
        if abs(angle) < 0.01:  # Skip very small rotations
            return
            
        steps = int(abs(angle) / self.ROTATION_ANGLE_PER_COMMAND)
        direction = 1 if angle > 0 else -1
        
        self.get_logger().info(f'Rotating {math.degrees(angle):.1f}° in {steps} steps')
        
        for _ in range(steps):
            if not self.running:
                break
            if direction > 0:  # Counterclockwise
                self.send_motor_command(-self.ROTATION_SPEED, self.ROTATION_SPEED)
            else:  # Clockwise
                self.send_motor_command(self.ROTATION_SPEED, -self.ROTATION_SPEED)
                
        # Update current pose
        self.current_pose.theta += angle
        # Normalize angle to [-pi, pi]
        self.current_pose.theta = math.atan2(math.sin(self.current_pose.theta), math.cos(self.current_pose.theta))
        self.get_logger().info(f'Current pose: ({self.current_pose.x:.3f}, {self.current_pose.y:.3f}, {math.degrees(self.current_pose.theta):.1f}°)')
        
    def execute_trajectory(self, trajectory):
        """Execute generated trajectory"""
        self.is_navigating = True
        self.get_logger().info(f'Executing trajectory with {len(trajectory)} steps')
        
        try:
            for i, (step_type, value) in enumerate(trajectory):
                if not self.running:
                    break
                    
                self.get_logger().info(f'Step {i+1}/{len(trajectory)}: {step_type} {value:.3f}')
                
                if step_type == 'rotate':
                    self.rotate(value)
                elif step_type == 'forward':
                    self.move_forward(value)
                    
            self.get_logger().info('Trajectory execution completed!')
            self.get_logger().info(f'Final pose: ({self.current_pose.x:.3f}, {self.current_pose.y:.3f}, {math.degrees(self.current_pose.theta):.1f}°)')
            
        except Exception as e:
            self.get_logger().error(f'Error during trajectory execution: {str(e)}')
        finally:
            self.is_navigating = False
            
    def navigate_to_waypoint(self, x, y, theta):
        """Navigate to single waypoint using trajectory generation"""
        if self.is_navigating:
            self.get_logger().warn('Already navigating!')
            return
            
        self.get_logger().info(f'Generating trajectory to ({x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°)')
        
        # Create target pose
        target_pose = Pose2D()
        target_pose.x = x
        target_pose.y = y
        target_pose.theta = theta
        
        # Generate trajectory
        trajectory = self.trajectory_generator.generate_trajectory(self.current_pose, target_pose)
        
        # Execute trajectory in separate thread
        nav_thread = threading.Thread(target=self.execute_trajectory, args=(trajectory,), daemon=True)
        nav_thread.start()
        
    def navigate_waypoint_sequence(self, waypoints):
        """Navigate through sequence of waypoints"""
        if self.is_navigating:
            self.get_logger().warn('Already navigating!')
            return
            
        self.get_logger().info(f'Generating multi-waypoint trajectory with {len(waypoints)} waypoints')
        
        # Generate complete trajectory
        trajectory = self.trajectory_generator.generate_multi_waypoint_trajectory(waypoints)
        
        # Execute trajectory in separate thread
        nav_thread = threading.Thread(target=self.execute_trajectory, args=(trajectory,), daemon=True)
        nav_thread.start()
        
    def stop_navigation(self):
        """Stop current navigation"""
        self.running = False
        self.is_navigating = False
        
        # Send stop command
        stop_msg = Float32MultiArray()
        stop_msg.data = [0.0, 0.0]
        self.publisher.publish(stop_msg)
        
        self.get_logger().info('Navigation stopped')
        
    def reset_pose(self):
        """Reset robot pose to origin"""
        self.current_pose.x = 0.0
        self.current_pose.y = 0.0
        self.current_pose.theta = 0.0
        self.get_logger().info('Robot pose reset to origin (0, 0, 0)')


def main():
    parser = argparse.ArgumentParser(description='Advanced Waypoint Navigation Node')
    parser.add_argument('--x', type=float, default=0.0, help='Target X position (meters)')
    parser.add_argument('--y', type=float, default=0.0, help='Target Y position (meters)')
    parser.add_argument('--theta', type=float, default=0.0, help='Target orientation (degrees)')
    parser.add_argument('--waypoints', type=str, help='Comma-separated waypoints: "x1,y1,theta1;x2,y2,theta2"')
    parser.add_argument('--reset', action='store_true', help='Reset robot pose to origin')
    parser.add_argument('--stop', action='store_true', help='Stop current navigation')
    parser.add_argument('--status', action='store_true', help='Show current robot status')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = AdvancedWaypointNavigationNode()
    
    try:
        if args.reset:
            node.reset_pose()
        elif args.stop:
            node.stop_navigation()
        elif args.status:
            node.get_logger().info(f'Current pose: ({node.current_pose.x:.3f}, {node.current_pose.y:.3f}, {math.degrees(node.current_pose.theta):.1f}°)')
        elif args.waypoints:
            # Parse waypoint sequence
            waypoints = []
            for wp_str in args.waypoints.split(';'):
                x, y, theta = map(float, wp_str.split(','))
                waypoints.append((x, y, theta))
            node.navigate_waypoint_sequence(waypoints)
        else:
            # Convert theta from degrees to radians
            target_theta = math.radians(args.theta)
            node.navigate_to_waypoint(args.x, args.y, target_theta)
            
            # Keep node alive to monitor navigation
            while rclpy.ok() and node.is_navigating:
                rclpy.spin_once(node, timeout_sec=0.1)
                
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.stop_navigation()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
