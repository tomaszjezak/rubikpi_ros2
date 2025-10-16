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


class WaypointNavigationNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigation_node')
        
        # Create publisher for motor commands
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'motor_commands',
            10
        )
        
        # Robot state
        self.current_pose = Pose2D()  # x, y, theta (in meters and radians)
        self.target_pose = Pose2D()
        self.is_navigating = False
        self.running = True
        
        # Motion parameters (calibrated for your robot)
        self.FORWARD_DISTANCE_PER_COMMAND = 0.034  # 3.4 cm per forward command
        self.ROTATION_ANGLE_PER_COMMAND = math.radians(21)  # 21 degrees per rotation command
        self.FORWARD_SPEED = 0.3  # Motor speed for forward motion
        self.ROTATION_SPEED = 0.5  # Motor speed for rotation
        
        # Command timing
        self.COMMAND_DURATION = 0.1  # Duration to hold each command (seconds)
        
        self.get_logger().info('Waypoint Navigation Node started')
        self.get_logger().info(f'Motion parameters:')
        self.get_logger().info(f'  Forward: {self.FORWARD_DISTANCE_PER_COMMAND*100:.1f} cm per command')
        self.get_logger().info(f'  Rotation: {math.degrees(self.ROTATION_ANGLE_PER_COMMAND):.1f} degrees per command')
        
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
        self.current_pose.x += distance
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
        
    def plan_path_to_waypoint(self, target_x, target_y, target_theta):
        """Plan path from current pose to target waypoint"""
        self.target_pose.x = target_x
        self.target_pose.y = target_y
        self.target_pose.theta = target_theta
        
        self.get_logger().info(f'Planning path to waypoint: ({target_x:.3f}, {target_y:.3f}, {math.degrees(target_theta):.1f}°)')
        self.get_logger().info(f'Starting from: ({self.current_pose.x:.3f}, {self.current_pose.y:.3f}, {math.degrees(self.current_pose.theta):.1f}°)')
        
        # Calculate relative movement
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.01:  # Already at target position
            self.get_logger().info('Already at target position')
            # Just rotate to target orientation
            angle_diff = target_theta - self.current_pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))  # Normalize
            if abs(angle_diff) > 0.01:
                self.rotate(angle_diff)
            return
            
        # Calculate required heading to reach target
        required_heading = math.atan2(dy, dx)
        
        # Calculate rotation needed to face target
        heading_diff = required_heading - self.current_pose.theta
        heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))  # Normalize
        
        # Calculate final rotation to reach target orientation
        final_rotation = target_theta - required_heading
        final_rotation = math.atan2(math.sin(final_rotation), math.cos(final_rotation))  # Normalize
        
        return [
            ('rotate', heading_diff),
            ('forward', distance),
            ('rotate', final_rotation)
        ]
        
    def execute_path(self, path):
        """Execute planned path"""
        self.is_navigating = True
        self.get_logger().info(f'Executing path with {len(path)} steps')
        
        try:
            for step_type, value in path:
                if not self.running:
                    break
                    
                if step_type == 'rotate':
                    self.rotate(value)
                elif step_type == 'forward':
                    self.move_forward(value)
                    
            self.get_logger().info('Path execution completed!')
            self.get_logger().info(f'Final pose: ({self.current_pose.x:.3f}, {self.current_pose.y:.3f}, {math.degrees(self.current_pose.theta):.1f}°)')
            
        except Exception as e:
            self.get_logger().error(f'Error during path execution: {str(e)}')
        finally:
            self.is_navigating = False
            
    def navigate_to_waypoint(self, x, y, theta):
        """Main navigation function"""
        if self.is_navigating:
            self.get_logger().warn('Already navigating to a waypoint!')
            return
            
        self.get_logger().info(f'Starting navigation to ({x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°)')
        
        # Plan path
        path = self.plan_path_to_waypoint(x, y, theta)
        
        # Execute path in separate thread to avoid blocking
        nav_thread = threading.Thread(target=self.execute_path, args=(path,), daemon=True)
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
    parser = argparse.ArgumentParser(description='Waypoint Navigation Node')
    parser.add_argument('--x', type=float, default=0.0, help='Target X position (meters)')
    parser.add_argument('--y', type=float, default=0.0, help='Target Y position (meters)')
    parser.add_argument('--theta', type=float, default=0.0, help='Target orientation (degrees)')
    parser.add_argument('--reset', action='store_true', help='Reset robot pose to origin')
    parser.add_argument('--stop', action='store_true', help='Stop current navigation')
    parser.add_argument('--status', action='store_true', help='Show current robot status')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = WaypointNavigationNode()
    
    try:
        if args.reset:
            node.reset_pose()
        elif args.stop:
            node.stop_navigation()
        elif args.status:
            node.get_logger().info(f'Current pose: ({node.current_pose.x:.3f}, {node.current_pose.y:.3f}, {math.degrees(node.current_pose.theta):.1f}°)')
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
