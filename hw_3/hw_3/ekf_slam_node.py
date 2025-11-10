#!/usr/bin/env python3
"""
EKF SLAM ROS2 Node
Main SLAM processing node that integrates with ROS2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import numpy as np
import math
import time

# Try to import apriltag_msgs, fallback if not available
try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    print("Warning: apriltag_msgs not available. AprilTag detection will be disabled.")

try:
    from .ekf_slam_core import (
        ekf_slam, ROBOT_STATE_SIZE, LM_SIZE, calc_n_lm,
        get_landmark_position_from_state, pi_2_pi
    )
except ImportError:
    from ekf_slam_core import (
        ekf_slam, ROBOT_STATE_SIZE, LM_SIZE, calc_n_lm,
        get_landmark_position_from_state, pi_2_pi
    )


class EKF_SLAM_Node(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')
        
        # Declare parameters
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('m_dist_threshold', 2.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('process_noise_v', 0.2)
        self.declare_parameter('process_noise_w', 0.1)
        self.declare_parameter('measurement_noise_r', 0.1)
        self.declare_parameter('measurement_noise_b', 0.1)
        self.declare_parameter('state_cov_x', 0.5)
        self.declare_parameter('state_cov_y', 0.5)
        self.declare_parameter('state_cov_yaw', 0.5)
        
        # Get parameters
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.m_dist_threshold = self.get_parameter('m_dist_threshold').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        process_noise_v = self.get_parameter('process_noise_v').get_parameter_value().double_value
        process_noise_w = self.get_parameter('process_noise_w').get_parameter_value().double_value
        measurement_noise_r = self.get_parameter('measurement_noise_r').get_parameter_value().double_value
        measurement_noise_b = self.get_parameter('measurement_noise_b').get_parameter_value().double_value
        state_cov_x = self.get_parameter('state_cov_x').get_parameter_value().double_value
        state_cov_y = self.get_parameter('state_cov_y').get_parameter_value().double_value
        state_cov_yaw = self.get_parameter('state_cov_yaw').get_parameter_value().double_value
        
        # Build noise matrices
        self.Q = np.diag([process_noise_v, process_noise_w]) ** 2
        self.R = np.diag([measurement_noise_r, measurement_noise_b]) ** 2
        self.Cx = np.diag([state_cov_x, state_cov_y, state_cov_yaw]) ** 2
        
        # Initialize TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize EKF state
        self.xEst = np.zeros((ROBOT_STATE_SIZE, 1))  # Start at [0, 0, 0]
        self.PEst = np.eye(ROBOT_STATE_SIZE) * np.diag([state_cov_x, state_cov_y, state_cov_yaw]) ** 2
        
        # Storage
        self.trajectory_history = []
        self.last_u = np.array([[0.0], [0.0]])
        self.last_u_time = None
        
        # Latest detections
        self.latest_detections = None
        self.latest_detections_time = None
        
        # Subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        if APRILTAG_AVAILABLE:
            self.detections_sub = self.create_subscription(
                AprilTagDetectionArray,
                '/detections',
                self.detections_callback,
                10
            )
        else:
            self.get_logger().warn('AprilTag detections disabled - apriltag_msgs not available')
        
        # Publishers
        self.robot_pose_pub = self.create_publisher(
            PoseStamped,
            '/ekf_slam/robot_pose',
            10
        )
        
        # Timer for EKF update
        self.ekf_timer = self.create_timer(self.dt, self.ekf_update)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('EKF SLAM Node initialized')
        self.get_logger().info(f'Update period: {self.dt}s')
        self.get_logger().info(f'Max range: {self.max_range}m')
        self.get_logger().info(f'Mahalanobis threshold: {self.m_dist_threshold}')
        self.get_logger().info(f'Process noise Q: {self.Q}')
        self.get_logger().info(f'Measurement noise R: {self.R}')
        self.get_logger().info(f'State covariance Cx: {self.Cx}')
        self.get_logger().info(f'Initial state: {self.xEst.flatten()}')
        self.get_logger().info('=' * 60)
    
    def cmd_vel_callback(self, msg):
        """Store latest control input"""
        self.last_u = np.array([[msg.linear.x], [msg.angular.z]])
        self.last_u_time = self.get_clock().now()
    
    def detections_callback(self, msg):
        """Store latest detections"""
        num_detections = len(msg.detections)
        self.get_logger().info(f'EKF SLAM received {num_detections} AprilTag detection(s) on /detections topic')
        
        # Log details of each detection
        for i, detection in enumerate(msg.detections):
            self.get_logger().info(f'  Detection {i+1}: tag_id={detection.id}')
        
        self.latest_detections = msg
        self.latest_detections_time = self.get_clock().now()
    
    def convert_detections_to_measurements(self):
        """
        Convert AprilTag detections to range/bearing measurements
        
        Returns:
            measurements: Nx2 array where each row is [range, bearing]
        """
        measurements = []
        
        if self.latest_detections is None:
            return np.array([]).reshape(0, 2)
        
        # Check if detections are fresh (< 0.25s old)
        if self.latest_detections_time is not None:
            time_diff = (self.get_clock().now() - self.latest_detections_time).nanoseconds / 1e9
            if time_diff > 0.25:
                return np.array([]).reshape(0, 2)
        
        # Get current robot yaw from state
        robot_yaw = self.xEst[2, 0]
        
        for detection in self.latest_detections.detections:
            try:
                # Get tag ID
                tag_id = detection.id
                
                # Lookup TF transform: base_link -> tag_<id>
                # Try different possible frame names
                possible_frames = [
                    f'tag_{tag_id}',
                    f'tag36h11:{tag_id}',
                    f'tag_{tag_id:02d}',
                ]
                
                transform = None
                for frame_name in possible_frames:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            'base_link',
                            frame_name,
                            rclpy.time.Time()
                        )
                        break
                    except Exception:
                        continue
                
                if transform is None:
                    continue
                
                # Extract translation
                dx = transform.transform.translation.x
                dy = transform.transform.translation.y
                dz = transform.transform.translation.z
                
                # Calculate range
                r = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Filter by max_range
                if r > self.max_range:
                    continue
                
                # Calculate bearing
                b = math.atan2(dy, dx) - robot_yaw
                b = pi_2_pi(b)
                
                # Append measurement [range, bearing]
                measurements.append([r, b])
                
            except Exception as e:
                self.get_logger().debug(f'Error processing detection: {e}')
                continue
        
        if len(measurements) == 0:
            return np.array([]).reshape(0, 2)
        
        return np.array(measurements)
    
    def ekf_update(self):
        """Main EKF update loop"""
        # Get control input
        u = self.last_u.copy()
        
        # Get measurements
        z = self.convert_detections_to_measurements()
        
        # Log when measurements are used in EKF
        if len(z) > 0:
            self.get_logger().info(f'EKF SLAM processing {len(z)} measurement(s) in update step')
            for i, meas in enumerate(z):
                self.get_logger().info(f'  Measurement {i+1}: range={meas[0]:.3f}m, bearing={meas[1]:.3f}rad')
        
        # Run EKF SLAM
        self.xEst, self.PEst = ekf_slam(
            self.xEst, self.PEst, u, z,
            self.dt, self.Q, self.R, self.Cx, self.m_dist_threshold
        )
        
        # Store trajectory history
        robot_state = self.xEst[0:ROBOT_STATE_SIZE].copy()
        self.trajectory_history.append(robot_state)
        
        # Publish robot pose
        self.publish_robot_pose()
        
        # Broadcast TF
        self.broadcast_tf()
        
        # Log landmark count
        nLM = calc_n_lm(self.xEst)
        if len(self.trajectory_history) % 10 == 0:  # Log every 10 updates
            self.get_logger().debug(f'Landmarks: {nLM}, Robot pose: [{robot_state[0,0]:.2f}, {robot_state[1,0]:.2f}, {robot_state[2,0]:.2f}]')
    
    def publish_robot_pose(self):
        """Publish estimated robot pose"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        
        msg.pose.position.x = float(self.xEst[0, 0])
        msg.pose.position.y = float(self.xEst[1, 0])
        msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.xEst[2, 0]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        msg.pose.orientation.w = cy
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sy
        
        self.robot_pose_pub.publish(msg)
    
    def broadcast_tf(self):
        """Broadcast TF transform: odom -> base_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = float(self.xEst[0, 0])
        t.transform.translation.y = float(self.xEst[1, 0])
        t.transform.translation.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.xEst[2, 0]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        t.transform.rotation.w = cy
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EKF_SLAM_Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('EKF SLAM Node stopped by keyboard interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

