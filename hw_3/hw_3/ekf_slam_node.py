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
        # Yaw calibration parameters (to fix non-linear turn error)
        self.declare_parameter('angular_velocity_scale', 1.0)  # Scale factor: if robot turns 80° when commanded 90°, use 90/80 = 1.125
        self.declare_parameter('angular_velocity_bias', 0.0)   # Constant bias (rad/s)
        
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
        self.angular_velocity_scale = self.get_parameter('angular_velocity_scale').get_parameter_value().double_value
        self.angular_velocity_bias = self.get_parameter('angular_velocity_bias').get_parameter_value().double_value
        
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
        self.is_turning = False  # Flag to detect pure rotation commands
        self.prev_is_turning = False  # Track previous state to detect transitions
        
        # Latest detections
        self.latest_detections = None
        self.latest_detections_time = None
        
        # Tag ID to landmark index mapping
        # Maps AprilTag ID -> landmark index in state vector
        self.tag_id_to_landmark_index = {}
        
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
        
        # Track previous values for change detection
        self.prev_robot_state = np.array([[0.0], [0.0], [0.0]])
        self.update_count = 0
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('EKF SLAM Node initialized')
        self.get_logger().info(f'Update period: {self.dt}s | Max range: {self.max_range}m | Threshold: {self.m_dist_threshold}')
        self.get_logger().info('=' * 70)
    
    def cmd_vel_callback(self, msg):
        """Store latest control input and detect turn commands"""
        self.last_u = np.array([[msg.linear.x], [msg.angular.z]])
        self.last_u_time = self.get_clock().now()
        
        # Detect turn command: linear velocity near zero but angular velocity non-zero
        # This indicates pure rotation (wheels turning in opposite directions)
        linear_threshold = 0.01  # 1cm/s threshold
        angular_threshold = 0.05  # ~3 deg/s threshold
        
        self.prev_is_turning = self.is_turning  # Store previous state
        
        if abs(msg.linear.x) < linear_threshold and abs(msg.angular.z) > angular_threshold:
            self.is_turning = True
            # Log when we first detect turning (transition from False to True)
            if not self.prev_is_turning:
                self.get_logger().info(f'[TURN DETECTED] Pure rotation command: v={msg.linear.x:.3f}m/s, ω={math.degrees(msg.angular.z):.2f}°/s')
        else:
            self.is_turning = False
    
    def detections_callback(self, msg):
        """Store latest detections"""
        #if len(msg.detections) > 0:
            #tag_ids = [str(d.id) for d in msg.detections]
            #self.get_logger().info(f'[DETECTIONS] {len(msg.detections)} tag(s): {", ".join(tag_ids)}')
        self.latest_detections = msg
        self.latest_detections_time = self.get_clock().now()
    
    def convert_detections_to_measurements(self):
        """
        Convert AprilTag detections to range/bearing measurements with tag IDs
        
        Returns:
            measurements_with_ids: List of tuples (tag_id, range, bearing)
        """
        measurements_with_ids = []
        
        if self.latest_detections is None:
            return []
        
        # Check if detections are fresh (< 0.25s old)
        if self.latest_detections_time is not None:
            time_diff = (self.get_clock().now() - self.latest_detections_time).nanoseconds / 1e9
            if time_diff > 0.25:
                return []
        
        for detection in self.latest_detections.detections:
            tag_id = detection.id
            
            # apriltag_ros creates TF frame as tag_{tag_id}
            frame_name = f'tag_{tag_id}'
            
            # Lookup TF transform: base_link -> tag_{tag_id}
            if not self.tf_buffer.can_transform('base_link', frame_name, rclpy.time.Time()):
                continue
            
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                frame_name,
                rclpy.time.Time()
            )
            
            # Extract translation
            dx = transform.transform.translation.x
            dy = transform.transform.translation.y
            dz = transform.transform.translation.z
            
            # Calculate range (2D for ground-plane SLAM)
            r = math.sqrt(dx*dx + dy*dy)
            
            # Filter by max_range
            if r > self.max_range:
                continue
            
            # Calculate bearing (relative to robot heading in base_link frame)
            b = math.atan2(dy, dx)
            b = pi_2_pi(b)
            
            # Append (tag_id, range, bearing)
            measurements_with_ids.append((tag_id, r, b))
        
        return measurements_with_ids
    
    def ekf_update(self):
        """Main EKF update loop"""
        self.update_count += 1
        
        # Get control input - check if it's stale
        # If no cmd_vel received in last 0.2s, assume robot stopped
        cmd_vel_timeout = 0.2  # 200ms timeout
        if self.last_u_time is None:
            # No command ever received, use zero
            u = np.array([[0.0], [0.0]])
        else:
            time_since_cmd = (self.get_clock().now() - self.last_u_time).nanoseconds / 1e9
            if time_since_cmd > cmd_vel_timeout:
                # Command is stale - robot has stopped
                u = np.array([[0.0], [0.0]])
                # Also reset is_turning flag
                if self.is_turning:
                    self.is_turning = False
            else:
                # Command is fresh, use it
                u = self.last_u.copy()
        
        # Get landmark count before update
        nLM_before = calc_n_lm(self.xEst)
        
        # Get measurements with tag IDs
        measurements_with_ids = self.convert_detections_to_measurements()
        
        if len(measurements_with_ids) == 0:
            # No measurements, just predict
            z = np.array([]).reshape(0, 2)
            landmark_indices = None
        else:
            # Separate measurements and tag IDs
            tag_ids = [m[0] for m in measurements_with_ids]
            measurements = np.array([[m[1], m[2]] for m in measurements_with_ids])  # [range, bearing]
            z = measurements
            
            # Map tag IDs to landmark indices using tag ID directly
            # Known tags: use existing mapping
            # New tags: assign sequential indices starting from nLM_before
            landmark_indices = []
            new_tag_counter = 0
            
            for tag_id in tag_ids:
                if tag_id in self.tag_id_to_landmark_index:
                    # Known tag: use existing landmark index
                    landmark_indices.append(self.tag_id_to_landmark_index[tag_id])
                else:
                    # New tag: will be created at next available index
                    landmark_indices.append(nLM_before + new_tag_counter)
                    new_tag_counter += 1
            
            # Log which tags we're processing
            if len(tag_ids) > 0:
                new_tags = [tid for tid in tag_ids if tid not in self.tag_id_to_landmark_index]

                if new_tags:
                    self.get_logger().info(f'[TAGS] New: {new_tags}')
        
        # Apply yaw calibration to control input
        # Calibrate angular velocity to account for non-linear turn error
        u_calibrated = u.copy()
        u_calibrated[1, 0] = u[1, 0] * self.angular_velocity_scale + self.angular_velocity_bias
        
        # Run EKF SLAM with tag ID-based data association
        # Pass is_turning flag to prevent position updates during pure rotation
        self.xEst, self.PEst = ekf_slam(
            self.xEst, self.PEst, u_calibrated, z,
            self.dt, self.Q, self.R, self.Cx, self.m_dist_threshold,
            landmark_indices=landmark_indices,
            is_turning=self.is_turning
        )
        
        # Get robot state and landmark count after update
        robot_state = self.xEst[0:ROBOT_STATE_SIZE].copy()
        nLM_after = calc_n_lm(self.xEst)
        
        # Update tag ID to landmark index mapping for new landmarks
        # EKF SLAM behavior:
        # - First detection: Creates new landmark in state vector (extends xEst and PEst)
        # - Re-observation: Updates existing landmark using Kalman filter (improves estimate, reduces uncertainty)
        if len(measurements_with_ids) > 0:
            tag_ids = [m[0] for m in measurements_with_ids]
            
            # Check for new landmarks (state vector grew)
            if nLM_after > nLM_before:
                # Map new tag IDs to newly created landmarks
                new_tag_index = 0
                for tag_id in tag_ids:
                    if tag_id not in self.tag_id_to_landmark_index:
                        landmark_idx = nLM_before + new_tag_index
                        self.tag_id_to_landmark_index[tag_id] = landmark_idx
                        lm = get_landmark_position_from_state(self.xEst, landmark_idx)
                        self.get_logger().info(f'[NEW LANDMARK] Tag {tag_id} at [{lm[0,0]:.3f}, {lm[1,0]:.3f}]m')
                        new_tag_index += 1
                        if new_tag_index >= (nLM_after - nLM_before):
                            break
            
            # Log when we're updating existing landmarks (re-observation)
            # Note: This happens every time we re-observe a known tag - the EKF updates the landmark estimate
            # The Kalman filter improves the estimate and reduces uncertainty with each observation
            known_tags_observed = [tid for tid in tag_ids if tid in self.tag_id_to_landmark_index]
            if known_tags_observed and nLM_after == nLM_before:
                # Throttle: only log updates every 10 cycles to avoid spam
                if self.update_count % 10 == 0:
                    tag_str = ', '.join([str(tid) for tid in known_tags_observed])
                    #self.get_logger().info(f'[UPDATE LANDMARKS] Re-observing tags: {tag_str} (improving estimates)')
        
        # Detect robot state changes
        state_change = np.linalg.norm(robot_state - self.prev_robot_state)
        if state_change > 0.01:  # Log if moved more than 1cm
            dx = robot_state[0, 0] - self.prev_robot_state[0, 0]
            dy = robot_state[1, 0] - self.prev_robot_state[1, 0]
            dyaw = robot_state[2, 0] - self.prev_robot_state[2, 0]
            #self.get_logger().info(f'[ROBOT MOVE] Δx={dx:.3f}m, Δy={dy:.3f}m, Δyaw={math.degrees(dyaw):.2f}°')
            self.prev_robot_state = robot_state.copy()
        
        # Store trajectory history
        self.trajectory_history.append(robot_state)
        
        # Publish robot pose
        self.publish_robot_pose()
        
        # Broadcast robot TF: odom -> base_link (EKF estimated pose)
        self.broadcast_tf()
        
        # Note: Landmark positions are in EKF state vector and published via robot_pose topic
        # AprilTag already broadcasts tag_{tag_id} relative to base_link for detections
        # EKF estimates landmarks in odom frame, but we don't need to broadcast them as TF
        # since AprilTag TFs provide the detection data we need
        
        # Periodic status update every 50 updates (~5 seconds at 10Hz)
        if self.update_count % 50 == 0:
            self.log_status(robot_state, u, z, nLM_after)
            self.print_state_vector()
    
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
    
    def log_status(self, robot_state, u, z, nLM):
        """Log comprehensive status update"""
        self.get_logger().info('-' * 70)
        self.get_logger().info(f'[EKF STATUS] Update #{self.update_count} | Runtime: {self.update_count * self.dt:.1f}s')
        self.get_logger().info(f'  Robot Pose: x={robot_state[0,0]:.3f}m, y={robot_state[1,0]:.3f}m, yaw={math.degrees(robot_state[2,0]):.1f}°')
        self.get_logger().info(f'  Control: v={u[0,0]:.3f}m/s, ω={math.degrees(u[1,0]):.1f}°/s')
        self.get_logger().info(f'  Measurements: {len(z)} tag(s) in this update')
        self.get_logger().info(f'  Landmarks: {nLM} total')
        
        # Log landmark positions and covariance
        if nLM > 0:
            self.get_logger().info('  Landmark Positions:')
            # Sort by tag ID for consistent output
            tag_id_list = sorted(self.tag_id_to_landmark_index.keys())
            for tag_id in tag_id_list[:5]:  # Show up to 5 landmarks
                landmark_idx = self.tag_id_to_landmark_index[tag_id]
                lm = get_landmark_position_from_state(self.xEst, landmark_idx)
                # Get covariance for this landmark
                lm_start_idx = ROBOT_STATE_SIZE + LM_SIZE * landmark_idx
                cov_xx = self.PEst[lm_start_idx, lm_start_idx]
                cov_yy = self.PEst[lm_start_idx + 1, lm_start_idx + 1]
                std_x = math.sqrt(cov_xx)
                std_y = math.sqrt(cov_yy)
                self.get_logger().info(f'    Tag {tag_id}: [{lm[0,0]:.3f}, {lm[1,0]:.3f}]m ± [{std_x:.3f}, {std_y:.3f}]m')
            if len(tag_id_list) > 5:
                self.get_logger().info(f'    ... and {len(tag_id_list) - 5} more landmark(s)')
        
        # Log robot pose uncertainty
        robot_cov_xx = self.PEst[0, 0]
        robot_cov_yy = self.PEst[1, 1]
        robot_cov_yaw = self.PEst[2, 2]
        self.get_logger().info(f'  Robot Uncertainty: σ_x={math.sqrt(robot_cov_xx):.3f}m, σ_y={math.sqrt(robot_cov_yy):.3f}m, σ_yaw={math.degrees(math.sqrt(robot_cov_yaw)):.1f}°')
        self.get_logger().info('-' * 70)
    
    def print_state_vector(self):
        """Print the current state vector in a readable format"""
        nLM = calc_n_lm(self.xEst)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('[STATE VECTOR]')
        
        # Robot state
        robot_x = self.xEst[0, 0]
        robot_y = self.xEst[1, 0]
        robot_yaw = self.xEst[2, 0]
        self.get_logger().info(f'  Robot: x={robot_x:.4f}m, y={robot_y:.4f}m, yaw={math.degrees(robot_yaw):.2f}°')
        
        # Landmarks (sorted by tag ID)
        if nLM > 0:
            self.get_logger().info(f'  Landmarks ({nLM} total):')
            tag_id_list = sorted(self.tag_id_to_landmark_index.keys())
            for tag_id in tag_id_list:
                landmark_idx = self.tag_id_to_landmark_index[tag_id]
                lm_start_idx = ROBOT_STATE_SIZE + LM_SIZE * landmark_idx
                lm_x = self.xEst[lm_start_idx, 0]
                lm_y = self.xEst[lm_start_idx + 1, 0]
                self.get_logger().info(f'    Tag {tag_id}: x={lm_x:.4f}m, y={lm_y:.4f}m')
        
        # State vector size
        state_size = len(self.xEst)
        self.get_logger().info(f'  State vector size: {state_size} (robot: {ROBOT_STATE_SIZE}, landmarks: {state_size - ROBOT_STATE_SIZE})')
        self.get_logger().info('=' * 70)


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

