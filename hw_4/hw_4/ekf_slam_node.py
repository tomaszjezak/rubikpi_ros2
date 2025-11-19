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
import pickle
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory

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
    def load_ground_truth_landmarks(self):
        """Load ground truth landmarks from YAML file"""
        try:
            # Use ROS2 package share directory to find config
            pkg_share = get_package_share_directory('hw_4')
            config_path = os.path.join(pkg_share, 'configs', 'ground_truth_landmarks.yaml')
        except Exception as e:
            self.get_logger().error(f'Could not find hw_4 package: {e}')
            return {}

        if not os.path.exists(config_path):
            self.get_logger().error(f'Landmark config not found at {config_path}')
            return {}

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        landmarks = {}
        for lm in config['landmarks']:
            tag_id = lm['tag_id']
            x = lm['x']
            y = lm['y']
            landmarks[tag_id] = (x, y)

        self.get_logger().info(f'Loaded {len(landmarks)} ground truth landmarks from {config_path}')
        return landmarks

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

        # Load ground truth landmarks
        ground_truth_landmarks = self.load_ground_truth_landmarks()

        # Initialize EKF state with robot + all known landmarks
        self.xEst = np.zeros((ROBOT_STATE_SIZE, 1))  # Start at [0, 0, 0]

        # Pre-populate tag_id to landmark index mapping
        self.tag_id_to_landmark_index = {}

        # Add all known landmarks to state vector
        landmark_uncertainty = 0.01  # 1cm standard deviation (very confident in landmark positions)
        for landmark_idx, (tag_id, (lm_x, lm_y)) in enumerate(sorted(ground_truth_landmarks.items())):
            # Add landmark to state vector
            self.xEst = np.vstack((self.xEst, np.array([[lm_x], [lm_y]])))
            # Map tag_id to landmark index
            self.tag_id_to_landmark_index[tag_id] = landmark_idx
            self.get_logger().info(f'Pre-loaded landmark: Tag {tag_id} at [{lm_x:.4f}, {lm_y:.4f}]m (index {landmark_idx})')

        # Initialize covariance matrix
        # Robot part: use parameters
        robot_cov = np.diag([state_cov_x, state_cov_y, state_cov_yaw]) ** 2

        # Landmark part: very small uncertainty (landmarks are known)
        num_landmarks = len(ground_truth_landmarks)
        landmark_cov = np.eye(LM_SIZE * num_landmarks) * (landmark_uncertainty ** 2)

        # Build full covariance matrix
        self.PEst = np.zeros((ROBOT_STATE_SIZE + LM_SIZE * num_landmarks,
                              ROBOT_STATE_SIZE + LM_SIZE * num_landmarks))
        self.PEst[0:ROBOT_STATE_SIZE, 0:ROBOT_STATE_SIZE] = robot_cov
        self.PEst[ROBOT_STATE_SIZE:, ROBOT_STATE_SIZE:] = landmark_cov
        
        # Storage
        self.trajectory_history = []  # Full trajectory at EKF update rate
        self.trajectory_1hz = []  # Trajectory sampled at 1Hz for plotting
        self.waypoint_timestamps = []  # List of (waypoint_idx, timestamp, x, y, yaw)
        self.last_u = np.array([[0.0], [0.0]])
        self.last_u_time = None
        self.is_turning = False  # Flag to detect pure rotation commands
        self.prev_is_turning = False  # Track previous state to detect transitions
        
        # Waypoint detection - use same waypoints and tolerance as hw4.py
        # These match the waypoints defined in hw4.py
        self.waypoints = np.array([
            [ 0.000000,  0.850000,  0.0],
            [-0.850000,  0.850000, 1.5708],
            [-0.850000,  0.000000,  3.14159],
            [ 0.000000,  0.000000,  -1.5708],
        ])
        self.waypoint_tolerance = 0.15  # meters - same as hw4.py tolerance
        self.reached_waypoints = set()  # Track which waypoints we've already logged
        
        # Latest detections
        self.latest_detections = None
        self.latest_detections_time = None

        # Note: tag_id_to_landmark_index is now initialized earlier with ground truth landmarks

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
        
        # Timer for 1Hz trajectory logging
        self.trajectory_log_timer = self.create_timer(1.0, self.log_trajectory_1hz)
        
        # Track previous values for change detection
        self.prev_robot_state = np.array([[0.0], [0.0], [0.0]])
        self.update_count = 0
        
        # Data file path for saving
        self.data_file_path = os.path.join(
            os.path.expanduser('~'), '.ros', 'slam_data.pkl'
        )
        # Ensure directory exists
        os.makedirs(os.path.dirname(self.data_file_path), exist_ok=True)
        
        # Broadcast initial TF immediately so odom frame exists right away
        # This ensures other nodes (like hw4.py) can look up odom -> base_link immediately
        self.broadcast_tf()
        
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
    
    def check_waypoint_reached(self, robot_state):
        """Check if robot is near any waypoint and log it if reached"""
        robot_x = robot_state[0, 0]
        robot_y = robot_state[1, 0]
        robot_yaw = robot_state[2, 0]
        
        for waypoint_idx, waypoint in enumerate(self.waypoints):
            # Skip if already logged
            if waypoint_idx in self.reached_waypoints:
                continue
            
            wp_x, wp_y, wp_yaw = waypoint
            # Check distance to waypoint
            distance = math.sqrt((robot_x - wp_x)**2 + (robot_y - wp_y)**2)
            
            if distance < self.waypoint_tolerance:
                # Waypoint reached!
                timestamp = self.get_clock().now().nanoseconds / 1e9
                self.waypoint_timestamps.append((
                    waypoint_idx,
                    timestamp,
                    float(robot_x),
                    float(robot_y),
                    float(robot_yaw)
                ))
                self.reached_waypoints.add(waypoint_idx)
                self.get_logger().info(f'[WAYPOINT REACHED] Waypoint {waypoint_idx} at [{robot_x:.3f}, {robot_y:.3f}], yaw={math.degrees(robot_yaw):.1f}°')
                break  # Only log one waypoint per check
    
    def log_trajectory_1hz(self):
        """Log trajectory data at 1Hz for visualization"""
        robot_state = self.xEst[0:ROBOT_STATE_SIZE].copy()
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.trajectory_1hz.append((
            timestamp,
            float(robot_state[0, 0]),
            float(robot_state[1, 0]),
            float(robot_state[2, 0])
        ))
    
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
            z = np.array([[m[1], m[2]] for m in measurements_with_ids])  # [range, bearing]

            # Map tag IDs to landmark indices - will crash if unknown tag (intentional)
            landmark_indices = [self.tag_id_to_landmark_index[tag_id] for tag_id in tag_ids]
        
        # Run EKF SLAM with tag ID-based data association
        # Pass is_turning flag to prevent position updates during pure rotation
        self.xEst, self.PEst = ekf_slam(
            self.xEst, self.PEst, u, z,
            self.dt, self.Q, self.R, self.Cx, self.m_dist_threshold,
            landmark_indices=landmark_indices,
            is_turning=self.is_turning
        )
        
        # Get robot state and landmark count after update
        robot_state = self.xEst[0:ROBOT_STATE_SIZE].copy()
        nLM_after = calc_n_lm(self.xEst)

        # Sanity check: state vector should never grow (all landmarks pre-loaded)
        if nLM_after != nLM_before:
            self.get_logger().error(f'[ERROR] State vector changed size! Before: {nLM_before}, After: {nLM_after}')
            self.get_logger().error('This should never happen with pre-loaded landmarks!')
        
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
        
        # Check if waypoint reached
        self.check_waypoint_reached(robot_state)
        
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
            self.get_logger().info('  Landmark Positions and Covariances:')
            # Sort by tag ID for consistent output
            tag_id_list = sorted(self.tag_id_to_landmark_index.keys())
            for tag_id in tag_id_list:
                landmark_idx = self.tag_id_to_landmark_index[tag_id]
                lm = get_landmark_position_from_state(self.xEst, landmark_idx)
                # Get covariance for this landmark
                lm_start_idx = ROBOT_STATE_SIZE + LM_SIZE * landmark_idx
                cov_xx = self.PEst[lm_start_idx, lm_start_idx]
                cov_yy = self.PEst[lm_start_idx + 1, lm_start_idx + 1]
                cov_xy = self.PEst[lm_start_idx, lm_start_idx + 1]
                std_x = math.sqrt(cov_xx)
                std_y = math.sqrt(cov_yy)
                # Calculate correlation coefficient
                correlation = cov_xy / (std_x * std_y) if (std_x * std_y) > 0 else 0.0
                # Calculate 2σ uncertainty ellipse semi-axes (95% confidence)
                eigenvals, eigenvecs = np.linalg.eigh(np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]]))
                semi_major = 2.0 * math.sqrt(max(eigenvals))  # 2σ
                semi_minor = 2.0 * math.sqrt(min(eigenvals))  # 2σ
                self.get_logger().info(f'    Tag {tag_id}: pos=[{lm[0,0]:.4f}, {lm[1,0]:.4f}]m')
                self.get_logger().info(f'      Covariance: σ_x={std_x:.4f}m, σ_y={std_y:.4f}m, σ_xy={cov_xy:.6f}')
                self.get_logger().info(f'      Correlation: {correlation:.4f}, 2σ ellipse: [{semi_major:.4f}, {semi_minor:.4f}]m')
        
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
    
    def log_landmark_covariances(self):
        """Log detailed covariance information for all landmarks"""
        nLM = calc_n_lm(self.xEst)
        if nLM == 0:
            return
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('[LANDMARK COVARIANCE REPORT]')
        self.get_logger().info(f'Total landmarks: {nLM}')
        
        # Sort by tag ID for consistent output
        tag_id_list = sorted(self.tag_id_to_landmark_index.keys())
        
        for tag_id in tag_id_list:
            landmark_idx = self.tag_id_to_landmark_index[tag_id]
            lm = get_landmark_position_from_state(self.xEst, landmark_idx)
            lm_start_idx = ROBOT_STATE_SIZE + LM_SIZE * landmark_idx
            
            # Extract 2x2 covariance matrix for this landmark
            cov_xx = self.PEst[lm_start_idx, lm_start_idx]
            cov_yy = self.PEst[lm_start_idx + 1, lm_start_idx + 1]
            cov_xy = self.PEst[lm_start_idx, lm_start_idx + 1]
            cov_yx = self.PEst[lm_start_idx + 1, lm_start_idx]  # Should equal cov_xy
            
            std_x = math.sqrt(cov_xx)
            std_y = math.sqrt(cov_yy)
            correlation = cov_xy / (std_x * std_y) if (std_x * std_y) > 0 else 0.0
            
            # Eigenvalue decomposition for uncertainty ellipse
            cov_matrix = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])
            eigenvals, eigenvecs = np.linalg.eigh(cov_matrix)
            semi_major_2sigma = 2.0 * math.sqrt(max(eigenvals))
            semi_minor_2sigma = 2.0 * math.sqrt(min(eigenvals))
            angle_rad = math.atan2(eigenvecs[1, 0], eigenvecs[0, 0])
            angle_deg = math.degrees(angle_rad)
            
            self.get_logger().info(f'  Tag {tag_id}:')
            self.get_logger().info(f'    Position: [{lm[0,0]:.6f}, {lm[1,0]:.6f}] m')
            self.get_logger().info(f'    Covariance Matrix:')
            self.get_logger().info(f'      [{cov_xx:.6f}, {cov_xy:.6f}]')
            self.get_logger().info(f'      [{cov_yx:.6f}, {cov_yy:.6f}]')
            self.get_logger().info(f'    Standard Deviations: σ_x={std_x:.6f} m, σ_y={std_y:.6f} m')
            self.get_logger().info(f'    Correlation Coefficient: {correlation:.6f}')
            self.get_logger().info(f'    2σ Uncertainty Ellipse (95% confidence):')
            self.get_logger().info(f'      Semi-major axis: {semi_major_2sigma:.6f} m')
            self.get_logger().info(f'      Semi-minor axis: {semi_minor_2sigma:.6f} m')
            self.get_logger().info(f'      Orientation: {angle_deg:.2f}°')
        
        self.get_logger().info('=' * 70)
    
    def save_slam_data(self):
        """Save SLAM data to pickle file for post-processing visualization"""
        try:
            # Log detailed covariance information before saving
            self.log_landmark_covariances()
            
            data = {
                'trajectory_1hz': self.trajectory_1hz,  # List of (timestamp, x, y, yaw)
                'waypoint_timestamps': self.waypoint_timestamps,  # List of (waypoint_idx, timestamp, x, y, yaw)
                'final_state_vector': self.xEst.copy(),  # Final state vector
                'final_covariance': self.PEst.copy(),  # Final covariance matrix
                'tag_id_to_landmark_index': self.tag_id_to_landmark_index.copy(),  # Tag ID mapping
                'robot_state_size': ROBOT_STATE_SIZE,
                'landmark_size': LM_SIZE
            }
            
            with open(self.data_file_path, 'wb') as f:
                pickle.dump(data, f)
            
            self.get_logger().info(f'[DATA SAVED] SLAM data saved to {self.data_file_path}')
            self.get_logger().info(f'  Trajectory samples (1Hz): {len(self.trajectory_1hz)}')
            self.get_logger().info(f'  Waypoints reached: {len(self.waypoint_timestamps)}')
            self.get_logger().info(f'  Landmarks: {len(self.tag_id_to_landmark_index)}')
        except Exception as e:
            self.get_logger().error(f'[DATA SAVE ERROR] Failed to save SLAM data: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = EKF_SLAM_Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('EKF SLAM Node stopped by keyboard interrupt')
    finally:
        # Save data before shutdown
        node.save_slam_data()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

