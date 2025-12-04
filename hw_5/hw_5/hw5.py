#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from prm_planner.srv import GetPRMPath
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Quaternion
import numpy as np
import os
import yaml
import time
import math
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

"""
The class of the pid controller for differential drive robot.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0, 0.0])
        self.lastError = np.array([0.0, 0.0])
        self.timestep = 0.1
        self.maximumValue = 0.2

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0, 0.0]) 
        self.lastError = np.array([0.0, 0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the error between current and target state
        for differential drive: distance error and heading error
        """
        delta_x = targetState[0] - currentState[0]
        delta_y = targetState[1] - currentState[1]
        
        distance = np.sqrt(delta_x**2 + delta_y**2)
        
        angle_to_target = np.arctan2(delta_y, delta_x)
        
        desired_heading = angle_to_target
        
        heading_error = desired_heading - currentState[2]
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        
        if abs(distance) < 0.05:
            heading_error = targetState[2] - currentState[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            distance = 0.0
        
        return np.array([distance, heading_error])

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value based on PID control
        Returns: [linear_velocity, angular_velocity]
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        if abs(result[0]) > self.maximumValue:
            result[0] = np.sign(result[0]) * self.maximumValue
            
        max_angular = 1.5 # rad/s
        if abs(result[1]) > max_angular:
            result[1] = np.sign(result[1]) * max_angular
        
        if abs(e[0]) < 0.05:
            result[0] = 0.0
        return result

class Hw5Node(Node):
    def __init__(self, planning_mode='distance'):
        super().__init__('hw5_node')
        self.planning_mode = planning_mode
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        self.tag_positions = {}

        # Robot's actual starting position (new coordinate system)
        robot_x_init = 1.9685
        robot_y_init = 0.7620

        # Hardcoded waypoint patterns from hw3 (defined in old coordinate system where robot starts at 0,0)
        # Square drive pattern waypoints (starts by turning left 90 degrees)
        waypoints_single_square_relative = np.array([
            [ 0.000000,  0.850000,  0.0],  # Drive to north 0.85m, face east to read tag
            [-0.850000,  0.850000, 1.5708],  # Drive west 0.85m, turn north to read tag
            [-0.850000,  0.000000,  3.14159],  # Drive south 0.85m, turn west to read tag
            [ 0.000000,  0.000000,  -1.5708],  # Drive east 0.85m, turn south to read tag
        ])
        # Octagonal drive pattern (defined relative to 0,0)
        waypoints_octagon_relative = np.array([
            # 0. Drive forward by 0.425. Face up 45 degrees to read tag
            [ 0.425000, 0.000000, 0.7854],
            # Now we are on the octagon edge
            #. 1. Go up by 0.85m. face upwards cuz whatever
            [ 0.425000, 0.850000, 1.5708],
            # 2. Go diagonal up-left by sqrt(2)*0.425
            [ 0.000000, 1.275000, 2.3562],
            # 3. Go left by 0.85m
            [-0.850000, 1.275000, 3.14159],
            # 4. Go diagonal down-left by sqrt(2)*0.425
            [-1.275000, 0.850000, -2.3562],
            # 5. Go down by 0.85m
            [-1.275000, 0.000000, -1.5708],
            # 6. Go diagonal down-right by sqrt(2)*0.425
            [-0.850000, -0.425000, -0.7854],
            # 7. Go right by 0.85m
            [ 0.000000, -0.425000, 0.0],
            # 8. Go diagonal up-right by sqrt(2)*0.425
            [ 0.425000, 0.000000, 0.7854],
        ])

        # Offset waypoints to robot's actual starting position
        waypoints_single_square = waypoints_single_square_relative.copy()
        waypoints_single_square[:, 0] += robot_x_init  # Offset x
        waypoints_single_square[:, 1] += robot_y_init  # Offset y

        waypoints_octagon = waypoints_octagon_relative.copy()
        waypoints_octagon[:, 0] += robot_x_init  # Offset x
        waypoints_octagon[:, 1] += robot_y_init  # Offset y

        # Double patterns
        waypoints_double_square = np.vstack((waypoints_single_square, waypoints_single_square))
        waypoints_double_octagon = np.vstack((waypoints_octagon, waypoints_octagon))
        # 0,0,0 waypoint for testing
        waypoints_zero = np.array([[0.0, 0.0, 0.0]])

        # Declare parameters for path planning
        self.declare_parameter('start_x', 1.9685)  # Robot at old (0,0) = new (1.9685, 0.762)
        self.declare_parameter('start_y', 0.7620)
        self.declare_parameter('goal_x', 0.31)   # 22" to the right of Tag 4
        self.declare_parameter('goal_y', 2.117725)   # +0.5m in y direction from previous
        self.declare_parameter('use_prm_planner', False)
        self.declare_parameter('enable_movement', True)  # Can be overridden via launch file parameter

        # Waypoints list - populated from PRM planner service
        # Format: numpy array of [x, y, theta] where x,y are positions and theta is orientation
        self.waypoints = np.array([[0.0, 0.0, 0.0]])  # Default fallback

        # PID controller used to smooth out the forward drive acceleration
        # The parameters are Kp, Ki, Kd
        # It does not affect the rotational movement, even though the
        # output contains angular velocity as well.
        self.pid = PIDcontroller(0.8, 0.01, 0.005)

        self.current_state = np.array([1.9685, 0.7620, np.pi/2])  # [x, y, yaw] - starts facing upwards
        # This variable is updated from EKF SLAM pose via TF (odom -> base_link)
        self.obs_current_state = np.array([1.9685, 0.7620, np.pi/2])
        
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.tolerance = 0.30 # [m] - increased for more lenient waypoint reaching
        self.angle_tolerance = 0.35 # [rad] ≈20° - more chill about orientation
        # self.backward_threshold = np.deg2rad(100)  # Skip waypoints more than 100° behind - DISABLED

        self.last_tag_detection_time = 0.0
        self.using_tag_localization = False
        self.tag_initialized = False

        # Request path from PRM planner service if enabled
        if self.get_parameter('use_prm_planner').get_parameter_value().bool_value:
            self._request_prm_path()
        else:
            # Use octagonal drive pattern when PRM is disabled
            self.waypoints = waypoints_octagon
            self.get_logger().info(f'Using octagonal path with {len(self.waypoints)} waypoints')

        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.localization_timer = self.create_timer(self.dt, self.localization_update) 
        
        self.stage = 'rotate_to_face_selected_waypoint'
        self.prev_stage = self.stage
        self.stage_pid = PIDcontroller(0.8, 0.01, 0.005)
        self.fixed_rotation_vel = 0.700  # 0.785

        self.do_nothing_ticks = 0

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw

    def _request_prm_path(self):
        """
        Request a path from the PRM planner service and populate waypoints list.
        This is called once during initialization.
        """
        # Get start and goal from parameters
        start_x = self.get_parameter('start_x').get_parameter_value().double_value
        start_y = self.get_parameter('start_y').get_parameter_value().double_value
        goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        goal_y = self.get_parameter('goal_y').get_parameter_value().double_value

        self.get_logger().info(
            f'Requesting PRM path from ({start_x:.3f}, {start_y:.3f}) '
            f'to ({goal_x:.3f}, {goal_y:.3f})'
        )

        # Create service client
        prm_client = self.create_client(GetPRMPath, 'get_prm_path')

        # Wait for service to be available (10 second timeout)
        if not prm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'PRM planner service not available! Using fallback waypoint [0,0,0]'
            )
            self.waypoints = np.array([[0.0, 0.0, 0.0]])
            return

        # Create request
        request = GetPRMPath.Request()
        request.start_x = start_x
        request.start_y = start_y
        request.goal_x = goal_x
        request.goal_y = goal_y
        request.planning_mode = self.planning_mode

        # Call service synchronously
        future = prm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                # Log received path plan with first few elements
                path_len = len(response.path_x)
                first_few = min(3, path_len)
                path_preview = ', '.join([f'({response.path_x[i]:.3f}, {response.path_y[i]:.3f})' for i in range(first_few)])
                self.get_logger().info(
                    f'Received path plan: {path_len} points. First {first_few}: [{path_preview}]'
                )
                # Plot the path plan
                self._plot_path_plan(response.path_x, response.path_y, start_x, start_y, goal_x, goal_y, self.planning_mode)
                # Convert path to waypoints with orientations
                self._convert_path_to_waypoints(response.path_x, response.path_y)
            else:
                self.get_logger().error(
                    f'PRM path planning failed: {response.message}'
                )
                self.waypoints = np.array([[0.0, 0.0, 0.0]])
        else:
            self.get_logger().error('PRM service call failed or timed out')
            self.waypoints = np.array([[0.0, 0.0, 0.0]])

    def _plot_path_plan(self, path_x, path_y, start_x, start_y, goal_x, goal_y, planning_mode='distance'):
        """
        Plot the planned path with workspace and obstacle bounds for visualization.
        
        Args:
            path_x: List of x coordinates
            path_y: List of y coordinates
            start_x: Start x coordinate
            start_y: Start y coordinate
            goal_x: Goal x coordinate
            goal_y: Goal y coordinate
            planning_mode: Planning mode used ("safe" or "distance")
        """
        try:
            fig, ax = plt.subplots(figsize=(10, 10))
            
            # Workspace bounds (from PRM planner defaults)
            workspace_bounds = [0.0, 2.8067, 0.0, 2.6544]
            obstacle_bounds = [1.336675, 1.654175, 1.003300, 1.416000]
            
            # Draw workspace
            ws_min_x, ws_max_x, ws_min_y, ws_max_y = workspace_bounds
            ax.add_patch(plt.Rectangle((ws_min_x, ws_min_y), 
                                      ws_max_x - ws_min_x, ws_max_y - ws_min_y,
                                      fill=False, edgecolor='gray', linestyle='--', linewidth=1, label='Workspace'))
            
            # Draw obstacle box
            obs_min_x, obs_max_x, obs_min_y, obs_max_y = obstacle_bounds
            ax.add_patch(plt.Rectangle((obs_min_x, obs_min_y),
                                      obs_max_x - obs_min_x, obs_max_y - obs_min_y,
                                      fill=True, facecolor='red', alpha=0.3, edgecolor='red', linewidth=2, label='Obstacle'))
            
            # Plot path
            if len(path_x) > 0:
                ax.plot(path_x, path_y, 'b-o', linewidth=2, markersize=4, label='Planned Path', zorder=3)
                # Mark start and goal
                ax.plot(path_x[0], path_y[0], 'go', markersize=10, label='Start', zorder=4)
                ax.plot(path_x[-1], path_y[-1], 'ro', markersize=10, label='Goal', zorder=4)
            
            # Also plot requested start/goal (in case they differ from path endpoints)
            ax.plot(start_x, start_y, 'g*', markersize=15, label='Requested Start', zorder=5)
            ax.plot(goal_x, goal_y, 'r*', markersize=15, label='Requested Goal', zorder=5)
            
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title(f'PRM Path Plan - {planning_mode.upper()} Mode ({len(path_x)} waypoints)')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
            ax.set_xlim(ws_min_x - 0.1, ws_max_x + 0.1)
            ax.set_ylim(ws_min_y - 0.1, ws_max_y + 0.1)
            
            plt.tight_layout()
            plt.savefig('/tmp/prm_path_plan.png', dpi=150, bbox_inches='tight')
            self.get_logger().info('Path plan saved to /tmp/prm_path_plan.png')
            plt.close(fig)
        except Exception as e:
            self.get_logger().warn(f'Failed to plot path: {e}')
    
    def _convert_path_to_waypoints(self, path_x, path_y):
        """
        Convert PRM path (x, y coordinates) to waypoints with orientations.
        Each waypoint faces the direction of travel to the next point.

        Args:
            path_x: List of x coordinates
            path_y: List of y coordinates
        """
        if len(path_x) == 0 or len(path_y) == 0:
            self.waypoints = np.array([[0.0, 0.0, 0.0]])
            self.current_waypoint_idx = 0  # Reset index
            self.waypoint_reached = False  # Reset waypoint reached flag
            return

        waypoints_list = []

        for i in range(len(path_x)):
            x = path_x[i]
            y = path_y[i]

            # Calculate orientation to face next waypoint
            if i < len(path_x) - 1:
                # Not the last point - face toward next point
                delta_x = path_x[i + 1] - x
                delta_y = path_y[i + 1] - y
                theta = np.arctan2(delta_y, delta_x)
            else:
                # Last point - keep the same orientation as the previous waypoint
                if len(waypoints_list) > 0:
                    theta = waypoints_list[-1][2]
                else:
                    theta = 0.0

            waypoints_list.append([x, y, theta])

        self.waypoints = np.array(waypoints_list)
        self.current_waypoint_idx = 0  # Reset to start of new path
        self.waypoint_reached = False  # Reset waypoint reached flag
        self.get_logger().info(f'Converted path to {len(self.waypoints)} waypoints')

    def get_desired_heading_to_goal(self, current_wp):
        """
        Get the desired heading to face towards (or away from) the goal
        """
        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        angle_to_target = np.arctan2(delta_y, delta_x)

        desired_heading = angle_to_target

        return desired_heading

    def is_waypoint_behind(self, waypoint_idx):
        """
        Check if a waypoint is significantly behind the robot's current heading.
        Returns True if the waypoint is more than backward_threshold degrees behind.

        Args:
            waypoint_idx: Index of waypoint to check

        Returns:
            bool: True if waypoint is behind robot, False otherwise
        """
        if waypoint_idx >= len(self.waypoints):
            return False

        waypoint = self.waypoints[waypoint_idx]

        # Calculate angle to waypoint
        delta_x = waypoint[0] - self.current_state[0]
        delta_y = waypoint[1] - self.current_state[1]
        angle_to_waypoint = np.arctan2(delta_y, delta_x)

        # Calculate relative angle (waypoint angle - robot heading)
        relative_angle = angle_to_waypoint - self.current_state[2]
        # Normalize to [-pi, pi]
        relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi

        # Check if waypoint is behind (absolute angle > threshold)
        return abs(relative_angle) > self.backward_threshold

    def get_rotation_direction(self, heading_error):
        """
        Determine rotation direction based on heading error.
        Returns: angular velocity with fixed magnitude but correct direction
        """
        if heading_error > 0:
            return self.fixed_rotation_vel
        else:
            return -self.fixed_rotation_vel
    
    def stop_robot(self):
        """
        Stop the robot by publishing zero velocities
        """
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)

    def control_loop(self):
        """
        Main control loop with three stages: rotate to goal, drive, rotate to orientation
        """
        # Check if movement is disabled
        if not self.get_parameter('enable_movement').get_parameter_value().bool_value:
            self.stop_robot()
            return

        # EKF SLAM node is authoritative for odom -> base_link TF
        # Wait for transform to be available (EKF SLAM must be running)
        if not self.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
            # Transform not available yet - skip this iteration
            return

        # Read robot pose from EKF TF
        transform = self.tf_buffer.lookup_transform(
            'odom', 'base_link', rclpy.time.Time()
        )
        # Update current_state from EKF estimate
        self.current_state[0] = transform.transform.translation.x
        self.current_state[1] = transform.transform.translation.y
        
        # Extract yaw from quaternion
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w
        
        # Convert quaternion to yaw (ZYX Euler)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_state[2] = math.atan2(siny_cosp, cosy_cosp)
        
        # Dead check:
        if self.prev_stage != self.stage:
            self.do_nothing_ticks = 5
            self.prev_stage = self.stage
        if self.do_nothing_ticks > 0:
            self.do_nothing_ticks -= 1
            self.stop_robot()
            return
        
        # Skip waypoints that are behind the robot - DISABLED
        # while self.current_waypoint_idx < len(self.waypoints):
        #     if self.is_waypoint_behind(self.current_waypoint_idx):
        #         self.get_logger().info(
        #             f'Skipping waypoint {self.current_waypoint_idx} '
        #             f'(more than {np.degrees(self.backward_threshold):.0f}° behind robot)'
        #         )
        #         self.current_waypoint_idx += 1
        #         self.waypoint_reached = False
        #     else:
        #         break

        if self.current_waypoint_idx >= len(self.waypoints):
            # Only log once when all waypoints are reached
            if not hasattr(self, '_waypoints_complete_logged'):
                self.get_logger().info('All waypoints reached! Stopping robot.')
                self._waypoints_complete_logged = True
            # self.stop_robot()
            return


        current_wp = self.waypoints[self.current_waypoint_idx]

        # Track previous stage to only log on changes
        prev_stage = getattr(self, '_prev_stage', None)

        if not self.waypoint_reached:
            self.pid.setTarget(current_wp)
            self.waypoint_reached = True
            self.stage = 'rotate_to_face_selected_waypoint'
            self.stage_pid.setTarget(current_wp)
            # Log target waypoint when starting new waypoint
            self.get_logger().info(
                f'Targeting waypoint {self.current_waypoint_idx}: '
                f'pos=({current_wp[0]:.3f}, {current_wp[1]:.3f}), yaw={np.degrees(current_wp[2]):.1f}°'
            )

        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        position_error = np.sqrt(delta_x**2 + delta_y**2)
        twist_msg = Twist()
        
        # Stage 1: Rotate to face the selected waypoint
        if self.stage == 'rotate_to_face_selected_waypoint':
            desired_heading = self.get_desired_heading_to_goal(current_wp)
            heading_error = desired_heading - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            
            if abs(heading_error) < 0.05:
                # Switch to DRIVE
                if prev_stage != 'drive':
                    # self.get_logger().info(f'Stage: {self.stage} -> drive')
                    pass
                self.stage = 'drive'
                twist_msg.angular.z = 0.0
            else:
                if prev_stage != self.stage:
                    self.get_logger().info(f'Stage: {self.stage}')
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
        # Stage 2: Drive towards the selected waypoint
        elif self.stage == 'drive':
            position_error = np.sqrt(delta_x**2 + delta_y**2)
            if position_error < self.tolerance:
                # We are close enough to the waypoint, mark it as reached
                self.get_logger().info(f'Waypoint {self.current_waypoint_idx} reached!')
                self.current_waypoint_idx += 1
                self.waypoint_reached = False
                twist_msg.linear.x = 0.0
            else:
                # Still far away, keep driving
                desired_heading = self.get_desired_heading_to_goal(current_wp)
                heading_error = desired_heading - self.current_state[2]
                heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
                if abs(heading_error) > 0.2:
                    # We are pointing too far off course, need to rotate first
                    if prev_stage != 'rotate_to_face_selected_waypoint':
                        # self.get_logger().info(f'Stage: {self.stage} -> rotate_to_face_selected_waypoint')
                        pass
                    self.stage = 'rotate_to_face_selected_waypoint'
                    twist_msg.linear.x = 0.0
                else:
                    update_value = self.pid.update(self.current_state)
                    twist_msg.linear.x = float(update_value[0])

        # Store current stage for next iteration
        self._prev_stage = self.stage

        # EKF SLAM node handles pose estimation via TF
        self.cmd_vel_pub.publish(twist_msg)

        # Publish waypoint visualization markers
        self.publish_waypoint_visualization()
        
        
    def localization_update(self):
        """Localization update - reads pose from EKF SLAM via TF
        
        This function is called every 0.1s by a timer in the constructor.
        The EKF SLAM node handles pose estimation and broadcasts it via TF.
        """
        current_time = time.time()
        tag_detected = False
        closest_tag_id = None
        closest_observation = None
        closest_distance = float('inf')
        for tag_id in self.tag_positions.keys():
            try:
                tag_frame = f'tag_{tag_id}'
                observation = self.tf_buffer.lookup_transform('base_link', tag_frame, rclpy.time.Time())
                transform_time = rclpy.time.Time.from_msg(observation.header.stamp)
                time_diff = (self.get_clock().now() - transform_time).nanoseconds / 1e9
                if time_diff > 0.25:
                    continue
                dx = observation.transform.translation.x
                dy = observation.transform.translation.y
                dz = observation.transform.translation.z
                distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_tag_id = tag_id
                    closest_observation = observation
            except Exception:
                continue
        
        if closest_observation is not None:
            self.compute_and_publish_robot_pose_from_tag(closest_tag_id, closest_observation)
            tag_detected = True
            self.last_tag_detection_time = current_time
            self.using_tag_localization = True
        else:
            tag_detected = False
            time_since_last_tag = current_time - self.last_tag_detection_time
            if time_since_last_tag > 1.0:  # 1 second timeout
                self.using_tag_localization = False
                
    def compute_and_publish_robot_pose_from_tag(self, tag_id, tag_observation):
        """Compute robot pose from tag observation and publish it
        This is a helper function for the updating localization function.
        
        TODO This code assumes the tag locations are known.
        """
        tag_map = self.tag_positions[tag_id]
        
        tag_map_pos = np.array([tag_map['x'], tag_map['y'], tag_map['z']])
        tag_map_rot = Rotation.from_quat([tag_map['qx'], tag_map['qy'], tag_map['qz'], tag_map['qw']])
        
        obs_pos = np.array([
            tag_observation.transform.translation.x,
            tag_observation.transform.translation.y,
            tag_observation.transform.translation.z
        ])
        obs_rot = Rotation.from_quat([
            tag_observation.transform.rotation.x,
            tag_observation.transform.rotation.y,
            tag_observation.transform.rotation.z,
            tag_observation.transform.rotation.w
        ])
        
        tag_to_robot_rot = obs_rot.inv()
        tag_to_robot_pos = -tag_to_robot_rot.apply(obs_pos)
        
        robot_map_rot = tag_map_rot * tag_to_robot_rot
        robot_map_pos = tag_map_pos + tag_map_rot.apply(tag_to_robot_pos)
        
        yaw = robot_map_rot.as_euler('xyz')[2]
        self.current_state = np.array([robot_map_pos[0], robot_map_pos[1], yaw])
        self.obs_current_state = np.array([robot_map_pos[0], robot_map_pos[1], yaw])
        
        self.get_logger().info(
            f'Updated pose from tag {tag_id}: '
            f'pos=({robot_map_pos[0]:.3f}, {robot_map_pos[1]:.3f}), yaw={yaw:.3f}'
        )

    def publish_waypoint_visualization(self):
        """
        Publish visualization markers for waypoints in rviz2.
        Shows all waypoints with different colors:
        - Completed waypoints: Green spheres
        - Current target waypoint: Large cyan arrow showing orientation
        - Future waypoints: Gray spheres
        """
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()

        # Marker for all waypoint positions as spheres
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = current_time
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = float(waypoint[0])
            marker.pose.position.y = float(waypoint[1])
            marker.pose.position.z = 0.05  # Slightly above ground

            # Orientation (identity quaternion)
            marker.pose.orientation.w = 1.0

            # Size - smaller for waypoints
            if i == self.current_waypoint_idx:
                # Current waypoint - skip, we'll add special arrow marker
                continue
            else:
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08

            # Color based on status
            if i < self.current_waypoint_idx:
                # Completed waypoints - green
                marker.color.r = 0.0
                marker.color.g = 0.8
                marker.color.b = 0.0
                marker.color.a = 0.6
            else:
                # Future waypoints - light gray
                marker.color.r = 0.6
                marker.color.g = 0.6
                marker.color.b = 0.6
                marker.color.a = 0.4

            marker_array.markers.append(marker)

        # Special marker for current target waypoint - large arrow showing orientation
        if self.current_waypoint_idx < len(self.waypoints):
            current_wp = self.waypoints[self.current_waypoint_idx]

            # Arrow marker for current waypoint
            arrow_marker = Marker()
            arrow_marker.header.frame_id = 'odom'
            arrow_marker.header.stamp = current_time
            arrow_marker.ns = 'current_waypoint'
            arrow_marker.id = 0
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD

            # Position
            arrow_marker.pose.position.x = float(current_wp[0])
            arrow_marker.pose.position.y = float(current_wp[1])
            arrow_marker.pose.position.z = 0.1

            # Orientation from waypoint's theta (yaw)
            qx, qy, qz, qw = self.euler_to_quaternion(0, 0, float(current_wp[2]))
            arrow_marker.pose.orientation.x = qx
            arrow_marker.pose.orientation.y = qy
            arrow_marker.pose.orientation.z = qz
            arrow_marker.pose.orientation.w = qw

            # Arrow size
            arrow_marker.scale.x = 0.20  # Length
            arrow_marker.scale.y = 0.03  # Width
            arrow_marker.scale.z = 0.03  # Height

            # Bright cyan color
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.9
            arrow_marker.color.b = 0.9
            arrow_marker.color.a = 1.0

            marker_array.markers.append(arrow_marker)

            # Add a sphere at current waypoint too for emphasis
            sphere_marker = Marker()
            sphere_marker.header.frame_id = 'odom'
            sphere_marker.header.stamp = current_time
            sphere_marker.ns = 'current_waypoint_sphere'
            sphere_marker.id = 0
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD

            sphere_marker.pose.position.x = float(current_wp[0])
            sphere_marker.pose.position.y = float(current_wp[1])
            sphere_marker.pose.position.z = 0.05
            sphere_marker.pose.orientation.w = 1.0

            sphere_marker.scale.x = 0.12
            sphere_marker.scale.y = 0.12
            sphere_marker.scale.z = 0.12

            # Yellow color for current waypoint sphere
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.9
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 0.8

            marker_array.markers.append(sphere_marker)

            # Add text label showing waypoint number
            text_marker = Marker()
            text_marker.header.frame_id = 'odom'
            text_marker.header.stamp = current_time
            text_marker.ns = 'waypoint_labels'
            text_marker.id = 0
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = float(current_wp[0])
            text_marker.pose.position.y = float(current_wp[1])
            text_marker.pose.position.z = 0.25  # Above the waypoint
            text_marker.pose.orientation.w = 1.0

            text_marker.scale.z = 0.08  # Text height

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"WP {self.current_waypoint_idx}/{len(self.waypoints)-1}"

            marker_array.markers.append(text_marker)

        # Publish the marker array
        self.waypoint_marker_pub.publish(marker_array)

def main(args=None):
    # Parse command-line arguments for planning mode
    import sys
    planning_mode = 'distance'  # default
    
    # Check for --safe or --planning-mode flags
    if '--safe' in sys.argv:
        planning_mode = 'safe'
    elif '--planning-mode' in sys.argv:
        try:
            idx = sys.argv.index('--planning-mode')
            if idx + 1 < len(sys.argv):
                mode = sys.argv[idx + 1].lower()
                if mode in ['distance', 'safe']:
                    planning_mode = mode
        except (ValueError, IndexError):
            pass
    
    rclpy.init(args=args)
    node = Hw5Node(planning_mode=planning_mode)
    node.get_logger().info(f'Starting with planning mode: {planning_mode}')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by keyboard interrupt')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
