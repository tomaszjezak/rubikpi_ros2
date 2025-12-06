#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from vroomba_coordinator.srv import GetNextWaypoint
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import os
import yaml
import time
import math
from ament_index_python.packages import get_package_share_directory

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
    def __init__(self):
        super().__init__('hw5_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.tag_marker_pub = self.create_publisher(MarkerArray, '/apriltag_locations', 10)

        # Clear all previous markers on startup
        self.clear_all_markers()

        self.tf_buffer = Buffer(cache_time=Duration(seconds=0.25))  # 0.25s cache - AprilTag frames disappear quickly
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        self.tag_positions = {}

        # Robot's actual starting position (new coordinate system)
        robot_x_init = 1.9685
        robot_y_init = 0.7620

        # Hardcoded waypoint pattern from hw3 (defined in old coordinate system where robot starts at 0,0)
        # Square drive pattern waypoints (starts by turning left 90 degrees)
        waypoints_single_square_relative = np.array([
            [ 0.000000,  0.850000,  0.0],  # Drive to north 0.85m, face east to read tag
            [-0.850000,  0.850000, 1.5708],  # Drive west 0.85m, turn north to read tag
            [-0.850000,  0.000000,  3.14159],  # Drive south 0.85m, turn west to read tag
            [ 0.000000,  0.000000,  -1.5708],  # Drive east 0.85m, turn south to read tag
        ])

        # Offset waypoints to robot's actual starting position
        waypoints_single_square = waypoints_single_square_relative.copy()
        waypoints_single_square[:, 0] += robot_x_init  # Offset x
        waypoints_single_square[:, 1] += robot_y_init  # Offset y

        # Declare parameters
        self.declare_parameter('enable_movement', True)  # Can be overridden via launch file parameter

        # PID controller used to smooth out the forward drive acceleration
        # The parameters are Kp, Ki, Kd
        # It does not affect the rotational movement, even though the
        # output contains angular velocity as well.
        self.pid = PIDcontroller(0.8, 0.01, 0.005)

        self.current_state = np.array([1.9685, 0.7620, np.pi/2])  # [x, y, yaw] - starts facing upwards

        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.tolerance = 0.2 # [m] - waypoint reaching tolerance

        self.waypoints = waypoints_single_square
        self.get_logger().info(f'Using square path with {len(self.waypoints)} waypoints')

        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        # Load ground truth landmarks and publish visualization
        self.load_ground_truth_landmarks()
        self.create_timer(1.0, self.publish_tag_markers) 
        
        self.stage = 'rotate_to_face_selected_waypoint'
        self.prev_stage = self.stage
        self.fixed_rotation_vel = 0.785

        self.do_nothing_ticks = 0

        # Vroomba service client for waypoint generation
        self.vroomba_client = self.create_client(GetNextWaypoint, 'get_next_waypoint')
        self.waiting_for_waypoint = False
        self.get_logger().info('Waiting for vroomba waypoint service...')
        # Don't block here - service will be checked when needed

    def clear_all_markers(self):
        """
        Clear all existing waypoint markers in rviz2 on node startup
        """
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL

        marker_array = MarkerArray()
        marker_array.markers.append(delete_marker)

        # Clear waypoint markers only
        self.waypoint_marker_pub.publish(marker_array)

        self.get_logger().info('Cleared all previous waypoint markers from rviz2')

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

    def get_desired_heading_to_goal(self, current_wp):
        """
        Get the desired heading to face towards (or away from) the goal
        """
        delta_x = current_wp[0] - self.current_state[0]
        delta_y = current_wp[1] - self.current_state[1]
        angle_to_target = np.arctan2(delta_y, delta_x)

        desired_heading = angle_to_target

        return desired_heading

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
    
    def request_new_waypoint(self):
        """
        Request a new waypoint from vroomba coordinator service
        """
        # Check if service is available
        if not self.vroomba_client.service_is_ready():
            self.get_logger().warn('Vroomba service not available yet')
            return
        
        # Create request with current robot pose
        request = GetNextWaypoint.Request()
        request.current_x = float(self.current_state[0])
        request.current_y = float(self.current_state[1])
        request.current_yaw = float(self.current_state[2])
        request.num_waypoints = 1  # Request 1 waypoint at a time
        
        self.get_logger().info(
            f'Calling vroomba service at position ({request.current_x:.3f}, {request.current_y:.3f}, '
            f'{np.degrees(request.current_yaw):.1f}°)'
        )
        
        # Call service asynchronously
        future = self.vroomba_client.call_async(request)
        future.add_done_callback(self.waypoint_response_callback)
    
    def waypoint_response_callback(self, future):
        """
        Handle response from vroomba waypoint service
        """
        try:
            response = future.result()
            
            if response.success and len(response.waypoint_x) > 0:
                # Got waypoint(s) from vroomba
                new_waypoints = []
                for i in range(len(response.waypoint_x)):
                    x = response.waypoint_x[i]
                    y = response.waypoint_y[i]
                    # Calculate heading ourselves (vroomba doesn't provide useful theta)
                    theta = math.atan2(y - self.current_state[1], x - self.current_state[0])
                    new_waypoints.append([x, y, theta])
                    self.get_logger().info(
                        f'Received waypoint from vroomba: ({x:.3f}, {y:.3f}, {np.degrees(theta):.1f}°)'
                    )
                
                # Add new waypoints to the list
                if len(self.waypoints) == 0:
                    self.waypoints = np.array(new_waypoints)
                else:
                    self.waypoints = np.vstack((self.waypoints, new_waypoints))
                
                self.waiting_for_waypoint = False
                self.get_logger().info(f'Added {len(new_waypoints)} waypoint(s), total now: {len(self.waypoints)}')
                
            else:
                self.get_logger().error('Vroomba service failed to generate waypoint')
                self.waiting_for_waypoint = False
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.waiting_for_waypoint = False

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
            self.do_nothing_ticks = 0 # 2
            self.prev_stage = self.stage
        if self.do_nothing_ticks > 0:
            self.do_nothing_ticks -= 1
            self.stop_robot()
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            # Waypoints exhausted - request new waypoint from vroomba
            if not self.waiting_for_waypoint:
                self.get_logger().info('Waypoints exhausted - requesting new waypoint from vroomba...')
                self.request_new_waypoint()
                self.waiting_for_waypoint = True
            # Stop and wait for new waypoint
            self.stop_robot()
            return


        current_wp = self.waypoints[self.current_waypoint_idx]

        # Track previous stage to only log on changes
        prev_stage = getattr(self, '_prev_stage', None)

        if not self.waypoint_reached:
            self.pid.setTarget(current_wp)
            self.waypoint_reached = True
            self.stage = 'rotate_to_face_selected_waypoint'
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

    def load_ground_truth_landmarks(self):
        """Load ground truth landmark positions from YAML config file"""
        try:
            hw5_pkg_path = get_package_share_directory('hw_5')
            landmark_file = os.path.join(hw5_pkg_path, 'configs', 'ground_truth_landmarks.yaml')

            with open(landmark_file, 'r') as f:
                config = yaml.safe_load(f)

            # Store landmarks in tag_positions dict
            for landmark in config['landmarks']:
                tag_id = landmark['tag_id']
                self.tag_positions[tag_id] = {
                    'x': landmark['x'],
                    'y': landmark['y'],
                    'z': 0.0  # Ground level
                }

            self.get_logger().info(f'Loaded {len(self.tag_positions)} ground truth landmarks for visualization')
        except Exception as e:
            self.get_logger().error(f'Failed to load ground truth landmarks: {e}')

    def publish_tag_markers(self):
        """Publish ground truth AprilTag locations as small sphere markers in RViz"""
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()

        for tag_id, tag_data in self.tag_positions.items():
            # Small sphere marker for tag position
            sphere = Marker()
            sphere.header.frame_id = 'odom'
            sphere.header.stamp = current_time
            sphere.ns = 'apriltag_positions'
            sphere.id = tag_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = tag_data['x']
            sphere.pose.position.y = tag_data['y']
            sphere.pose.position.z = 0.08  # Slightly above ground
            sphere.pose.orientation.w = 1.0

            # Small sphere size
            sphere.scale.x = 0.1
            sphere.scale.y = 0.1
            sphere.scale.z = 0.1

            # Bright orange color
            sphere.color.r = 1.0
            sphere.color.g = 0.5
            sphere.color.b = 0.0
            sphere.color.a = 0.9

            marker_array.markers.append(sphere)

            # Text label showing tag ID
            text = Marker()
            text.header.frame_id = 'odom'
            text.header.stamp = current_time
            text.ns = 'apriltag_labels'
            text.id = tag_id + 1000  # Offset to avoid ID collision
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = tag_data['x']
            text.pose.position.y = tag_data['y']
            text.pose.position.z = 0.25  # Above the sphere
            text.pose.orientation.w = 1.0

            text.scale.z = 0.08  # Text height

            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0

            text.text = f"Tag {tag_id}"

            marker_array.markers.append(text)

        self.tag_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Hw5Node()

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
