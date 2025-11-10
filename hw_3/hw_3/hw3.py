#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import numpy as np
import os 
import yaml
import time
import math
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

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

class Hw3Node(Node):
    def __init__(self):
        super().__init__('hw3_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        
        self.tag_positions = {}
        

        # Octagonal drive pattern waypoints
        edge = 0.8 # diameter of 80%
        s = (1.0 - 1.0/np.sqrt(2.0)) * 0.8  # ≈ 0.234314574
        self.waypoints = np.array([
            [0.0,     0.0,    0.0],
            [edge,    0.0,    0.0],

            [edge,    s,      np.pi/2],
            [s,       edge,   3*np.pi/4],
            [-s,      edge,   np.pi],
            [-edge,   s,     -3*np.pi/4],
            [-edge,  -s,     -np.pi/2],
            [-s,     -edge,  -np.pi/4],
            [s,      -edge,   0.0],
            [edge,   -s,      np.pi/4],

            [edge,    0.0,    np.pi/2],
            [0.0,     0.0,    np.pi],
        ])
        # 0,0,0 waypoint for testing
        self.waypoints = np.array([[0.0, 0.0, 0.0]])

        # PID controller used to smooth out the forward drive acceleration
        # The parameters are Kp, Ki, Kd
        # It does not affect the rotational movement, even though the
        # output contains angular velocity as well.
        self.pid = PIDcontroller(0.8, 0.01, 0.005)
        
        self.current_state = np.array([0.0, 0.0, 0.0])
        # This variable is updated by the localization module
        # It currently uses AprilTag observations when available,
        # otherwise it falls back to dead reckoning.
        self.obs_current_state = np.array([0.0, 0.0, 0.0])
        
        self.current_waypoint_idx = 0
        self.waypoint_reached = False
        self.tolerance = 0.15
        self.angle_tolerance = 0.1
        
        self.last_tag_detection_time = 0.0
        self.using_tag_localization = False
        self.tag_initialized = False
                
        self.dt = 0.1
        self.control_timer = self.create_timer(self.dt, self.control_loop)
        self.localization_timer = self.create_timer(self.dt, self.localization_update) 
        
        self.stage = 'rotate_to_face_selected_waypoint'
        self.stage_pid = PIDcontroller(0.8, 0.01, 0.005)
        self.fixed_rotation_vel = 0.785
        
    def update_dead_reckoning(self, linear_vel, angular_vel):
        """
        Update robot pose using dead reckoning
        """
        self.current_state[0] += linear_vel * np.cos(self.current_state[2]) * self.dt
        self.current_state[1] += linear_vel * np.sin(self.current_state[2]) * self.dt
        self.current_state[2] += angular_vel * self.dt
        self.current_state[2] = (self.current_state[2] + np.pi) % (2 * np.pi) - np.pi
        
    def broadcast_base_link_tf(self):
        """
        Broadcast TF transform from odom to base_link
        """
        current_time = self.get_clock().now()
        
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.current_state[0]
        t.transform.translation.y = self.current_state[1]
        t.transform.translation.z = 0.0
        
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.current_state[2])
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
        
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

    def control_loop(self):
        """
        Main control loop with three stages: rotate to goal, drive, rotate to orientation
        """
        self.broadcast_base_link_tf()
        
        if self.current_waypoint_idx >= len(self.waypoints):
            # Only log once when all waypoints are reached
            if not hasattr(self, '_waypoints_complete_logged'):
                self.get_logger().info('All waypoints reached! Stopping robot.')
                self._waypoints_complete_logged = True
            self.stop_robot()
            return

        current_wp = self.waypoints[self.current_waypoint_idx]
        
        if not self.waypoint_reached:
            self.pid.setTarget(current_wp)
            self.waypoint_reached = True
            self.stage = 'rotate_to_face_selected_waypoint'
            self.stage_pid.setTarget(current_wp)

        delta_x = current_wp[0] - self.obs_current_state[0]
        delta_y = current_wp[1] - self.obs_current_state[1]
        position_error = np.sqrt(delta_x**2 + delta_y**2)
        twist_msg = Twist()
        
        # Stage 1: Rotate to face the selected waypoint
        if self.stage == 'rotate_to_face_selected_waypoint':
            desired_heading = self.get_desired_heading_to_goal(current_wp)
            heading_error = desired_heading - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            
            if abs(heading_error) < 0.05:
                # Switch to DRIVE
                self.stage = 'drive'
                twist_msg.angular.z = 0.0
            else:
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
        # Stage 2: Drive towards the selected waypoint
        elif self.stage == 'drive':
            position_error = np.sqrt(delta_x**2 + delta_y**2)
            if position_error < self.tolerance:
                # We are close enough to the waypoint, switch to final orientation stage
                self.stage = 'rotate_to_final_orientation'
                twist_msg.linear.x = 0.0
            else:
                # Still far away, keep driving
                desired_heading = self.get_desired_heading_to_goal(current_wp)
                heading_error = desired_heading - self.current_state[2]
                heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
                if abs(heading_error) > 0.2:
                    # We are pointing too far off course, need to rotate first
                    self.stage = 'rotate_to_face_selected_waypoint'
                    twist_msg.linear.x = 0.0
                else:
                    update_value = self.pid.update(self.current_state)
                    twist_msg.linear.x = float(update_value[0])
        
        # Stage 3: Rotate to the selected waypoint's final orientation
        elif self.stage == 'rotate_to_final_orientation':
            heading_error = current_wp[2] - self.current_state[2]
            heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
            if abs(heading_error) < self.angle_tolerance:
                # We are within tolerance of final orientation, mark waypoint reached
                self.current_waypoint_idx += 1
                self.waypoint_reached = False
                twist_msg.angular.z = 0.0
            else:
                # Rotate constant speed towards final orientation
                twist_msg.angular.z = float(self.get_rotation_direction(heading_error))
        
        self.update_dead_reckoning(twist_msg.linear.x, twist_msg.angular.z)
        # self.cmd_vel_pub.publish(twist_msg)
        
        
    def localization_update(self):
        """Main localization update - tries AprilTag first, then dead reckoning
        
        
        This function is called every 0.1s by a timer in the constructor. We will
        likely place our EKF update logic here.
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
    

def main(args=None):
    rclpy.init(args=args)
    node = Hw3Node()
    
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
