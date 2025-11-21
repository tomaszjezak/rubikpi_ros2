from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch file for hw_4 package.
    Starts nodes in sequence:
    1. robot_vision_camera - camera driver
    2. apriltag_ros - AprilTag detection
    3. camera_tf - static tf_node
    4. [5s delay] motor_control, velocity_mapping, ekf_slam_node
    5. [8s delay] prm_planner_node - path planning service
    6. [10s delay] hw4 - PID-based waypoint navigation with PRM integration
    """
    
    # Get package paths
    camera_pkg_path = FindPackageShare(package='robot_vision_camera').find('robot_vision_camera')
    apriltag_pkg_path = FindPackageShare(package='apriltag_ros').find('apriltag_ros')
    
    camera_launch_file = os.path.join(camera_pkg_path, 'launch', 'robot_vision_camera.launch.py')
    apriltag_launch_file = os.path.join(apriltag_pkg_path, 'launch', 'apriltag_launch.py')
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )
    
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(apriltag_launch_file)
    )
    
    motor_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw_4',
                executable='motor_control',
                name='motor_control',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    velocity_mapping = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw_4',
                executable='velocity_mapping',
                name='velocity_mapping',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    camera_tf = Node(
        package='hw_4',
        executable='camera_tf',
        name='camera_tf',
        output='screen',
        emulate_tty=True,
    )

    ekf_slam = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw_4',
                executable='ekf_slam',
                name='ekf_slam_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    prm_planner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='prm_planner',
                executable='prm_planner_node',
                name='prm_planner_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    hw4 = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='hw_4',
                executable='hw4',
                name='hw4',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'start_x': 1.9685,  # Robot at old (0,0) = new (1.9685, 0.762)
                    'start_y': 0.7620,
                    'goal_x': 0.5588,   # 22" to the right of Tag 4
                    'goal_y': 2.1256,   # +0.5m in y direction from previous
                    'use_prm_planner': True,  # Enable PRM path planning
                }]
            )
        ]
    )

    return LaunchDescription([
        camera_launch,
        apriltag_launch,
        camera_tf,
        motor_controller,
        velocity_mapping,
        ekf_slam,
        prm_planner,
        hw4,
    ])
