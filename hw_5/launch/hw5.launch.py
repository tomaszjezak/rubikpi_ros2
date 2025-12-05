from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    Launch file for hw_5 package with Vroomba grid coverage.
    Starts nodes in sequence:
    1. robot_vision_camera - camera driver
    2. apriltag_ros - AprilTag detection
    3. camera_tf - static tf_node
    4. [5s delay] motor_control, velocity_mapping, ekf_slam_node
    5. [8s delay] vroomba_coordinator_node - grid coverage waypoint generation
    6. [10s delay] hw5 - PID-based waypoint navigation with vroomba integration

    Usage:
        ros2 launch hw_5 hw5.launch.py                              # default mode, movement enabled
        ros2 launch hw_5 hw5.launch.py enable_movement:=false      # disable robot movement
    """
    
    # Declare launch argument for planning mode
    planning_mode_arg = DeclareLaunchArgument(
        'planning_mode',
        default_value='distance',
        description='Path planning mode: "distance" (minimize path length) or "safe" (maximize clearance)',
        choices=['distance', 'safe']
    )

    # Declare launch argument for enable_movement
    enable_movement_arg = DeclareLaunchArgument(
        'enable_movement',
        default_value='true',
        description='Enable robot movement: "true" or "false"'
    )
    
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
                package='hw_5',
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
                package='hw_5',
                executable='velocity_mapping',
                name='velocity_mapping',
                output='screen',
                emulate_tty=True,
            )
        ]
    )
    
    camera_tf = Node(
        package='hw_5',
        executable='camera_tf',
        name='camera_tf',
        output='screen',
        emulate_tty=True,
    )

    # Get config file path for ekf_slam
    hw5_pkg_path = FindPackageShare(package='hw_5').find('hw_5')
    ekf_config_file = os.path.join(hw5_pkg_path, 'configs', 'ekf_slam_params.yaml')

    ekf_slam = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hw_5',
                executable='ekf_slam',
                name='ekf_slam_node',
                output='screen',
                emulate_tty=True,
                parameters=[ekf_config_file]
            )
        ]
    )

    # prm_planner = TimerAction(
    #     period=8.0,
    #     actions=[
    #         Node(
    #             package='prm_planner',
    #             executable='prm_planner_node',
    #             name='prm_planner_node',
    #             output='screen',
    #             emulate_tty=True,
    #         )
    #     ]
    # )

    vroomba_coordinator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='vroomba_coordinator',
                executable='vroomba_coordinator_node',
                name='vroomba_coordinator_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    hw5 = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='hw_5',
                executable='hw5',
                name='hw5',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'start_x': 1.9685,  # Robot at old (0,0) = new (1.9685, 0.762)
                    'start_y': 0.7620,
                    'goal_x': 0.31,   # 22" to the right of Tag 4
                    'goal_y': 2.117725,   # +0.5m in y direction from previous
                    'use_prm_planner': False,  # Disable PRM path planning
                    'enable_movement': LaunchConfiguration('enable_movement'),
                }],
                arguments=['--planning-mode', LaunchConfiguration('planning_mode')]
            )
        ]
    )

    return LaunchDescription([
        planning_mode_arg,
        enable_movement_arg,
        camera_launch,
        apriltag_launch,
        camera_tf,
        motor_controller,
        velocity_mapping,
        ekf_slam,
        vroomba_coordinator,
        hw5,
    ])
