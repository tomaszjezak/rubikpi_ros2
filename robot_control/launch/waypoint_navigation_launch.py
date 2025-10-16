from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # Motor controller node (required for robot communication)
    motor_node = Node(
        package='robot_control',
        executable='motor_control',
        name='motor_controller_node',
        output='screen',
        emulate_tty=True,
        parameters=[],
    )

    # Waypoint navigation node
    waypoint_node = Node(
        package='robot_control',
        executable='waypoint_navigation',
        name='waypoint_navigation_node',
        output='screen',
        emulate_tty=True,
        parameters=[],
    )

    # If the motor controller exits for any reason, bring everything down
    shutdown_when_motor_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=motor_node,
            on_exit=[Shutdown(reason='Motor controller exited')]
        )
    )

    return LaunchDescription([
        motor_node,
        waypoint_node,
        shutdown_when_motor_exits,
    ])
