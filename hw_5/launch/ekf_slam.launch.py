from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch file for EKF SLAM node
    """
    
    # Get package share directory
    pkg_share = FindPackageShare(package='hw_5').find('hw_5')
    
    # Path to config file
    config_file = os.path.join(pkg_share, 'configs', 'ekf_slam_params.yaml')
    
    # EKF SLAM node
    ekf_slam_node = Node(
        package='hw_5',
        executable='ekf_slam',
        name='ekf_slam',
        output='screen',
        emulate_tty=True,
        parameters=[config_file]
    )
    
    return LaunchDescription([
        ekf_slam_node,
    ])

