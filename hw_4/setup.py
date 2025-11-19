from setuptools import find_packages, setup

package_name = 'hw_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/configs', ['configs/apriltags_position.yaml', 'configs/ekf_slam_params.yaml', 'configs/ground_truth_landmarks.yaml']),
        ('share/' + package_name + '/launch', ['launch/hw4.launch.py', 'launch/ekf_slam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bryce and Tomasz',
    maintainer_email='none@ucsd.edu',
    description='Homework 4',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = hw_4.motor_control:main',
            'velocity_mapping = hw_4.velocity_mapping:main',
            'hw4 = hw_4.hw4:main',
            'camera_tf = hw_4.camera_tf:main',
            'ekf_slam = hw_4.ekf_slam_node:main',
        ],
    },
)
