PRM Planner ROS2 Node Implementation Plan
Overview
Convert the standalone PRM planner Python script into a ROS2 node that can receive planning requests and publish paths. The plan covers package creation, message interfaces, node implementation, and testing.

Phase 1: Package Structure Setup
Step 1.1: Create Package Directory Structure
Navigate to /home/ubuntu/ros2_ws/rubikpi_ros2/
Create new directory: prm_planner/
Create subdirectories:
prm_planner/prm_planner/ (Python package)
prm_planner/configs/ (parameter files)
prm_planner/launch/ (launch files)
prm_planner/resource/ (resource marker file)
prm_planner/test/ (test files)
Step 1.2: Create package.xml
File: prm_planner/package.xml
Dependencies needed:
rclpy (ROS2 Python client library)
geometry_msgs (for Pose, Point messages)
nav_msgs (for Path message)
std_msgs (for basic message types)
sensor_msgs (optional, if using point cloud obstacles)
Set build type to ament_python
Include test dependencies: ament_copyright, ament_flake8, ament_pep257, python3-pytest
Step 1.3: Create setup.py
File: prm_planner/setup.py
Configure entry point for the PRM planner node
Include data files for configs and launch files
Set up console script entry point: prm_planner_node = prm_planner.prm_planner_node:main
Step 1.4: Create setup.cfg
File: prm_planner/setup.cfg
Configure flake8 and other linting tools (copy from hw_4 example)
Step 1.5: Create Resource Marker File
File: prm_planner/resource/prm_planner
Empty file that marks this as a ROS2 resource
Phase 2: Core PRM Algorithm Module
Step 2.1: Extract Core Algorithm to Separate Module
File: prm_planner/prm_planner/prm_core.py
Move all PRM algorithm functions from the original script:
sample_points() function
generate_road_map() function
is_collision() function
dijkstra_planning() function
Node class
Keep algorithm parameters as module-level constants (N_SAMPLE, N_KNN, MAX_EDGE_LEN)
Remove matplotlib dependencies (visualization will be handled separately)
Ensure functions accept rng parameter for reproducibility
Step 2.2: Create Algorithm Parameters Class (Optional)
File: prm_planner/prm_planner/prm_params.py
Create a dataclass or class to hold PRM parameters:
n_sample (default: 500)
n_knn (default: 10)
max_edge_len (default: 30.0)
robot_radius (default: 5.0)
This allows easy parameter passing and configuration
Phase 3: ROS2 Message Interface Design
Step 3.1: Decide on Planning Request Interface
Option A: Service-Based (Recommended for on-demand planning)

Create a service file: prm_planner/srv/PlanPath.srv
Service request contains:
geometry_msgs/PoseStamped start_pose
geometry_msgs/PoseStamped goal_pose
float64 robot_radius
geometry_msgs/Point[] obstacles (or use sensor_msgs/PointCloud2)
Service response contains:
nav_msgs/Path path
bool success
string message
Option B: Topic-Based (For continuous planning)

Subscribe to: /start_pose (geometry_msgs/PoseStamped)
Subscribe to: /goal_pose (geometry_msgs/PoseStamped)
Subscribe to: /obstacles (sensor_msgs/PointCloud2 or custom message)
Publish to: /prm_path (nav_msgs/Path)
Recommendation: Implement Option A (Service) as primary, with Option B as alternative

Step 3.2: Create Service Definition (if using Option A)
File: prm_planner/srv/PlanPath.srv
Define request and response fields
Update setup.py to include service files in data_files
Note: If using standard messages only, skip custom service creation
Step 3.3: Path Message Structure
Use nav_msgs/Path message type
Contains:
std_msgs/Header header (with frame_id, timestamp)
geometry_msgs/PoseStamped[] poses (list of waypoints)
Each PoseStamped contains position (x, y) and orientation (can be set to 0 or calculated)
Phase 4: Obstacle Input Handling
Step 4.1: Obstacle Input Options
Option 1: Parameter-based obstacles (Simple, for testing)

Read obstacle list from ROS2 parameters
Format: List of x, y coordinates
Good for static environments
Option 2: PointCloud2 topic (Realistic)

Subscribe to /obstacles topic (sensor_msgs/PointCloud2)
Convert point cloud to obstacle list
Good for dynamic environments
Option 3: OccupancyGrid (nav_msgs/OccupancyGrid)

Subscribe to /map topic
Convert grid cells to obstacle points
Most realistic for SLAM-based systems
Recommendation: Start with Option 1, add Option 2/3 later

Step 4.2: Create Obstacle Converter Module
File: prm_planner/prm_planner/obstacle_handler.py
Functions:
obstacles_from_params(node) - Read from parameters
obstacles_from_pointcloud(msg) - Convert PointCloud2 to list
obstacles_from_occupancy_grid(msg) - Convert OccupancyGrid to list
Phase 5: ROS2 Node Implementation
Step 5.1: Create Main Node Class
File: prm_planner/prm_planner/prm_planner_node.py
Class: PRMPlannerNode(Node)
Initialize with:
Service server (if using service approach)
OR subscribers/publishers (if using topic approach)
Parameter declarations
Logger setup
Step 5.2: Declare ROS2 Parameters
In __init__() method, declare parameters:

n_sample (default: 500, type: int)
n_knn (default: 10, type: int)
max_edge_len (default: 30.0, type: double)
robot_radius (default: 5.0, type: double)
obstacles_x (default: [], type: double_array)
obstacles_y (default: [], type: double_array)
frame_id (default: "map", type: string)
use_service (default: true, type: bool)
Step 5.3: Implement Service Callback (if using service)
Function: plan_path_callback(request, response)
Extract start/goal poses from request
Get obstacles (from parameters or topic)
Call prm_planning() function
Convert result to nav_msgs/Path
Set response.success and response.path
Return response
Step 5.4: Implement Topic Callbacks (if using topics)
Function: start_pose_callback(msg) - Store start pose
Function: goal_pose_callback(msg) - Store goal pose, trigger planning
Function: obstacles_callback(msg) - Update obstacle list
Function: plan_and_publish() - Execute planning and publish path
Step 5.5: Convert PRM Output to ROS2 Path Message
Function: prm_path_to_ros2_path(rx, ry, frame_id)
Input: Lists rx, ry from PRM algorithm
Output: nav_msgs/Path message
Set header.frame_id and header.stamp
Convert each (x, y) to geometry_msgs/PoseStamped
Set orientation (can use atan2 to next point, or set to 0)
Step 5.6: Integrate PRM Core Algorithm
Import prm_core module
Call prm_planning() with:
Start/goal coordinates extracted from ROS2 messages
Obstacle lists converted from ROS2 messages/parameters
Robot radius from parameters
Algorithm parameters (N_SAMPLE, N_KNN, MAX_EDGE_LEN) from parameters
Step 5.7: Error Handling
Handle cases where no path is found
Validate input poses (check if in obstacle)
Handle empty obstacle lists
Log warnings/errors using self.get_logger()
Step 5.8: Create Main Function
Function: main()
Initialize rclpy
Create node instance
Spin node
Cleanup on shutdown
Phase 6: Configuration Files
Step 6.1: Create Parameter Configuration File
File: prm_planner/configs/prm_params.yaml
Define default parameters:
prm_planner:
  ros__parameters:
    n_sample: 1000
    n_knn: 15
    max_edge_len: 2.0
    robot_radius: 5.0
    frame_id: "map"
    obstacles_x: [0.0, 60.0, 60.0, ...]
    obstacles_y: [0.0, 0.0, 60.0, ...]
Step 6.2: Create Launch File
File: prm_planner/launch/prm_planner.launch.py
Use LaunchDescription and Node from launch and launch_ros
Launch PRM planner node
Load parameter file
Set up topic remappings if needed
Include option to launch with RViz for visualization
Phase 7: Visualization (Optional)
Step 7.1: Create Visualization Node (Optional)
File: prm_planner/prm_planner/prm_visualizer.py
Subscribe to /prm_path topic
Subscribe to /obstacles topic
Use matplotlib to visualize:
Obstacles (black dots)
Sampled points (blue dots)
Roadmap edges (gray lines)
Final path (red line)
Can run as separate node or integrated into main node
Step 7.2: RViz Configuration
Create RViz config file: prm_planner/configs/prm_visualization.rviz
Display:
Path (nav_msgs/Path) as line
Obstacles as markers or point cloud
Start/goal as markers
Phase 8: Testing
Step 8.1: Create Unit Tests
File: prm_planner/test/test_prm_core.py
Test PRM core functions:
test_sample_points()
test_is_collision()
test_dijkstra_planning()
Use pytest framework
Step 8.2: Create Integration Test
File: prm_planner/test/test_prm_node.py
Test ROS2 node:
Test service call (if using service)
Test parameter loading
Test path message conversion
Step 8.3: Manual Testing Steps
Build package: colcon build --packages-select prm_planner
Source workspace: source install/setup.bash
Launch node: ros2 launch prm_planner prm_planner.launch.py
Test service call:
ros2 service call /plan_path prm_planner/srv/PlanPath "{start_pose: {...}, goal_pose: {...}}"
Check published path: ros2 topic echo /prm_path
Phase 9: Integration Considerations
Step 9.1: Coordinate Frame Handling
Ensure all poses use consistent frame_id (typically "map" or "odom")
Handle TF transforms if start/goal are in different frames
Use tf2_ros for coordinate transformations if needed
Step 9.2: Real-time Considerations
PRM planning can be slow for large N_SAMPLE values
Consider:
Making planning asynchronous (don't block service callback)
Caching roadmap if obstacles are static
Using action server for long-running planning tasks
Step 9.3: Tuning Parameters for 5-Degree Curve Requirement
As mentioned in the overview:

Increase n_sample to 1000-1500
Decrease max_edge_len to 2.0-3.0
Increase n_knn to 15-20
Make these configurable via parameters
Phase 10: Documentation
Step 10.1: Create README.md
File: prm_planner/README.md
Document:
Package overview
ROS2 topics/services
Parameters
Usage examples
Tuning guide
Step 10.2: Add Code Comments
Add docstrings to all functions
Document ROS2 message formats
Explain parameter choices
File Structure Summary
prm_planner/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── prm_planner
├── prm_planner/
│   ├── __init__.py
│   ├── prm_planner_node.py      # Main ROS2 node
│   ├── prm_core.py              # Core PRM algorithm
│   ├── prm_params.py            # Parameter class (optional)
│   ├── obstacle_handler.py      # Obstacle conversion utilities
│   └── prm_visualizer.py        # Visualization (optional)
├── configs/
│   ├── prm_params.yaml          # Parameter configuration
│   └── prm_visualization.rviz   # RViz config (optional)
├── launch/
│   └── prm_planner.launch.py    # Launch file
└── test/
    ├── test_prm_core.py
    └── test_prm_node.py
Key Implementation Notes
Message Types: Use nav_msgs/Path for output, geometry_msgs/PoseStamped for start/goal
Service vs Topic: Start with service for on-demand planning, add topic support later
Obstacles: Start with parameter-based, add PointCloud2/OccupancyGrid support incrementally
Frame ID: Always set frame_id in published messages
Parameters: Make all PRM tuning parameters configurable via ROS2 parameters
Error Handling: Always check if path planning succeeded before publishing
Logging: Use ROS2 logger for debugging: self.get_logger().info/warn/error()
Dependencies to Install
sudo apt install ros-$ROS_DISTRO-nav-msgs
sudo apt install ros-$ROS_DISTRO-geometry-msgs
pip3 install numpy scipy matplotlib  # If not already installed

