#include "vroomba_coordinator/vroomba_coordinator_node.hpp"
#include <chrono>
#include <random>

namespace vroomba_coordinator
{

VroombaCoordinatorNode::VroombaCoordinatorNode()
: Node("vroomba_coordinator_node"),
  grid_initialized_(false),
  rng_(std::random_device{}())
{
  // Declare parameters
  declare_parameter<double>("cell_size", 0.1);
  declare_parameter<double>("robot_radius", 0.11);
  declare_parameter<int>("lookahead_radius", 2);
  // Scoring heuristic weights (for Layer 1 waypoint selection)
  // These control how much each factor matters when choosing which frontier to go to
  declare_parameter<double>("forward_bias_weight", 2.0);  // Prefer cells in front of robot
  declare_parameter<double>("turn_weight", 1.0);  // Prefer smaller turn angles (45° > 90°)
  declare_parameter<double>("density_weight", 1.5);  // Prefer cells with more unknown neighbors
  declare_parameter<double>("distance_weight", 0.5);  // Prefer closer cells
  declare_parameter<double>("viz_rate", 1.0);  // Visualization publish rate (Hz)
  
  // Get parameters
  cell_size_ = get_parameter("cell_size").as_double();
  robot_radius_ = get_parameter("robot_radius").as_double();
  lookahead_radius_ = get_parameter("lookahead_radius").as_int();
  forward_bias_weight_ = get_parameter("forward_bias_weight").as_double();
  turn_weight_ = get_parameter("turn_weight").as_double();
  density_weight_ = get_parameter("density_weight").as_double();
  distance_weight_ = get_parameter("distance_weight").as_double();
  viz_rate_ = get_parameter("viz_rate").as_double();
  
  // Initialize TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize grid map
  grid_map_ = std::make_unique<grid_coverage_base::GridMap>();
  
  // Create subscribers
  robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ekf_slam/robot_pose",
    10,
    std::bind(&VroombaCoordinatorNode::robot_pose_callback, this, std::placeholders::_1)
  );
  
  // Create publishers
  grid_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_coverage/grid_map", 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("/grid_coverage/status", 10);
  
  // Create service
  waypoint_service_ = create_service<vroomba_coordinator::srv::GetNextWaypoint>(
    "get_next_waypoint",
    std::bind(&VroombaCoordinatorNode::waypoint_service_callback, this,
              std::placeholders::_1, std::placeholders::_2)
  );
  
  // Create visualization timer
  viz_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / viz_rate_)),
    std::bind(&VroombaCoordinatorNode::publish_visualization, this)
  );
  
  RCLCPP_INFO(get_logger(), "Vroomba Coordinator Node initialized");
  RCLCPP_INFO(get_logger(), "Service available at: get_next_waypoint");
  RCLCPP_INFO(get_logger(), "Waiting for workspace bounds from EKF SLAM...");
}

void VroombaCoordinatorNode::robot_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    latest_robot_pose_ = msg;
    last_pose_time_ = get_clock()->now();

    // Update grid map if initialized
    if (grid_initialized_) {
        update_grid_map();
    }
}

bool VroombaCoordinatorNode::is_grid_initialized() const
{
    return grid_initialized_;
}

void VroombaCoordinatorNode::update_grid_map()
{
    if (!latest_robot_pose_) return;

    double robot_x = latest_robot_pose_->pose.position.x;
    double robot_y = latest_robot_pose_->pose.position.y;

    // Mark cells as visited using robot footprint
    grid_map_->markVisitedRadius(robot_x, robot_y, robot_radius_);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
double VroombaCoordinatorNode::normalize_angle(double angle)
{
  // Normalize angle to [-pi, pi]
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

int VroombaCoordinatorNode::count_unknown_neighbors(int grid_x, int grid_y)
{
  // Count how many unknown neighbors this cell has (8-connected)
  int count = 0;
  std::vector<std::pair<int, int>> neighbors = grid_map_->get8ConnectedNeighbors(grid_x, grid_y);
  
  for (const auto& [nx, ny] : neighbors) {
    if (grid_map_->getCellState(nx, ny) == grid_coverage_base::CellState::UNKNOWN) {
      count++;
    }
  }
  
  return count;
}

double VroombaCoordinatorNode::score_candidate_cell(
    int cell_grid_x, int cell_grid_y,
    int robot_grid_x, int robot_grid_y, double robot_yaw)
  {
    // Get cell world position
    auto [cell_x, cell_y] = grid_map_->gridToWorld(cell_grid_x, cell_grid_y);
    auto [robot_x, robot_y] = grid_map_->gridToWorld(robot_grid_x, robot_grid_y);
    
    double score = 0.0;
    
    // 1. Forward bias: prefer cells in front of robot
    double dx = cell_x - robot_x;
    double dy = cell_y - robot_y;
    double angle_to_cell = std::atan2(dy, dx);
    double angle_diff = normalize_angle(angle_to_cell - robot_yaw);
    double forward_score = std::cos(angle_diff);  // 1.0 for straight ahead, -1.0 for behind
    score += forward_bias_weight_ * forward_score;

    // 2. Turn minimization: prefer smaller turn angles
    double turn_cost = std::abs(angle_diff);  // 0 to pi
    score -= turn_weight_ * turn_cost;

    // 3. Frontier density: prefer cells with more unknown neighbors
    int unknown_count = count_unknown_neighbors(cell_grid_x, cell_grid_y);
    score += density_weight_ * unknown_count;
    
    // 4. Distance: prefer closer cells
    double distance = std::sqrt(dx*dx + dy*dy);
    score -= distance_weight_ * distance;
    
    return score;
}





}