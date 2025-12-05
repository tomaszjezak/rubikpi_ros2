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
  declare_parameter<int>("lookahead_radius", 3);  // Target 3 cells ahead
  declare_parameter<int>("max_search_radius", 15);  // Max expansion for progressive search
  // Scoring heuristic weights (for Layer 1 waypoint selection)
  // These control how much each factor matters when choosing which frontier to go to
  declare_parameter<double>("forward_bias_weight", 2.0);  // Prefer cells in front of robot
  declare_parameter<double>("turn_weight", 1.0);  // Prefer smaller turn angles (45° > 90°)
  declare_parameter<double>("density_weight", 1.5);  // Prefer cells with more unknown neighbors
  declare_parameter<double>("distance_weight", 0.5);  // Prefer closer cells (but see min_distance)
  declare_parameter<double>("min_distance_cells", 3.0);  // Minimum distance in cells (filter out closer ones)
  declare_parameter<double>("viz_rate", 1.0);  // Visualization publish rate (Hz)
  
  // Get parameters
  cell_size_ = get_parameter("cell_size").as_double();
  robot_radius_ = get_parameter("robot_radius").as_double();
  lookahead_radius_ = get_parameter("lookahead_radius").as_int();
  max_search_radius_ = get_parameter("max_search_radius").as_int();
  forward_bias_weight_ = get_parameter("forward_bias_weight").as_double();
  turn_weight_ = get_parameter("turn_weight").as_double();
  density_weight_ = get_parameter("density_weight").as_double();
  distance_weight_ = get_parameter("distance_weight").as_double();
  min_distance_cells_ = get_parameter("min_distance_cells").as_double();
  viz_rate_ = get_parameter("viz_rate").as_double();
  
  // Initialize TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize grid map with hardcoded workspace bounds
  grid_map_ = std::make_unique<grid_coverage_base::GridMap>();
  
  // Hardcoded workspace bounds: x=[0, 2.8194], y=[0, 2.6416]
  grid_map_->initialize(0.0, 2.8194, 0.0, 2.6416, cell_size_);
  grid_initialized_ = true;
  
  auto [width, height] = grid_map_->getGridDimensions();
  RCLCPP_INFO(get_logger(), "Grid initialized with hardcoded bounds: %dx%d cells", width, height);
  RCLCPP_INFO(get_logger(), "Workspace: [0.0, 2.8194] x [0.0, 2.6416] meters");
  RCLCPP_INFO(get_logger(), "Cell size: %.3f meters, Total explorable cells: %d", 
              cell_size_, grid_map_->getTotalCells());
  
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
  RCLCPP_INFO(get_logger(), "Ready to generate waypoints!");
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
  std::vector<std::pair<int, int>> neighbors = grid_map_->getNeighbors8(grid_x, grid_y);
  
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
    
    // 4. Distance: prefer closer cells (within filtered candidates)
    double distance = std::sqrt(dx*dx + dy*dy);
    score -= distance_weight_ * distance;
    
    return score;
}

// layer 2 random walk - fallback
std::vector<std::tuple<double, double, double>> VroombaCoordinatorNode::layer2_random_walk(
    double robot_x, double robot_y, double robot_yaw, int num_waypoints)
{
  RCLCPP_WARN(get_logger(), "🎲 Layer 2: RANDOM WALK (exploring to escape local minimum)");

  std::vector<std::tuple<double, double, double>> waypoints;

  // generate random walk waypoints
  std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);
  std::uniform_real_distribution<double> distance_dist(0.2, 0.5);  // 20 to 50 cm ahead

  for (int i = 0; i < num_waypoints; i++) {
    double random_angle = robot_yaw + angle_dist(rng_);
    double random_distance = distance_dist(rng_);

    double waypoint_x = robot_x + random_distance * std::cos(random_angle);
    double waypoint_y = robot_y + random_distance * std::sin(random_angle);
    
    // Check if waypoint is in workspace
    if (grid_map_->isInWorkspace(waypoint_x, waypoint_y)) {
      waypoints.push_back({waypoint_x, waypoint_y, 0.0});  // hw5 will calculate heading
    }
  }
  
  // If no valid waypoints, just go forward a bit
  if (waypoints.empty()) {
    RCLCPP_WARN(get_logger(), "⚠️  Random walk generated no valid waypoints (all out of workspace)");
    RCLCPP_WARN(get_logger(), "   Attempting to move forward 0.3m as last resort");
    double forward_x = robot_x + 0.3 * std::cos(robot_yaw);
    double forward_y = robot_y + 0.3 * std::sin(robot_yaw);
    waypoints.push_back({forward_x, forward_y, 0.0});  // hw5 will calculate heading
  } else {
    RCLCPP_INFO(get_logger(), "  Random walk generated %zu waypoint(s)", waypoints.size());
  }
  
  return waypoints;
}

// ============================================================================
// LAYER 1: SMART WANDER 
// ============================================================================
std::vector<std::tuple<double, double, double>> VroombaCoordinatorNode::layer1_smart_wander(
    double robot_x, double robot_y, double robot_yaw, int num_waypoints)
{
  RCLCPP_INFO(get_logger(), "Layer 1: Smart Wander (Progressive Radius Expansion)");
  RCLCPP_INFO(get_logger(), "  Searching from radius %d to %d cells (%.1fm to %.1fm)", 
              lookahead_radius_, max_search_radius_,
              lookahead_radius_ * grid_map_->getCellSize(), 
              max_search_radius_ * grid_map_->getCellSize());

  // get robot grid position
  auto [robot_grid_x, robot_grid_y] = grid_map_->worldToGrid(robot_x, robot_y);

  // Progressive radius expansion: start at lookahead_radius_, expand up to max_search_radius_
  for (int search_radius = lookahead_radius_; search_radius <= max_search_radius_; search_radius++) {
    
    // Find all unknown cells within current search radius
    std::vector<CandidateCell> candidates;
    
    for (int dy = -search_radius; dy <= search_radius; dy++) {
      for (int dx = -search_radius; dx <= search_radius; dx++) {
        int check_x = robot_grid_x + dx;
        int check_y = robot_grid_y + dy;

        // Skip if out of bounds
        if (!grid_map_->isInBounds(check_x, check_y)) continue;

        // Only consider UNKNOWN cells
        if (grid_map_->getCellState(check_x, check_y) == grid_coverage_base::CellState::UNKNOWN) {
          // Score this cell using existing scoring function
          double score = score_candidate_cell(check_x, check_y, robot_grid_x, robot_grid_y, robot_yaw);
          candidates.push_back({check_x, check_y, score});
        }
      }
    }
    
    // If we found candidates at this radius, score and return
    if (!candidates.empty()) {
      if (search_radius > lookahead_radius_) {
        RCLCPP_INFO(get_logger(), "✓ Found %zu unknown cells at expanded radius %d (started at %d)", 
                    candidates.size(), search_radius, lookahead_radius_);
      } else {
        RCLCPP_INFO(get_logger(), "✓ Found %zu unknown cells at radius %d", 
                    candidates.size(), search_radius);
      }
      
      // Sort by score (highest first)
      std::sort(candidates.begin(), candidates.end(),
        [](const CandidateCell& a, const CandidateCell& b) {
          return a.score > b.score;
        });
      
      // Generate waypoint from best cell
      std::vector<std::tuple<double, double, double>> waypoints;
      
      auto [best_cell_x, best_cell_y] = grid_map_->gridToWorld(
        candidates[0].grid_x, candidates[0].grid_y);
      
      // Calculate direction from robot to best cell
      double dx = best_cell_x - robot_x;
      double dy = best_cell_y - robot_y;
      double direction = std::atan2(dy, dx);
      
      // Scale waypoint distance with search radius, but cap at 5 cells (0.5m)
      // This makes steps larger in open areas but keeps them reasonable
      int waypoint_distance_cells = std::min(search_radius, 5);
      double target_distance = waypoint_distance_cells * grid_map_->getCellSize();
      double waypoint_x = robot_x + target_distance * std::cos(direction);
      double waypoint_y = robot_y + target_distance * std::sin(direction);
      
      waypoints.push_back({waypoint_x, waypoint_y, 0.0});

      RCLCPP_INFO(get_logger(), "  Best cell at grid (%d, %d), score=%.2f",
                  candidates[0].grid_x, candidates[0].grid_y, candidates[0].score);
      RCLCPP_INFO(get_logger(), "  Generated waypoint: (%.3f, %.3f) at %.3fm away (%d cells)",
                  waypoint_x, waypoint_y, target_distance, waypoint_distance_cells);
      
      return waypoints;
    }
    
    // No cells at this radius, log and expand
    if (search_radius < max_search_radius_) {
      // Only log every few expansions to avoid spam, or at key milestones
      if (search_radius == lookahead_radius_ || search_radius % 3 == 0 || search_radius == max_search_radius_ - 1) {
        RCLCPP_WARN(get_logger(), "  ⟳ Radius %d empty, expanding... (%.1fm search area)", 
                    search_radius, search_radius * grid_map_->getCellSize());
      }
    }
  }
  
  // Exhausted all radii up to max_search_radius - fall back to Layer 2
  RCLCPP_ERROR(get_logger(), "⚠️  Layer 1 FAILED: No unknown cells within %d cell radius!", max_search_radius_);
  RCLCPP_ERROR(get_logger(), "⚠️  FALLING BACK TO Layer 2 (Random Walk)");
  return layer2_random_walk(robot_x, robot_y, robot_yaw, num_waypoints);
}

// ============================================================================
// SERVICE CALLBACK
// ============================================================================
void VroombaCoordinatorNode::waypoint_service_callback(
  const std::shared_ptr<vroomba_coordinator::srv::GetNextWaypoint::Request> request,
  std::shared_ptr<vroomba_coordinator::srv::GetNextWaypoint::Response> response)
{
  RCLCPP_INFO(get_logger(), "Waypoint service called at (%.2f, %.2f, %.2f rad)",
              request->current_x, request->current_y, request->current_yaw);
  
  // Vroomba Architecture: Try Layer 1 (smart wander), fallback to Layer 2 (random walk)
  std::vector<std::tuple<double, double, double>> waypoints = layer1_smart_wander(
    request->current_x, request->current_y, request->current_yaw, request->num_waypoints);
  
  // Pack waypoints into response
  for (const auto& [x, y, theta] : waypoints) {
    response->waypoint_x.push_back(x);
    response->waypoint_y.push_back(y);
    response->waypoint_theta.push_back(theta);
  }
  
  response->success = !waypoints.empty();
  
  if (response->success) {
    RCLCPP_INFO(get_logger(), "Generated %zu waypoint(s)", waypoints.size());
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to generate waypoints");
  }
}

// ============================================================================
// VISUALIZATION
// ============================================================================
void VroombaCoordinatorNode::publish_visualization()
{
  if (!grid_initialized_) return;
  
  // Publish grid map as OccupancyGrid for visualization in RViz
  auto grid_msg = nav_msgs::msg::OccupancyGrid();
  grid_msg.header.stamp = rclcpp::Time(0);  // Use latest available TF
  grid_msg.header.frame_id = "odom";
  
  auto [min_x, max_x, min_y, max_y] = grid_map_->getWorkspaceBounds();
  auto [width, height] = grid_map_->getGridDimensions();
  
  grid_msg.info.resolution = cell_size_;
  grid_msg.info.width = width;
  grid_msg.info.height = height;
  grid_msg.info.origin.position.x = min_x;
  grid_msg.info.origin.position.y = min_y;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  
  grid_msg.data.resize(width * height);
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int idx = y * width + x;
      auto state = grid_map_->getCellState(x, y);
      
      if (state == grid_coverage_base::CellState::WALL) {
        grid_msg.data[idx] = 100;  // Black (occupied)
      } else if (state == grid_coverage_base::CellState::VISITED) {
        grid_msg.data[idx] = 0;  // White (free/visited)
      } else {  // UNKNOWN
        grid_msg.data[idx] = 50;  // Gray (unknown)
      }
    }
  }
  
  grid_map_pub_->publish(grid_msg);
  
  // Publish status message
  auto status_msg = std_msgs::msg::String();
  int visited = grid_map_->getVisitedCount();
  int total = grid_map_->getTotalCells();
  double coverage = total > 0 ? (100.0 * visited / total) : 0.0;
  status_msg.data = "Coverage: " + std::to_string(visited) + "/" + 
                    std::to_string(total) + " cells (" + 
                    std::to_string(static_cast<int>(coverage)) + "%)";
  status_pub_->publish(status_msg);
}

}  // namespace vroomba_coordinator

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vroomba_coordinator::VroombaCoordinatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}