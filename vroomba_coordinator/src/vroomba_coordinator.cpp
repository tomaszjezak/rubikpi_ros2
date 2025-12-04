#include "vroomba_coordinator/vroomba_coordinator_node.hpp"
#include <chrono>
#include <random>

namespace vroomba_coordinator
{

VroombaCoordinatorNode::VroombaCoordinatorNode()
: Node("vroomba_coordinator_node"),
  grid_initialized_(false),
  in_random_turn_(false),
  rng_(std::random_device{}())
{
  // declare parameters
  this->declare_parameter<double>("cell_size", 0.1);
  this->declare_parameter<double>("robot_radius", 0,11);
  
  // get parameters
  cell_size_ = this->get_parameter("cell_size").as_double();
  robot_radius_ = this->get_parameter("robot_radius").as_double();

  // initialize grid map 
  grid_map_ = std::make_unique<grid_coverage_base::GridMap>();

  // create subscribers
  robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ekf_slam/robot_pose",
    10,
    std::bind(&VroombaCoordinatorNode::robot_pose_callback, this, std::placeholders::_1)
  );

  // create publishers
  grid_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_coverage/grid_map", 10);

  RCLCPP_INFO(this->get_logger(), "Vroomba Coordinator Node initialized");
  RCLCPP_INFO(this->get_logger(), "Waiting for workspace bounds from EKF SLAM...");
}

void VroombaCoordinatorNode::robot_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    latest_robot_pose_ = msg;
    last_pose_time_ = this->get_clock()->now();

    // update grid map if initialized
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

    // mark cells as visited using robot footprint
    grid_map_->markVisitedRadius(robot_x, robot_y, robot_radius);
}

void VroombaCoordinatorNode::control_loop()
{
    // check if grid is initialized
    if (!grid_initialized_) {
        // todo: initialize grid when workspace bounds received from ekf slam
        return;
    }

    // check if we have recent pose data
    if (!latest_robot_pose_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No robot pose received");
        return;
      }
      
      auto now = this->get_clock()->now();
      auto time_since_pose = (now - last_pose_time_).seconds();
      if (time_since_pose > 1.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot pose is stale");
        return;
      }

      // SUBSUMPTION ARCHITECTURE

      geometry_msgs::msg::Twist cmd;
      bool command_set = false;

      // layer 1 - smart wander *medium priority*
      cmd = layer1_smart_wander();

      // layer 2 - random walk *lowest priority / fallback*
      if (!command_set) {
        cmd = layer2_random_walk();
      }

      // publish vel cmd 
}

geometry_msgs::msg::Twist VroombaCoordinatorNode::layer1_smart_wander()
{
    geometry_msgs::msg::Twist cmd;

    if (!l)
}



}