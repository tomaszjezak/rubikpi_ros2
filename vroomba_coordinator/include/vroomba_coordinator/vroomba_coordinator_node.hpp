#ifndef VROOMBA_COORDINATOR__VROOMBA_COORDINATOR_NODE_HPP_
#define VROOMBA_COORDINATOR__VROOMBA_COORDINATOR_NODE_HPP_

// Vroomba Architecture - Subsumption-based grid coverage coordinator

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "grid_coverage_base/grid_map.hpp"
#include "vroomba_coordinator/srv/get_next_waypoint.hpp"

#include <memory>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>

namespace vroomba_coordinator
{

struct CandidateCell {
  int grid_x;
  int grid_y;
  double score;
};

class VroombaCoordinatorNode : public rclcpp::Node
{
public:
  VroombaCoordinatorNode();

private:
  // ROS2 subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  
  // ROS2 publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  
  // ROS2 service
  rclcpp::Service<vroomba_coordinator::srv::GetNextWaypoint>::SharedPtr waypoint_service_;
  
  // Timer for visualization
  rclcpp::TimerBase::SharedPtr viz_timer_;
  
  // TF
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Grid map
  std::unique_ptr<grid_coverage_base::GridMap> grid_map_;
  bool grid_initialized_;
  
  // Robot state
  geometry_msgs::msg::PoseStamped::SharedPtr latest_robot_pose_;
  rclcpp::Time last_pose_time_;
  
  // Parameters
  double cell_size_;
  double robot_radius_;
  int lookahead_radius_;
  int max_search_radius_;  // Maximum search radius for progressive expansion
  double forward_bias_weight_;  // Weight for forward bias heuristic
  double turn_weight_;  // Weight for turn minimization heuristic
  double density_weight_;  // Weight for frontier density heuristic
  double distance_weight_;  // Weight for distance heuristic
  double min_distance_cells_;  // Minimum distance in cells (filter out closer waypoints)
  double viz_rate_;  // Visualization publish rate (Hz)
  
  // Random number generator
  std::mt19937 rng_;
  
  // Callbacks
  void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void waypoint_service_callback(
    const std::shared_ptr<vroomba_coordinator::srv::GetNextWaypoint::Request> request,
    std::shared_ptr<vroomba_coordinator::srv::GetNextWaypoint::Response> response);
  void publish_visualization();
  
  // Vroomba Architecture Subsumption Layers
  std::vector<std::tuple<double, double, double>> layer1_smart_wander(
    double robot_x, double robot_y, double robot_yaw, int num_waypoints);
  std::vector<std::tuple<double, double, double>> layer2_random_walk(
    double robot_x, double robot_y, double robot_yaw, int num_waypoints);
  
  // Helper functions
  bool is_grid_initialized() const;
  void update_grid_map();
  double score_candidate_cell(
    int cell_grid_x, int cell_grid_y,
    int robot_grid_x, int robot_grid_y, double robot_yaw);
  int count_unknown_neighbors(int grid_x, int grid_y);
  double normalize_angle(double angle);
};

}  // namespace vroomba_coordinator

#endif  // VROOMBA_COORDINATOR__VROOMBA_COORDINATOR_NODE_HPP_

