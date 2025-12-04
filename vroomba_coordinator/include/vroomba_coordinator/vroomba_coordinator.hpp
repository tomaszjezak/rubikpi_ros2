#ifndef VROOMBA_COORDINATOR__VROOMBA_COORDINATOR_NODE_HPP_
#define VROOMBA_COORDINATOR__VROOMBA_COORDINATOR_NODE_HPP_

// Vroomba Architecture - Subsumption-based grid coverage coordinator

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "grid_coverage_base/grid_map.hpp"

#include <memory>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>

namespace vroomba_coordinator
{

class VroombaCoordinatorNode : public rclcpp::Node
{
public:
    VroombaCoordinatorNode():
}

private:
// ROS subscribers
//ROS publishers
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;


// Grid map
std::unique_ptr<grid_coverage_base::GridMap> grid_map_;
bool grid_initialized_;

// robot state
geometry_msgs::msg::PoseStamped::SharedPtr latest_robot_pose_;
rclcpp::Time last_pose_time_;

// Parameters
double cell_size_;
double robot_radius_;

// random walk state
std::mt19937 rng_;
rclcpp::Time random_turn_start_time_;
bool in_random_turn_;

// callbacks
void robot_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
void control_loop();

// vroomba architecture subsumption layers
geometry_msgs::msg::Twist layer1_smart_wander();
geometry_msgs::msg::Twist layer2_random_walk();

// helper functions
bool is_grid_initialized() const;
void update_grid_map();




}