#ifndef VFH_CONTROLLER_HPP_
#define VFH_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace motion_planning
{

class VFHController : public nav2_core::Controller
{
public:
  VFHController() = default;
  ~VFHController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // VFH Logic Helpers
  std::vector<double> createPolarHistogram(const geometry_msgs::msg::PoseStamped & robot_pose);
  
  // Finds the best valley in the histogram and returns the target steering angle (radians)
  double determineSteeringDirection(
      const std::vector<double>& histogram, 
      double goal_heading, 
      double current_yaw);

  // Helper to find a target point on the global path to aim for
  geometry_msgs::msg::PoseStamped getLookAheadPoint(const geometry_msgs::msg::PoseStamped & pose);

  // Members
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("VFHController")};
  nav_msgs::msg::Path global_plan_;

  // Parameters
  double max_linear_vel_;
  double max_angular_vel_;
  double goal_dist_tolerance_;
  double rotate_to_heading_angular_vel_;
  
  // VFH Specific Parameters
  int histogram_sectors_;     // e.g., 72 for 5-degree resolution
  double active_window_size_; // e.g., 2.0 meters
  double obstacle_threshold_; // e.g., 100.0
  double robot_radius_;       // Approximation for simple VFH
  double safety_margin_angle_; // Buffer around obstacles
  
  // Constants for VFH Magnitude
  double vfh_a_ = 10.0;
  double vfh_b_ = 5.0;
};

}  // namespace motion_planning

#endif  // motion_planning__VFH_CONTROLLER_HPP_