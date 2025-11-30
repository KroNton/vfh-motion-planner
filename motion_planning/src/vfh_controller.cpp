#include "motion_planning/vfh_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "angles/angles.h"
#include "tf2/utils.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_planning
{

void VFHController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  
  // Declare Parameters
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".active_window_size", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".histogram_sectors", rclcpp::ParameterValue(72));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_threshold", rclcpp::ParameterValue(50.0));

  // Get Parameters
  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".active_window_size", active_window_size_);
  node->get_parameter(plugin_name_ + ".histogram_sectors", histogram_sectors_);
  node->get_parameter(plugin_name_ + ".obstacle_threshold", obstacle_threshold_);

  safety_margin_angle_ = 4 * (2.0 * M_PI / histogram_sectors_); // Safety buffer of 4 sectors
}

void VFHController::activate() {
  // Add publishers/subscribers if needed
}

void VFHController::deactivate() {
  // Cleanup
}

void VFHController::cleanup() {
  // Reset pointers
}

void VFHController::setPlan(const nav_msgs::msg::Path & path) {
  global_plan_ = path;
}

void VFHController::setSpeedLimit(const double & speed_limit, const bool & percentage) {
    if (percentage) {
        max_linear_vel_ = max_linear_vel_ * speed_limit / 100.0;
    } else {
        max_linear_vel_ = speed_limit;
    }
}

// --------------------------------------------------------------------------------
// STAGE 1 & 2: Grid to Polar Histogram
// --------------------------------------------------------------------------------
std::vector<double> VFHController::createPolarHistogram(const geometry_msgs::msg::PoseStamped & robot_pose) {
    std::vector<double> histogram(histogram_sectors_, 0.0);
    double sector_res = (2.0 * M_PI) / histogram_sectors_;

    // Get Robot Map Coords
    unsigned int r_mx, r_my;
    double r_wx = robot_pose.pose.position.x;
    double r_wy = robot_pose.pose.position.y;
    
    if (!costmap_->worldToMap(r_wx, r_wy, r_mx, r_my)) {
        return histogram; // Off map
    }

    // Determine Window Size in Cells
    int window_radius_cells = active_window_size_ / costmap_->getResolution();
    
    int min_i = std::max(0, (int)r_mx - window_radius_cells);
    int max_i = std::min((int)costmap_->getSizeInCellsX(), (int)r_mx + window_radius_cells);
    int min_j = std::max(0, (int)r_my - window_radius_cells);
    int max_j = std::min((int)costmap_->getSizeInCellsY(), (int)r_my + window_radius_cells);

    // Iterate Active Window
    for (int i = min_i; i < max_i; i++) {
        for (int j = min_j; j < max_j; j++) {
            unsigned char cost = costmap_->getCost(i, j);
            
            // Filter: Ignore free space (0) and unknown (255)
            if (cost <= 10 || cost == 255) continue; 

            // Calculate distance
            double o_wx, o_wy;
            costmap_->mapToWorld(i, j, o_wx, o_wy);
            double dist = std::hypot(o_wx - r_wx, o_wy - r_wy);
            
            // VFH Magnitude Formula: m = c^2 * (a - b*d)
            double magnitude = std::pow(double(cost), 2) * (active_window_size_ - dist);
            if (magnitude < 0) magnitude = 0;

            // Calculate Angle
            double angle = std::atan2(o_wy - r_wy, o_wx - r_wx);
            
            // Normalize to [0, 2PI)
            if (angle < 0) angle += 2.0 * M_PI;

            int sector = static_cast<int>(angle / sector_res) % histogram_sectors_;
            histogram[sector] += magnitude;
        }
    }
    return histogram;
}

// --------------------------------------------------------------------------------
// STAGE 3: Valley Selection
// --------------------------------------------------------------------------------
double VFHController::determineSteeringDirection(const std::vector<double>& histogram, double goal_heading, double current_yaw) {
    double sector_res = (2.0 * M_PI) / histogram_sectors_;
    
    // Normalize goal_heading to [0, 2PI)
    if (goal_heading < 0) goal_heading += 2.0 * M_PI;
    int goal_sector = static_cast<int>(goal_heading / sector_res) % histogram_sectors_;

    // Identify Valleys (Consecutive sectors < threshold)
    std::vector<std::pair<int, int>> valleys; // <start_index, end_index>
    bool in_valley = false;
    int start_idx = -1;

    // We loop 2x sectors to handle wrap-around (0 degree crossover) easily
    for (int i = 0; i < histogram_sectors_ * 2; i++) {
        int idx = i % histogram_sectors_;
        bool is_free = histogram[idx] < obstacle_threshold_;

        if (is_free && !in_valley) {
            in_valley = true;
            start_idx = i;
        } else if (!is_free && in_valley) {
            in_valley = false;
            // Only add if it hasn't wrapped around completely (avoid duplicates)
            if (i - start_idx < histogram_sectors_) {
                valleys.push_back({start_idx, i - 1});
            }
        }
    }

    // Heuristic: Choose valley
    // 1. If Goal is in a valley, steer to goal.
    // 2. If Goal is blocked, steer to closest valley edge + safety.
    
    if (valleys.empty()) return current_yaw + M_PI; // Stuck! Turn around.

    double best_angle = current_yaw;
    double min_dist_to_goal = std::numeric_limits<double>::max();

    for (auto v : valleys) {
        int v_start = v.first;
        int v_end = v.second;
        int v_width = v_end - v_start + 1;

        // Check if goal sector is inside this valley (handling wrap-around math is tricky here, simplified logic:)
        // We convert everything to angular distance
        
        // Find center of valley
        double valley_center_idx = (v_start + v_end) / 2.0;
        double valley_center_angle = valley_center_idx * sector_res;
        while(valley_center_angle > 2.0*M_PI) valley_center_angle -= 2.0*M_PI;
        
        double diff = angles::shortest_angular_distance(valley_center_angle, goal_heading);
        
        // If the valley is "wide" and the goal is roughly in that direction
        if (std::abs(diff) < (v_width * sector_res) / 2.0) {
            return goal_heading; // Goal is safe!
        }

        // Otherwise, aim for the edge closer to the goal
        if (std::abs(diff) < min_dist_to_goal) {
            min_dist_to_goal = std::abs(diff);
            // Decide which edge (left or right)
            if (diff > 0) {
                 // Goal is to the left (CCW), aim at end of valley
                 best_angle = (v_end * sector_res); // - safety buffer
            } else {
                 // Goal is to the right (CW), aim at start of valley
                 best_angle = (v_start * sector_res); // + safety buffer
            }
        }
    }

    return best_angle;
}

// --------------------------------------------------------------------------------
// STAGE 4: Compute Velocity
// --------------------------------------------------------------------------------
geometry_msgs::msg::TwistStamped VFHController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
    // 1. Get Histogram
    auto histogram = createPolarHistogram(pose);

    // 2. Find Goal Heading (Global Plan Lookahead)
    // Simple lookahead: Take a point 1.0m away or end of path
    double lookahead_dist = 1.0;
    // (Simplification: assuming global plan has points, real code needs interpolation)
    double target_x = global_plan_.poses.back().pose.position.x; 
    double target_y = global_plan_.poses.back().pose.position.y;
    
    // Iterate path to find first point > lookahead_dist
    for(const auto& p : global_plan_.poses) {
        double d = std::hypot(p.pose.position.x - pose.pose.position.x, p.pose.position.y - pose.pose.position.y);
        if (d > lookahead_dist) {
            target_x = p.pose.position.x;
            target_y = p.pose.position.y;
            break;
        }
    }

    double goal_heading = std::atan2(target_y - pose.pose.position.y, target_x - pose.pose.position.x);
    double current_yaw = tf2::getYaw(pose.pose.orientation);

    // 3. VFH Decision
    double desired_heading = determineSteeringDirection(histogram, goal_heading, current_yaw);

    // 4. Control Law (P-Controller)
    double error = angles::shortest_angular_distance(current_yaw, desired_heading);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header = pose.header;
    cmd.header.stamp = node_.lock()->get_clock()->now();

    // Turn
    cmd.twist.angular.z = 2.0 * error; 
    // Clamp angular
    cmd.twist.angular.z = std::min(std::max(cmd.twist.angular.z, -max_angular_vel_), max_angular_vel_);

    // Drive (slow down if turning hard)
    if (std::abs(error) < 0.5) { // If facing roughly correct direction
        cmd.twist.linear.x = max_linear_vel_;
    } else {
        cmd.twist.linear.x = 0.0; // Rotate in place first
    }

    return cmd;
}

} // namespace motion_planning

// Register the plugin
PLUGINLIB_EXPORT_CLASS(motion_planning::VFHController, nav2_core::Controller)