#include <motion_planning/pd_motion_planner.hpp>

namespace motion_planning
{

    PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner_node"),
                                         kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3), max_angular_velocity_(1.0),
                                         prev_linear_error_(0.0), prev_angular_error_(0.0)
    {
        declare_parameter<double>("kp", kp_);
        declare_parameter<double>("kd", kd_);
        declare_parameter<double>("step_size", step_size_);
        declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

        kp_ = get_parameter("kp").as_double();
        kd_ = get_parameter("kd").as_double();
        step_size_ = get_parameter("step_size").as_double();
        max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

        path_sub_ = create_subscription<nav_msgs::msg::Path>("/path", 10, bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        control_loop_ = create_wall_timer(std::chrono::milliseconds(100), bind(&PDMotionPlanner::controlLoop, this));
        last_cycle_time_ = get_clock()->now();
    }

    void PDMotionPlanner::controlLoop()
    {
        if (global_plan_.poses.empty())
        {
            return;
        }
        geometry_msgs::msg::TransformStamped robot_pose;
        try
        {
            robot_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could not transform : %s", ex.what());
            return;
        }

        RCLCPP_INFO(get_logger(), "frame_id Robot Pose: %s", robot_pose.header.frame_id.c_str());
        RCLCPP_INFO(get_logger(), "frame_id Global Plan: %s", global_plan_.header.frame_id.c_str());

        geometry_msgs::msg::PoseStamped robot_pose_stamped;
        robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
        robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
        robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
        robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;
        auto next_pose = getNextPose(robot_pose_stamped);
    }

    void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        global_plan_ = *path;
    }
    geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped &robot_pose)
    {
        auto next_pose=global_plan_.poses.back();
    }

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motion_planning::PDMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
