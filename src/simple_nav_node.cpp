#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <memory>

class SimpleNavNode : public rclcpp::Node
{
public:
    SimpleNavNode() : Node("simple_nav_node")
    {
        // Create publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&SimpleNavNode::goalPoseCallback, this, std::placeholders::_1));

        // Subscribe to costmap
        costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
            "local_costmap/costmap_raw", 10,
            std::bind(&SimpleNavNode::costmapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Simple Navigation Node has been initialized");
    }

private:
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Simple proportional control based on goal pose in base_link frame
        double distance = std::sqrt(
            msg->pose.position.x * msg->pose.position.x +
            msg->pose.position.y * msg->pose.position.y
        );

        // Simple proportional control
        geometry_msgs::msg::Twist cmd_vel;
        if (distance > 0.1) {
            // Linear velocity proportional to distance, capped at 0.5 m/s
            cmd_vel.linear.x = std::min(0.5, distance);
            
            // Angular velocity proportional to angle to goal
            double angle_to_goal = std::atan2(msg->pose.position.y, msg->pose.position.x);
            cmd_vel.angular.z = angle_to_goal * 0.5;
        }

        cmd_vel_pub_->publish(cmd_vel);
    }

    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        // Store the latest costmap data
        // This could be used for obstacle avoidance
        RCLCPP_DEBUG(this->get_logger(), "Received costmap update");
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 