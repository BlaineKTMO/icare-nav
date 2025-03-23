#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <cmath>

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

        // Subscribe to costmap in map frame
        costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
            "local_costmap/costmap_raw", 10,
            std::bind(&SimpleNavNode::costmapCallback, this, std::placeholders::_1));

        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize costmap
        costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>();

        // Declare parameters
        this->declare_parameter("max_linear_velocity", 0.5);
        this->declare_parameter("max_angular_velocity", 1.0);
        this->declare_parameter("goal_tolerance", 0.1);
        this->declare_parameter("obstacle_threshold", 0.3);  // meters
        this->declare_parameter("goal_weight", 1.0);        // higher value = more aggressive goal seeking
        this->declare_parameter("min_obstacle_distance", 0.2);  // minimum distance to maintain from obstacles
        this->declare_parameter("search_angle_range", 1.57);   // angle range to search for clear path (in radians)
        this->declare_parameter("turn_threshold", 0.1);       // angle difference that triggers turning
        this->declare_parameter("turn_speed", 0.8);          // speed of turning (0.0 to 1.0)

        RCLCPP_INFO(this->get_logger(), "Simple Navigation Node has been initialized");
    }

private:
    // Helper function to check if a point is clear of obstacles
    bool isPointClear(double x, double y) {
        if (!costmap_) return false;
        
        unsigned int mx, my;
        if (costmap_->worldToMap(x, y, mx, my)) {
            unsigned char cost = costmap_->getCost(mx, my);
            return cost == 0;  // 0 means no obstacle
        }
        return false;
    }

    // Helper function to check for obstacles in a direction
    bool hasObstacleInDirection(double angle, double distance) {
        for (double d = 0.1; d < distance; d += 0.1) {
            double check_x = d * std::cos(angle);
            double check_y = d * std::sin(angle);
            if (!isPointClear(check_x, check_y)) {
                return true;
            }
        }
        return false;
    }

    // Helper function to find the best direction to move
    double findBestDirection(double current_angle, double goal_angle, double search_range) {
        double best_angle = current_angle;
        double min_cost = std::numeric_limits<double>::max();
        
        // Search in a range around the current angle
        for (double angle = current_angle - search_range; angle <= current_angle + search_range; angle += 0.1) {
            // Check points along this direction
            bool path_clear = true;
            double total_cost = 0;
            
            for (double d = 0.1; d < 0.5; d += 0.1) {
                double check_x = d * std::cos(angle);
                double check_y = d * std::sin(angle);
                
                if (!isPointClear(check_x, check_y)) {
                    path_clear = false;
                    break;
                }
                
                // Calculate cost based on deviation from goal angle
                double angle_diff = std::abs(angle - goal_angle);
                if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
                total_cost += angle_diff;
            }
            
            if (path_clear && total_cost < min_cost) {
                min_cost = total_cost;
                best_angle = angle;
            }
        }
        
        return best_angle;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        try {
            // Transform goal pose to base_link frame
            geometry_msgs::msg::PoseStamped transformed_pose = tf_buffer_->transform(
                *msg, "base_link", tf2::Duration::from_seconds(1.0));

            // Calculate distance and angle to goal
            double distance = std::sqrt(
                transformed_pose.pose.position.x * transformed_pose.pose.position.x +
                transformed_pose.pose.position.y * transformed_pose.pose.position.y
            );
            double angle_to_goal = std::atan2(transformed_pose.pose.position.y, transformed_pose.pose.position.x);

            // Get parameters
            double max_linear_vel = this->get_parameter("max_linear_velocity").as_double();
            double max_angular_vel = this->get_parameter("max_angular_velocity").as_double();
            double goal_tolerance = this->get_parameter("goal_tolerance").as_double();
            double min_obstacle_distance = this->get_parameter("min_obstacle_distance").as_double();
            double search_range = this->get_parameter("search_angle_range").as_double();
            double goal_weight = this->get_parameter("goal_weight").as_double();
            double turn_threshold = this->get_parameter("turn_threshold").as_double();
            double turn_speed = this->get_parameter("turn_speed").as_double();

            // Calculate velocities
            geometry_msgs::msg::Twist cmd_vel;
            
            if (distance > goal_tolerance) {
                // Check for obstacles in the direction of the goal
                bool obstacle_ahead = hasObstacleInDirection(angle_to_goal, min_obstacle_distance);
                
                if (obstacle_ahead) {
                    // Obstacle detected, prioritize turning
                    cmd_vel.linear.x = 0.0;  // Stop forward motion
                    cmd_vel.angular.z = max_angular_vel * turn_speed;  // Turn at full speed
                } else {
                    // Find the best direction to move
                    double best_angle = findBestDirection(0.0, angle_to_goal, search_range);
                    
                    // Calculate how much we need to turn
                    double angle_diff = best_angle - angle_to_goal;
                    if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
                    if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

                    // Set velocities based on the best direction
                    if (std::abs(angle_diff) > turn_threshold) {
                        // Need to turn more
                        cmd_vel.linear.x = 0.0;  // Stop forward motion while turning
                        cmd_vel.angular.z = std::copysign(max_angular_vel * turn_speed, angle_diff);
                    } else {
                        // Moving in the right direction, proceed towards goal
                        cmd_vel.linear.x = std::min(max_linear_vel, distance * goal_weight);
                        cmd_vel.angular.z = angle_diff * 0.5;  // Small correction to maintain direction
                    }
                }
            }

            // Log the calculated velocities
            RCLCPP_INFO(this->get_logger(), "Sending velocities - linear: %.2f, angular: %.2f", 
                        cmd_vel.linear.x, cmd_vel.angular.z);

            cmd_vel_pub_->publish(cmd_vel);
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform goal pose: %s", ex.what());
        }
    }

    void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
    {
        try {
            // Update costmap with the new data
            costmap_->resizeMap(msg->metadata.size_x, msg->metadata.size_y, msg->metadata.resolution);
            costmap_->updateOrigin(msg->metadata.origin.position.x, msg->metadata.origin.position.y);
            
            // Copy the costmap data
            for (size_t i = 0; i < msg->data.size(); ++i) {
                costmap_->setCost(i % msg->metadata.size_x, i / msg->metadata.size_x, msg->data[i]);
            }

            RCLCPP_DEBUG(this->get_logger(), "Updated costmap in map frame");
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform costmap: %s", ex.what());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 