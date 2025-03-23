#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <cmath>
#include <algorithm>

class DWANavNode : public rclcpp::Node {
public:
    struct Trajectory {
        std::vector<double> x_points;
        std::vector<double> y_points;
        std::vector<double> yaw_points;
        double linear_vel;
        double angular_vel;
        double cost;  // Lower is better
    };

    struct DWAConfig {
        // Velocity sampling
        double max_linear_vel;
        double min_linear_vel;
        double max_angular_vel;
        double min_angular_vel;
        double linear_vel_resolution;
        double angular_vel_resolution;
        
        // Trajectory simulation
        double sim_time;
        double sim_dt;
        
        // Cost weights
        double goal_weight;
        double obstacle_weight;
        double velocity_weight;
        
        // Robot
        double robot_radius;
    };

    DWANavNode() : Node("dwa_nav_node") {
        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publishers and subscribers
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", sensor_qos,
            std::bind(&DWANavNode::scan_callback, this, std::placeholders::_1));
            
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&DWANavNode::goal_callback, this, std::placeholders::_1));

        // DWA configuration - scaled down for smaller spaces
        config_.max_linear_vel = declare_parameter("max_linear_vel", 0.2);  // Reduced from 0.5
        config_.min_linear_vel = declare_parameter("min_linear_vel", 0.0);
        config_.max_angular_vel = declare_parameter("max_angular_vel", 1.0);  // Reduced from 2.0
        config_.min_angular_vel = declare_parameter("min_angular_vel", -1.0);  // Reduced from -2.0
        config_.linear_vel_resolution = declare_parameter("linear_vel_resolution", 0.02);  // Finer resolution
        config_.angular_vel_resolution = declare_parameter("angular_vel_resolution", 0.1);
        config_.sim_time = declare_parameter("sim_time", 1.0);  // Reduced from 2.0 for faster reactions
        config_.sim_dt = declare_parameter("sim_dt", 0.1);
        config_.goal_weight = declare_parameter("goal_weight", 0.5);  // Reduced priority
        config_.obstacle_weight = declare_parameter("obstacle_weight", 2.5);  // Increased priority
        config_.velocity_weight = declare_parameter("velocity_weight", 0.15);
        config_.robot_radius = declare_parameter("robot_radius", 0.15);  // Reduced from 0.3

        goal_tolerance_ = declare_parameter("goal_tolerance", 0.05);  // Reduced from 0.1
        min_detection_range_ = declare_parameter("min_detection_range", 0.05);  // Reduced from 0.1

        // Initialize robot pose in map frame
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_theta_ = 0.0;
        last_update_time_ = now();
        last_cmd_vel_.linear.x = 0.0;
        last_cmd_vel_.angular.z = 0.0;

        // Control loop timer (10Hz for DWA)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DWANavNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "DWA Navigation Node initialized");
    }

private:
    struct ObstacleInfo {
        bool detected;
        double min_distance;
        double angle_to_obstacle;
    };

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Store the scan timestamp for validation
        latest_scan_ = msg;
        latest_scan_time_ = now();
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg->header.frame_id != "map") {
            RCLCPP_ERROR(get_logger(), "Goal must be in map frame!");
            return;
        }
        goal_pose_ = *msg;
        has_goal_ = true;
        movement_start_time_ = now();
        RCLCPP_INFO(get_logger(), "Received new goal in map frame: x=%.2f, y=%.2f", 
                    goal_pose_.pose.position.x, goal_pose_.pose.position.y);
    }

    ObstacleInfo check_obstacles() {
        ObstacleInfo info{false, std::numeric_limits<double>::max(), 0.0};
        
        if (!latest_scan_) {
            return info;
        }

        // Check if scan data is too old (more than 100ms old)
        auto scan_age = now() - latest_scan_time_;
        if (scan_age > rclcpp::Duration::from_seconds(0.1)) {
            RCLCPP_WARN(get_logger(), "Scan data is too old: %.3f seconds", scan_age.seconds());
            return info;
        }

        // Calculate robot's movement since scan was taken
        double dt = scan_age.seconds();
        double dx = last_cmd_vel_.linear.x * cos(robot_theta_) * dt;
        double dy = last_cmd_vel_.linear.x * sin(robot_theta_) * dt;
        double dtheta = last_cmd_vel_.angular.z * dt;

        // Check wider front sector (±90 degrees for better anticipation)
        int front_start = static_cast<int>((M_PI / 2) / latest_scan_->angle_increment);
        int front_end = latest_scan_->ranges.size() - front_start;
        
        double closest_obstacle_angle = 0.0;
        double min_front_dist = std::numeric_limits<double>::max();
        bool front_obstacle = false;
        
        // First check directly in front (±45 degrees - wider detection zone)
        int narrow_front_start = static_cast<int>((M_PI / 4) / latest_scan_->angle_increment);
        int narrow_front_end = latest_scan_->ranges.size() - narrow_front_start;
        
        // Count number of close obstacles to determine if path is blocked
        int close_obstacle_count = 0;
        std::vector<std::pair<double, double>> detected_obstacles;  // Store (angle, range) pairs
        
        // Process scan data with motion compensation
        for (int i = narrow_front_start; i < narrow_front_end; i++) {
            if (std::isfinite(latest_scan_->ranges[i])) {
                double range = latest_scan_->ranges[i];
                
                // Filter out readings below minimum detection range
                if (range < min_detection_range_) {
                    continue;
                }
                
                // Rotate scan reading by 180 degrees to account for backwards mounting
                double raw_angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
                double angle = raw_angle + M_PI;  // Add 180 degrees
                while (angle > M_PI) angle -= 2 * M_PI;  // Normalize to [-π, π]
                
                // Convert scan point to Cartesian coordinates
                double scan_x = range * cos(angle);
                double scan_y = range * sin(angle);
                
                // Apply motion compensation
                scan_x -= dx;
                scan_y -= dy;
                angle = angle - dtheta;
                
                // Convert back to polar coordinates
                range = std::sqrt(scan_x * scan_x + scan_y * scan_y);
                angle = std::atan2(scan_y, scan_x);
                
                // Store valid detection
                if (range < obstacle_slow_dist_) {
                    detected_obstacles.push_back({angle, range});
                }
                
                if (range < min_front_dist) {
                    min_front_dist = range;
                }
                
                // Use hysteresis for obstacle detection
                if (range < obstacle_min_dist_ * 1.2) {  // 20% margin for smoother transitions
                    close_obstacle_count++;
                }
                if (range < obstacle_min_dist_) {
                    front_obstacle = true;
                }
            }
        }
        
        // Cluster obstacles to avoid flickering
        if (!detected_obstacles.empty()) {
            // Find the closest obstacle in each 30-degree sector
            const int num_sectors = 6;  // Dividing the 180-degree front area into 6 sectors
            std::vector<double> sector_ranges(num_sectors, std::numeric_limits<double>::max());
            
            for (const auto& obs : detected_obstacles) {
                double sector_angle = obs.first + M_PI/2;  // Shift to [0, PI]
                int sector = static_cast<int>(sector_angle / (M_PI/num_sectors));
                sector = std::clamp(sector, 0, num_sectors-1);
                sector_ranges[sector] = std::min(sector_ranges[sector], obs.second);
            }
            
            // Find the closest obstacle and its sector
            int closest_sector = 0;
            double closest_range = std::numeric_limits<double>::max();
            for (int i = 0; i < num_sectors; i++) {
                if (sector_ranges[i] < closest_range) {
                    closest_range = sector_ranges[i];
                    closest_sector = i;
                }
            }
            
            // Update obstacle info with the clustered data
            info.min_distance = closest_range;
            info.angle_to_obstacle = (closest_sector * M_PI/num_sectors) - M_PI/2;
        }
        
        // Consider path blocked if there are multiple close obstacles
        if (close_obstacle_count > 3) {  // Reduced threshold due to motion compensation
            front_obstacle = true;
        }
        
        info.detected = front_obstacle;
        return info;
    }

    void update_robot_pose(const geometry_msgs::msg::Twist& cmd_vel) {
        auto current_time = now();
        double dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // Update robot pose based on velocity commands
        double delta_x = cmd_vel.linear.x * cos(robot_theta_) * dt;
        double delta_y = cmd_vel.linear.x * sin(robot_theta_) * dt;
        double delta_theta = cmd_vel.angular.z * dt;

        robot_x_ += delta_x;
        robot_y_ += delta_y;
        robot_theta_ += delta_theta;

        // Normalize angle
        while (robot_theta_ > M_PI) robot_theta_ -= 2 * M_PI;
        while (robot_theta_ < -M_PI) robot_theta_ += 2 * M_PI;

        // Broadcast transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = robot_x_;
        transform.transform.translation.y = robot_y_;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, robot_theta_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // Publish transform slightly into the future to avoid extrapolation errors
        transform.header.stamp = current_time + rclcpp::Duration::from_seconds(0.1);
        tf_broadcaster_->sendTransform(transform);
        latest_transform_time_ = current_time;
        last_cmd_vel_ = cmd_vel;
    }

    bool transform_goal_to_base_link(geometry_msgs::msg::PoseStamped& transformed_goal) {
        try {
            // Get the latest transform time
            tf2::TimePoint latest_time;
            std::string error_str;
            if (!tf_buffer_->canTransform("base_link", "map", tf2::TimePoint(),
                                        &error_str)) {
                RCLCPP_WARN(get_logger(), "Transform not available: %s", error_str.c_str());
                return false;
            }

            // Use the latest available transform time
            auto latest_transform = tf_buffer_->lookupTransform(
                "base_link", "map",
                tf2::TimePoint(),  // latest available transform
                tf2::durationFromSec(0.1)
            );

            // Update goal timestamp to match transform time
            goal_pose_.header.stamp = latest_transform.header.stamp;
            
            // Transform the goal
            transformed_goal = tf_buffer_->transform(
                goal_pose_, "base_link",
                tf2::durationFromSec(0.1)
            );
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "Could not transform goal to base_link: %s", ex.what());
            return false;
        }
    }

    void reset_movement() {
        has_goal_ = false;
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        update_robot_pose(stop_cmd);
        RCLCPP_INFO(get_logger(), "Movement time limit reached, stopping!");
    }

    void control_loop() {
        if (!has_goal_ || !latest_scan_) {
            geometry_msgs::msg::Twist zero_vel;
            update_robot_pose(zero_vel);
            return;
        }

        auto elapsed_time = now() - movement_start_time_;
        if (elapsed_time > rclcpp::Duration::from_seconds(5.0)) {
            reset_movement();
            return;
        }

        geometry_msgs::msg::PoseStamped transformed_goal;
        if (!transform_goal_to_base_link(transformed_goal)) {
            return;
        }

        goal_x_ = transformed_goal.pose.position.x;
        goal_y_ = transformed_goal.pose.position.y;

        auto cmd = compute_velocity();
        cmd_vel_pub_->publish(cmd);
        update_robot_pose(cmd);

        double distance_to_goal = std::sqrt(goal_x_ * goal_x_ + goal_y_ * goal_y_);
        if (distance_to_goal < goal_tolerance_) {
            reset_movement();
        }
    }

    std::vector<Trajectory> generate_trajectories(double current_linear_vel, double current_angular_vel) {
        std::vector<Trajectory> trajectories;
        
        // Calculate dynamic window based on current velocities and acceleration constraints
        double acc_dt = 0.1;  // Time step for acceleration calculation
        double linear_acc = 0.5;  // Reduced from 1.0 m/s²
        double angular_acc = 1.0;  // Reduced from 2.0 rad/s²
        
        double min_achievable_linear_vel = std::max(
            config_.min_linear_vel,
            current_linear_vel - linear_acc * acc_dt
        );
        double max_achievable_linear_vel = std::min(
            config_.max_linear_vel,
            current_linear_vel + linear_acc * acc_dt
        );
        double min_achievable_angular_vel = std::max(
            config_.min_angular_vel,
            current_angular_vel - angular_acc * acc_dt
        );
        double max_achievable_angular_vel = std::min(
            config_.max_angular_vel,
            current_angular_vel + angular_acc * acc_dt
        );

        // Always include a stopping trajectory
        Trajectory stop_traj;
        stop_traj.linear_vel = 0.0;
        stop_traj.angular_vel = 0.0;
        stop_traj.x_points.push_back(0.0);
        stop_traj.y_points.push_back(0.0);
        stop_traj.yaw_points.push_back(0.0);
        trajectories.push_back(stop_traj);

        // Sample velocities from dynamic window
        for (double v = min_achievable_linear_vel; 
             v <= max_achievable_linear_vel; 
             v += config_.linear_vel_resolution) {
            for (double w = min_achievable_angular_vel;
                 w <= max_achievable_angular_vel;
                 w += config_.angular_vel_resolution) {
                
                // Skip the zero velocity case as we already added it
                if (v == 0.0 && w == 0.0) continue;
                
                Trajectory traj;
                traj.linear_vel = v;
                traj.angular_vel = w;
                
                // Initialize trajectory with current position
                double x = 0.0;
                double y = 0.0;
                double yaw = 0.0;
                
                traj.x_points.push_back(x);
                traj.y_points.push_back(y);
                traj.yaw_points.push_back(yaw);
                
                // Simulate trajectory
                for (double t = config_.sim_dt; t <= config_.sim_time; t += config_.sim_dt) {
                    x += v * std::cos(yaw) * config_.sim_dt;
                    y += v * std::sin(yaw) * config_.sim_dt;
                    yaw += w * config_.sim_dt;
                    
                    traj.x_points.push_back(x);
                    traj.y_points.push_back(y);
                    traj.yaw_points.push_back(yaw);
                }
                
                trajectories.push_back(traj);
            }
        }
        
        return trajectories;
    }

    double calculate_obstacle_cost(const Trajectory& traj) {
        if (!latest_scan_) {
            return std::numeric_limits<double>::max();
        }

        double total_cost = 0.0;
        double min_dist_to_obstacle = std::numeric_limits<double>::max();
        const double safety_margin = 0.08;  // Reduced from 0.1
        const double critical_distance = config_.robot_radius + safety_margin;  // ~0.23m total
        const double influence_distance = 0.5;  // Increased from 0.4
        
        // Check each point in the trajectory
        for (size_t i = 0; i < traj.x_points.size(); ++i) {
            double traj_x = traj.x_points[i];
            double traj_y = traj.y_points[i];
            double point_cost = 0.0;
            
            // Check multiple laser beams around the trajectory point
            double scan_dist = std::sqrt(traj_x * traj_x + traj_y * traj_y);
            double base_angle = std::atan2(traj_y, traj_x);
            
            const int num_beams_to_check = 7;
            const double angle_spread = 0.3;
            
            for (int j = -num_beams_to_check; j <= num_beams_to_check; ++j) {
                double check_angle = base_angle + j * (angle_spread / num_beams_to_check);
                
                // Rotate check angle by 180 degrees to account for backwards mounting
                check_angle += M_PI;
                while (check_angle > M_PI) check_angle -= 2 * M_PI;
                
                // Find nearest laser scan point
                int beam_index = static_cast<int>((check_angle - latest_scan_->angle_min) 
                                                / latest_scan_->angle_increment);
                
                if (beam_index >= 0 && beam_index < static_cast<int>(latest_scan_->ranges.size())) {
                    double scan_range = latest_scan_->ranges[beam_index];
                    
                    if (std::isfinite(scan_range) && scan_range > min_detection_range_) {
                        // Calculate distance between trajectory point and obstacle
                        double obstacle_x = scan_range * std::cos(check_angle - M_PI);
                        double obstacle_y = scan_range * std::sin(check_angle - M_PI);
                        double dx = traj_x - obstacle_x;
                        double dy = traj_y - obstacle_y;
                        double dist_to_obstacle = std::sqrt(dx * dx + dy * dy);
                        
                        min_dist_to_obstacle = std::min(min_dist_to_obstacle, dist_to_obstacle);
                        
                        // More gradual cost scaling for obstacles
                        if (dist_to_obstacle < critical_distance) {
                            // Instead of max cost, use very high but finite cost
                            point_cost += 1000.0 * std::exp(-(dist_to_obstacle / critical_distance));
                        } else if (dist_to_obstacle < influence_distance) {
                            // Smoother cost scaling
                            double normalized_dist = (dist_to_obstacle - critical_distance) / 
                                                   (influence_distance - critical_distance);
                            point_cost += 2.0 * std::exp(-normalized_dist);
                        }
                    }
                }
            }
            
            // Weight points closer to the robot more heavily
            double time_weight = 1.0 - (0.6 * static_cast<double>(i) / traj.x_points.size());  // Reduced from 0.8
            total_cost += point_cost * time_weight;
        }
        
        // More gradual penalty for close obstacles
        if (min_dist_to_obstacle < influence_distance) {
            double dist_factor = (influence_distance - min_dist_to_obstacle) / influence_distance;
            total_cost *= (1.0 + dist_factor);  // Reduced from 2.0
        }
        
        return total_cost;
    }

    double calculate_goal_cost(const Trajectory& traj) {
        // Use end point of trajectory
        double final_x = traj.x_points.back();
        double final_y = traj.y_points.back();
        
        // Calculate distance to goal
        double dx = goal_x_ - final_x;
        double dy = goal_y_ - final_y;
        double goal_dist = std::sqrt(dx * dx + dy * dy);
        
        // Calculate heading difference to goal
        double goal_heading = std::atan2(goal_y_, goal_x_);
        double final_heading = traj.yaw_points.back();
        double heading_diff = std::abs(goal_heading - final_heading);
        while (heading_diff > M_PI) heading_diff -= 2 * M_PI;
        
        // Penalize trajectories that move away from the goal
        double start_dist = std::sqrt(goal_x_ * goal_x_ + goal_y_ * goal_y_);
        double dist_penalty = (goal_dist > start_dist) ? 1.5 : 1.0;
        
        return (goal_dist + 0.5 * std::abs(heading_diff)) * dist_penalty;
    }

    double calculate_velocity_cost(const Trajectory& traj) {
        // Allow more movement at low speeds even near obstacles
        double min_desired_vel = 0.05;  // Minimum desired velocity
        double base_cost = 0.0;
        
        if (traj.linear_vel < min_desired_vel) {
            // Penalize very low velocities unless we're very close to obstacles
            base_cost = (min_desired_vel - traj.linear_vel) * 2.0;
        } else {
            // Normal velocity cost calculation
            base_cost = config_.max_linear_vel - traj.linear_vel;
        }
        
        // Reduced penalty for angular velocity
        double angular_cost = 0.15 * std::abs(traj.angular_vel) / config_.max_angular_vel;
        
        return base_cost + angular_cost;
    }

    Trajectory select_best_trajectory(const std::vector<Trajectory>& trajectories) {
        Trajectory best_traj = trajectories[0];
        double min_cost = std::numeric_limits<double>::max();
        
        for (auto& traj : trajectories) {
            double obstacle_cost = calculate_obstacle_cost(traj);
            double goal_cost = calculate_goal_cost(traj);
            double velocity_cost = calculate_velocity_cost(traj);
            
            // Adjust weights to prefer movement
            double total_cost = config_.obstacle_weight * obstacle_cost +
                              config_.goal_weight * goal_cost +
                              config_.velocity_weight * velocity_cost;
            
            // Add small random factor to prevent getting stuck in local minima
            total_cost += (rand() % 100) * 0.0001;
            
            if (total_cost < min_cost) {
                min_cost = total_cost;
                best_traj = traj;
                best_traj.cost = total_cost;
            }
        }
        
        return best_traj;
    }

    geometry_msgs::msg::Twist compute_velocity() {
        geometry_msgs::msg::Twist cmd;
        
        // Generate trajectory candidates
        auto trajectories = generate_trajectories(
            last_cmd_vel_.linear.x,
            last_cmd_vel_.angular.z
        );
        
        if (trajectories.empty()) {
            RCLCPP_WARN(get_logger(), "No valid trajectories generated!");
            return cmd;
        }
        
        // Select best trajectory
        auto best_traj = select_best_trajectory(trajectories);
        
        // Use the velocities from the best trajectory
        cmd.linear.x = best_traj.linear_vel;
        cmd.angular.z = best_traj.angular_vel;
        
        RCLCPP_INFO(get_logger(), 
                    "Selected trajectory: linear=%.2f, angular=%.2f, cost=%.2f",
                    cmd.linear.x, cmd.angular.z, best_traj.cost);
        
        return cmd;
    }

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Latest sensor data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    bool has_goal_ = false;
    geometry_msgs::msg::PoseStamped goal_pose_;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;

    // Robot pose in map frame
    double robot_x_;
    double robot_y_;
    double robot_theta_;
    rclcpp::Time last_update_time_;
    rclcpp::Time latest_transform_time_{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Twist last_cmd_vel_;

    // Movement timing
    rclcpp::Time movement_start_time_{0, 0, RCL_ROS_TIME};

    // Parameters
    double max_vel_x_;
    double max_vel_theta_;
    double acc_x_;
    double acc_theta_;
    double goal_tolerance_;
    double obstacle_min_dist_;
    double obstacle_slow_dist_;
    double angle_tolerance_;
    double min_detection_range_;

    // Add new member variable for scan timing
    rclcpp::Time latest_scan_time_{0, 0, RCL_ROS_TIME};

    DWAConfig config_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWANavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 