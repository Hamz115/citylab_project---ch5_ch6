#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("robot_patrol"), direction_(0) {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // Initialize the publisher for the robot's velocity
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Initialize the subscriber for the laser scan data
        rclcpp::QoS qos(10);
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;

    
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&Patrol::laserCallback, this, std::placeholders::_1), options);

        // Start the control loop timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::controlLoop, this),callback_group_);
    }

private:
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    bool has_new_scan_ = false;
    double direction_;

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "laserCallback called.");
        auto start = std::chrono::high_resolution_clock::now();

    if (msg == nullptr || msg->ranges.empty()) {
        RCLCPP_WARN(this->get_logger(), "Invalid laser scan data received.");
        return;
    }

    latest_scan_ = msg;
    has_new_scan_ = true;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    float closest_obstacle_distance = std::numeric_limits<float>::max();

    for (float distance : msg->ranges) {
        if (distance >= msg->range_min && distance <= msg->range_max) {
            closest_obstacle_distance = std::min(closest_obstacle_distance, distance);
        }
    }

    int front_readings_count = static_cast<int>((msg->angle_max - msg->angle_min) / msg->angle_increment) + 3;

    int max_index = -1;
    float max_distance = 0.0;

    // Iterate through the front 180-degree readings to find the largest distance
    for (int i = 0; i < front_readings_count; ++i) {
        float distance = msg->ranges[i];
        if (distance >= msg->range_min && distance <= msg->range_max && distance > max_distance) {
            max_distance = distance;
            max_index = i;
        }
    }

    // Determine the direction to move
    if (max_index!= -1) {
        direction_ = msg->angle_min + max_index * msg->angle_increment;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected. Index: %d, Distance: %.2f m, Direction angle: %.2f radians", max_index, max_distance, direction_);
    } else {
        direction_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "No obstacles detected. Moving straight.");
    }
}

    void controlLoop() {
    if (!has_new_scan_ || latest_scan_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "No new scan data.");
        return;
    }

    geometry_msgs::msg::Twist cmd_vel_msg;
    float linear_speed = 0.1;
    float safety_distance = 0.5; // 33 cm

      float closest_obstacle_distance = *std::min_element(latest_scan_->ranges.begin(), latest_scan_->ranges.end());
        RCLCPP_DEBUG(this->get_logger(), "Closest obstacle distance: %.2f m", closest_obstacle_distance);

        if (latest_scan_->ranges.empty()) {
        RCLCPP_WARN(this->get_logger(), "Scan data is empty.");
        return;
    }

    if (closest_obstacle_distance < safety_distance) {
        // Move in a safe direction
        float safe_direction_angle = direction_ + (M_PI / 2); // turn 90 degrees to the right
        if (safe_direction_angle > M_PI) {
            safe_direction_angle -= 2 * M_PI;
        }
        cmd_vel_msg.linear.x = 0.1; // Move slowly
        cmd_vel_msg.angular.z = safe_direction_angle; // Turn towards the safe direction

        RCLCPP_INFO(this->get_logger(), "Obstacle too close. Moving in a safe direction with linear velocity: %.2f and angular velocity: %.2f", cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
    } else {
        cmd_vel_msg.linear.x = linear_speed;
        cmd_vel_msg.angular.z = 0.0; // Go straight

        RCLCPP_INFO(this->get_logger(), "Path is clear. Moving straight with linear velocity: %.2f", linear_speed);
    }

    velocity_publisher_->publish(cmd_vel_msg);
    has_new_scan_ = false; // Reset the flag
}

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


