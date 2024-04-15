#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <limits>

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("patrol_node") {
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "laser_scan", 10, std::bind(&Patrol::laserCallback, this, std::placeholders::_1));

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        direction_ = 0.0;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::updateMovement, this));
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const size_t mid_index = msg->ranges.size() / 2;
        const float angle_min = msg->angle_min;
        const float angle_increment = msg->angle_increment;

        float max_distance = msg->range_min;
        int max_index = mid_index;  // Default to straight ahead

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            // Make sure we are within the specified range limits
            if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max) {
                if (msg->ranges[i] > max_distance) {
                    max_distance = msg->ranges[i];
                    max_index = i;
                }
            }
        }

        if (max_distance < 0.3) { // Obstacle detected within 30 cm
            // Simple obstacle avoidance: turn in the opposite direction
            float angle = angle_min + (max_index * angle_increment);
            if (angle > M_PI / 2) angle -= M_PI;
            if (angle < -M_PI / 2) angle += M_PI;
            direction_ = (angle > 0) ? -1.0 : 1.0; // Turn away from the obstacle
        } else {
            // No close obstacle, proceed straight
            direction_ = 0;
        }

        RCLCPP_INFO(this->get_logger(), "Obstacle distance: %f, direction_: %f", max_distance, direction_);
    }

    void updateMovement() {
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0.1;  // Move forward at a constant speed
        vel_msg.angular.z = direction_;  // Set angular velocity based on direction
        velocity_publisher_->publish(vel_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing velocity - Linear: %f, Angular: %f", vel_msg.linear.x, vel_msg.angular.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float direction_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}