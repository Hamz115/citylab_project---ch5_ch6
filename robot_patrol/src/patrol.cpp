#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("patrol") {
        // Subscriber to the laser scan topic
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "laser_scan", 10, std::bind(&Patrol::laserCallback, this, std::placeholders::_1));

        // Publisher to the cmd_vel topic
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize the direction variable
        direction_ = 0.0;

        // Control loop timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Patrol::updateMovement, this));
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Process laser scan data to find the safest direction
        const size_t mid_index = msg->ranges.size() / 2;
        size_t range_count = msg->ranges.size();

        float max_distance = 0.0;
        int max_index = mid_index; // Default to straight ahead

        // Find the largest distance within the 180 degrees in front
        for (size_t i = mid_index - range_count / 4; i < mid_index + range_count / 4; ++i) {
            if (msg->ranges[i] > max_distance && msg->ranges[i] < std::numeric_limits<float>::infinity()) {
                max_distance = msg->ranges[i];
                max_index = i;
            }
        }

        if (max_distance < 0.3) { // Obstacle detected within 30 cm
            direction_ = 0; // Stop turning
        } else {
            // Convert index to angle
            float angle = (max_index - mid_index) * msg->angle_increment;
            direction_ = angle; // Set direction to turn
        }
    }

    void updateMovement() {
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0.1; // Always move forward at 0.1 m/s
        vel_msg.angular.z = direction_ / 2; // Set angular velocity based on direction

        velocity_publisher_->publish(vel_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float direction_; // Angle to the safest direction
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}