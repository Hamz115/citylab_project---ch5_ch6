#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>
class Patrol : public rclcpp::Node {
public:
    // Public methods
    Patrol() : rclcpp::Node("robot_patrol") {
        // Subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
        // Publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::velPub, this));
    }
private:
    // Private methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Reading laser data");
        float laser_idx = 0;
        int range_size = int(msg->ranges.size());
        float laser_max = float(msg->range_max);
        float farthest_object = 0;
        closest_object_ = laser_max;
        // SIMULATED ROBOT
        front_reading_ = msg->ranges[0];
        for (int i = 0; i < range_size; i++) {
            if ((i < 0.25 * range_size) || (i >= 0.75 * range_size)) {
                if (msg->ranges[i] <= laser_max && msg->ranges[i] > farthest_object) {
                    farthest_object = msg->ranges[i];
                    laser_idx = i < 0.25 * range_size ? i : i - range_size;
                }
                if (msg->ranges[i] < closest_object_ && (i < range_size * 0.125 || i > range_size * 0.875 )) {
                    closest_object_ = msg->ranges[i];
                    avoidance_direction_ = i < 0.25 * range_size ? -1 : 1;
                }
            }
        }

        // REAL ROBOT
    //     front_reading_ = msg->ranges[range_size * 0.5];
    //     for (int i = 0; i < range_size; i++) {
    //         if ((i < 0.25 * range_size) || (i >= 0.75 * range_size)) {
    //         if ((i >= 0.25 * range_size) || (i < 0.75 * range_size)) {
    //             if (msg->ranges[i] <= laser_max && msg->ranges[i] > farthest_object) {
    //                 farthest_object = msg->ranges[i];
    //                 laser_idx = i;
    //                 laser_idx = i < 0.25 * range_size ? i : i - range_size;
    //                 laser_idx = i - range_size * 0.5;
    //             }
    //             if (msg->ranges[i] < closest_object_ && (i < range_size * 0.125 || i > range_size * 0.875 )) {
    //             if (msg->ranges[i] < closest_object_ && (i < range_size * 0.625 && i > range_size * 0.375 )) {
    //                 closest_object_ = msg->ranges[i];
    //                 avoidance_direction_ = i < 0.25 * range_size ? -1 : 1;
    //                 avoidance_direction_ = i < 0.5 * range_size ? 1 : -1;
    //             }
    //         }
    //     }
    // }
        direction_ = laser_idx * 2 * pi_ / range_size;

        RCLCPP_INFO(this->get_logger(), "The farthest object is at: %.2f m", farthest_object);
        RCLCPP_INFO(this->get_logger(), "The closest object is at: %.2f m", closest_object_);
        RCLCPP_INFO(this->get_logger(), "The direction is : %.2f radians", direction_);
        RCLCPP_INFO(this->get_logger(), "The direction is : %.2f degrees", direction_ * 180 / pi_);

    }

    void velPub() {
        geometry_msgs::msg::Twist vel_message;

        if (closest_object_ > safety_distance && front_reading_ > safety_distance * 2) {
            vel_message.linear.x = 0.1;
            vel_message.angular.z = 0.0;
        }
        else if (closest_object_ > safety_distance && front_reading_ < safety_distance * 2){
            vel_message.linear.x = 0.05;
            vel_message.angular.z = direction_;
            vel_message.angular.z = direction_ / 2;
        }
        else {
            vel_message.linear.x = 0.0;
            vel_message.angular.z = avoidance_direction_ * 0.5;
        }
        RCLCPP_INFO(this->get_logger(), "Publishing linear velocity of : %.2f m/s", vel_message.linear.x);
        RCLCPP_INFO(this->get_logger(), "Publishing angular velocity of : %.2f rad/s", vel_message.angular.z);
        publisher_->publish(vel_message);
    }
    // Private attributes
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float direction_;
    float closest_object_;
    int avoidance_direction_;
    float front_reading_;
    float safety_distance = 0.25;
    float pi_ = std::atan2(0, -1);
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}


