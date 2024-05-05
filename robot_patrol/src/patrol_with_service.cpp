#include <rclcpp/rclcpp.hpp>
#include "robot_patrol/srv/get_direction.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <chrono>

using GetDirection = robot_patrol::srv::GetDirection;
using namespace std::chrono_literals;
using namespace std::placeholders;

class Patrol : public rclcpp::Node {
public:
    // Public methods
    Patrol() : rclcpp::Node("robot_patrol_service") {
        client_ = this->create_client<GetDirection>("/direction_service");
        // Subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&Patrol::laserCallback, this, std::placeholders::_1));
        // Publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    

        std::string mode;
        this->declare_parameter<std::string>("mode", "simulation");
        this->get_parameter("mode", mode);
        RCLCPP_INFO(this->get_logger(), "Mode is: %s.", mode.c_str());
        if (mode == "simulation") {
            this->front_idx_ = 0;
        }
        else {
            this->front_idx_ = 360;
        }
    }
private:

    
    // Private methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *msg;
        this->front_reading_ = msg->ranges[this->front_idx_];
        auto future_result = client_->async_send_request(
            request,
            std::bind(&Patrol::velPub, this, _1));
    }

    void velPub(const rclcpp::Client<GetDirection>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Got response: %s", response->direction.c_str());
        geometry_msgs::msg::Twist vel_message;
        if (front_reading_ > 1.0) {
            RCLCPP_INFO(this->get_logger(), "Front object detected at %.2f m, moving forward.", front_reading_);
            vel_message.linear.x = 0.1;
            vel_message.angular.z = 0.0;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Using service direction.");
            if (response->direction == "forward") {
                vel_message.linear.x = 0.1;
                vel_message.angular.z = 0.0;
            }
            else if (response->direction == "left") {
                vel_message.linear.x = 0.1;
                vel_message.angular.z = 0.5;
            }
            else if (response->direction == "right") {
                vel_message.linear.x = 0.1;
                vel_message.angular.z = -0.5;
            }
            else {
                vel_message.linear.x = 0.0;
                vel_message.angular.z = 0.0;
            }
        }
        publisher_->publish(vel_message);
    

    }
    // Private attributes
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<GetDirection>::SharedPtr client_;
    int front_idx_;
    float front_reading_;
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


