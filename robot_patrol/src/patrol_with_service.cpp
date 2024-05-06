
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <functional>
#include <string>

using GetDirection = robot_patrol::srv::GetDirection;
using namespace std::placeholders;
using namespace std::chrono_literals;

class PatrolWithService : public rclcpp::Node {
public:
    PatrolWithService() : rclcpp::Node("patrol_with_service") {
        // Publisher for robot velocity
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscription for laser scan data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&PatrolWithService::laser_callback, this, std::placeholders::_1));

        // Client for direction service
        client_ = this->create_client<GetDirection>("/direction_service");

        // Timer for the control loop
        control_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&PatrolWithService::control_loop, this));
    }

private:
    // Latest laser scan data
    sensor_msgs::msg::LaserScan::SharedPtr last_laser_;

    // Service client, publisher, and subscription
    rclcpp::Client<GetDirection>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Laser scan callback to store the latest scan data
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_laser_ = msg;
        RCLCPP_INFO(this->get_logger(), "Received new laser scan data.");
    }

    // Control loop to request direction and publish velocity
    void control_loop() {
        if (!last_laser_) {
            RCLCPP_WARN(this->get_logger(), "No laser scan data available.");
            return;
        }

        // Create a request for the direction service
        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *last_laser_;

        // Make an asynchronous request to the direction service
        auto future = client_->async_send_request(request, std::bind(&PatrolWithService::handle_service_response, this, std::placeholders::_1));
    }

    // Handle the service response to control the robot
    void handle_service_response(const rclcpp::Client<GetDirection>::SharedFuture future) {
        try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service response: %s", response->direction.c_str());

            // Create a velocity message based on the service response
            geometry_msgs::msg::Twist vel_msg;
            if (response->direction == "forward") {
                vel_msg.linear.x = 0.1;
                vel_msg.angular.z = 0.0;
            } else if (response->direction == "left") {
                vel_msg.linear.x = 0.1;
                vel_msg.angular.z = 0.5;
            } else if (response->direction == "right") {
                vel_msg.linear.x = 0.1;
                vel_msg.angular.z = -0.5;
            } else {
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
            }

            // Log the final velocity command
            RCLCPP_INFO(this->get_logger(), "Publishing velocity: linear.x = %.2f, angular.z = %.2f", vel_msg.linear.x, vel_msg.angular.z);

            // Publish the velocity message to control the robot
            publisher_->publish(vel_msg);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolWithService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}