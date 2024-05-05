#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <functional>
#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::chrono_literals;
using namespace std::placeholders;

class TestClient : public rclcpp::Node {
public:
    // Methods
    TestClient() : rclcpp::Node::Node("test_service_node") {
        client_ = this->create_client<GetDirection>("/direction_service");
        subscription_ = this->create_subscription<LaserScan>(
            "/scan",
            1,
            std::bind(&TestClient::scan_callback, this, _1));

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service.");
        }
        RCLCPP_INFO(this->get_logger(), "Client started.");
    }
private:
    // Atributes
    rclcpp::Client<GetDirection>::SharedPtr client_;
    rclcpp::Subscription<LaserScan>::SharedPtr subscription_;

    // Methods
    void scan_callback(const LaserScan::SharedPtr msg) {
        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *msg;
        auto future_result = client_->async_send_request(
            request,
            std::bind(&TestClient::handle_response, this, _1));
    }

    void handle_response(const rclcpp::Client<GetDirection>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Got response: %s", response->direction.c_str());
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


