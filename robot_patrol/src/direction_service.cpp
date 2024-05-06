

#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using GetDirection = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node {
public:
    DirectionService() : rclcpp::Node("direction_service") {
        service_ = this->create_service<GetDirection>(
            "/direction_service", 
            std::bind(&DirectionService::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Direction Service is ready.");
    }

private:
    rclcpp::Service<GetDirection>::SharedPtr service_;

    void handle_service(
        const std::shared_ptr<GetDirection::Request> request, 
        std::shared_ptr<GetDirection::Response> response
    ) {
        int array_size = request->laser_data.ranges.size();
        int right_idx = static_cast<int>(array_size * 0.25);
        int left_idx = static_cast<int>(array_size * 0.75);
        float range_max = request->laser_data.range_max;

        float sum_right = 0.0;
        float sum_left = 0.0;
        float sum_front = 0.0;

        int right_count = 0;
        int left_count = 0;
        int front_count = 0;

        // Calculate the average distances
        for (int i = 0; i < array_size; ++i) {
            float range_val = request->laser_data.ranges[i];

            if (range_val <= range_max) {
                if (i < right_idx) {
                    sum_right += range_val;
                    right_count++;
                } else if (i > left_idx) {
                    sum_left += range_val;
                    left_count++;
                } else {
                    sum_front += range_val;
                    front_count++;
                }
            }
        }

        // Avoid division by zero
        float avg_right = right_count ? sum_right / right_count : range_max;
        float avg_left = left_count ? sum_left / left_count : range_max;
        float avg_front = front_count ? sum_front / front_count : range_max;

        // Log averages for debugging purposes
        RCLCPP_INFO(this->get_logger(), "Right Avg: %.2f, Left Avg: %.2f, Front Avg: %.2f", avg_right, avg_left, avg_front);

        // Decide the direction based on the averages
        if (avg_front > (range_max / 2)) {
            response->direction = "forward";
        } else if (avg_right > avg_left) {
            response->direction = "right";
        } else {
            response->direction = "left";
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}