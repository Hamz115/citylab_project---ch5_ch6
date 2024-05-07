

// #include "rclcpp/rclcpp.hpp"
// #include "robot_patrol/srv/get_direction.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"

// using GetDirection = robot_patrol::srv::GetDirection;

// class DirectionService : public rclcpp::Node {
// public:
//     DirectionService() : rclcpp::Node("direction_service") {
//         service_ = this->create_service<GetDirection>(
//             "/direction_service", 
//             std::bind(&DirectionService::handle_service, this, std::placeholders::_1, std::placeholders::_2)
//         );
//         RCLCPP_INFO(this->get_logger(), "Direction Service is ready.");
//     }

// private:
//     rclcpp::Service<GetDirection>::SharedPtr service_;

//     void handle_service(
//         const std::shared_ptr<GetDirection::Request> request, 
//         std::shared_ptr<GetDirection::Response> response
//     ) {
//         int array_size = request->laser_data.ranges.size();
//         int right_idx = static_cast<int>(array_size * 0.25);
//         int left_idx = static_cast<int>(array_size * 0.75);
//         float range_max = request->laser_data.range_max;

//         float sum_right = 0.0;
//         float sum_left = 0.0;
//         float sum_front = 0.0;

//         int right_count = 0;
//         int left_count = 0;
//         int front_count = 0;

//         // Calculate the average distances
//         for (int i = 0; i < array_size; ++i) {
//             float range_val = request->laser_data.ranges[i];

//             if (range_val <= range_max) {
//                 if (i < right_idx) {
//                     sum_right += range_val;
//                     right_count++;
//                 } else if (i > left_idx) {
//                     sum_left += range_val;
//                     left_count++;
//                 } else {
//                     sum_front += range_val;
//                     front_count++;
//                 }
//             }
//         }

//         // Avoid division by zero
//         float avg_right = right_count ? sum_right / right_count : range_max;
//         float avg_left = left_count ? sum_left / left_count : range_max;
//         float avg_front = front_count ? sum_front / front_count : range_max;

//         // Log averages for debugging purposes
//         RCLCPP_INFO(this->get_logger(), "Right Avg: %.2f, Left Avg: %.2f, Front Avg: %.2f", avg_right, avg_left, avg_front);

//         // Decide the direction based on the averages
//         if (avg_front > (range_max / 2)) {
//             response->direction = "forward";
//         } else if (avg_right > avg_left) {
//             response->direction = "right";
//         } else {
//             response->direction = "left";
//         }
//     }
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<DirectionService>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <functional>
#include <memory>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <iostream>

using GetDirection = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node {
public:
    // Methods
    DirectionService() : rclcpp::Node::Node("direction_service") {
        using namespace std::placeholders;
        service_ = this->create_service<GetDirection>(
            "/direction_service", 
            std::bind(&DirectionService::handle_service, this, _1, _2));

        this->declare_parameter<float>("right_index_multiplier", 0.75);
        this->get_parameter("right_index_multiplier", this->right_index_multiplier_);

        if (this->right_index_multiplier_ == 0.75) {
            RCLCPP_WARN(this->get_logger(), "Direction Service started in SIMULATION mode.");
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Direction Service started in REAL ROBOT mode.");
        }

        RCLCPP_INFO(this->get_logger(), "Server started.");
    }
private:
    // Attributes
    rclcpp::Service<GetDirection>::SharedPtr service_;
    float right_index_multiplier_;

    // Methods
    void handle_service(const std::shared_ptr<GetDirection::Request> request, std::shared_ptr<GetDirection::Response> response) {
        int array_size = request->laser_data.ranges.size();
        int right_idx = array_size * right_index_multiplier_;
        float range_max = request->laser_data.range_max;
        float total_dist_sec_right = 0;
        float total_dist_sec_front = 0;
        float total_dist_sec_left = 0;
        int idx = right_idx;
        int half_array = array_size / 2;
        for (int i = 0; i < half_array; i++) {
            if ((i < half_array / 3) && (request->laser_data.ranges[idx] <= range_max)) {
                total_dist_sec_right += request->laser_data.ranges[idx];
            }
            else if ((i < 2 * half_array / 3) && (request->laser_data.ranges[idx] <= range_max)) {
                total_dist_sec_front += request->laser_data.ranges[idx];
            }
            else if ((i >= 2 * half_array / 3) && (request->laser_data.ranges[idx] <= range_max)) {
                total_dist_sec_left += request->laser_data.ranges[idx];
            }
            idx++;
            if (idx >= array_size) {idx = 0;}
        }
        if ((total_dist_sec_right > total_dist_sec_front) && (total_dist_sec_right > total_dist_sec_left)) {
            response->direction = "right";
        }
        else {
            response->direction = total_dist_sec_front > total_dist_sec_left ? "forward" : "left";
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}