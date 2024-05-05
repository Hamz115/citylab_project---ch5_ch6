// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include "robot_patrol/srv/get_direction.hpp"
// #include "rclcpp/service.hpp"

// using GetDirection = robot_patrol::srv::GetDirection;

// class DirectionService : public rclcpp::Node
// {
// public:
//   DirectionService() : Node("direction_service_node")
//   {
//     server_ = this->create_service<GetDirection>(
//       "/direction_service",
//       std::bind(&DirectionService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
//   }

// private:
//   rclcpp::Service<GetDirection>::SharedPtr server_;


// void handleRequest(
//   const std::shared_ptr<GetDirection::Request> request,
//   std::shared_ptr<GetDirection::Response> response)
// {
//   //Laser rays into 3 sections of 60° each
//   int num_rays = request->laser_data.ranges.size();
//   int section_size = num_rays / 3;

//   //Total distances for each section
//   double total_dist_sec_right = 0.0;
//   double total_dist_sec_front = 0.0;
//   double total_dist_sec_left = 0.0;

//   for (int i = 0; i < section_size; ++i) {
//     total_dist_sec_right += request->laser_data.ranges[i];
//     total_dist_sec_front += request->laser_data.ranges[i + section_size];
//     total_dist_sec_left += request->laser_data.ranges[i + 2 * section_size];
//   }

//   //Direction based on the largest total distance
//   double max_dist = std::max({total_dist_sec_right, total_dist_sec_front, total_dist_sec_left});
//   if (max_dist == total_dist_sec_right) {
//     response->direction = "right";
//   } else if (max_dist == total_dist_sec_front) {
//     response->direction = "forward";
//   } else {
//     response->direction = "left";
//   }
//   }
// };
// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<DirectionService>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
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
    DirectionService() : rclcpp::Node::Node("direction_service_node") {
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