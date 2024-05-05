#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "robot_patrol/srv/get_direction.hpp"
#include "rclcpp/service.hpp"

using GetDirection = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node
{
public:
  DirectionService() : Node("direction_service_node")
  {
    server_ = this->create_service<GetDirection>(
      "/direction_service",
      std::bind(&DirectionService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr server_;


void handleRequest(
  const std::shared_ptr<GetDirection::Request> request,
  std::shared_ptr<GetDirection::Response> response)
{
  //Laser rays into 3 sections of 60° each
  int num_rays = request->laser_data.ranges.size();
  int section_size = num_rays / 3;

  //Total distances for each section
  double total_dist_sec_right = 0.0;
  double total_dist_sec_front = 0.0;
  double total_dist_sec_left = 0.0;

  for (int i = 0; i < section_size; ++i) {
    total_dist_sec_right += request->laser_data.ranges[i];
    total_dist_sec_front += request->laser_data.ranges[i + section_size];
    total_dist_sec_left += request->laser_data.ranges[i + 2 * section_size];
  }

  //Direction based on the largest total distance
  double max_dist = std::max({total_dist_sec_right, total_dist_sec_front, total_dist_sec_left});
  if (max_dist == total_dist_sec_right) {
    response->direction = "right";
  } else if (max_dist == total_dist_sec_front) {
    response->direction = "forward";
  } else {
    response->direction = "left";
  }
  }
};
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}