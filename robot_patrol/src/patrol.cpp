#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>

class Patrol : public rclcpp::Node
{
public:
  Patrol() : Node("patrol_node")
  {
    // Subscriber to the laser scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
  }


private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int range_size = msg->ranges.size();
    float safe_distance = 0.3; // 30 cm
    bool obstacle_detected = false;

    // Assuming 180 degrees in front and that the scans are ordered from -90 to +90 degrees
    for(int i = range_size / 4; i < 3 * range_size / 4; ++i)
    {
        if(msg->ranges[i] < safe_distance)
        {
            obstacle_detected = true;
            break;
        }
    }

    if(obstacle_detected)
    {
        // Obstacle detected! Now find the largest gap.
        float max_distance = 0.0;
        int max_index = 0;

        // Scan for the largest gap in the 180-degree range
        for(int i = range_size / 4; i < 3 * range_size / 4; ++i)
        {
            if(msg->ranges[i] > max_distance && msg->ranges[i] < std::numeric_limits<float>::infinity())
            {
                max_distance = msg->ranges[i];
                max_index = i;
            }
        }

        // Calculate the angle from the index
        direction_ = msg->angle_min + max_index * msg->angle_increment;
        // Now you have the direction to move to avoid the obstacle

        // Convert radians to degrees for human-readable output
        double angle_in_degrees = direction_ * (180.0 / M_PI);

        // Normalize the angle to [-180, 180] degrees range
        if (angle_in_degrees > 180.0) angle_in_degrees -= 360.0;
        if (angle_in_degrees < -180.0) angle_in_degrees += 360.0;

        RCLCPP_INFO(this->get_logger(), "Obstacle detected! Rotate %f degrees.", angle_in_degrees);
        }
       else
        {
        // No obstacle within 30 cm detected. You can continue moving forward.
        direction_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Path is clear. Moving forward."); // This means straight ahead
       }


}

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  float direction_; // The angle to the safest direction
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}