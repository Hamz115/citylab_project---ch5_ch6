#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include <cmath>
#include <algorithm>
#include <functional>
#include <memory>
#include <thread>
#include <cmath>
#include <algorithm>

using Pose2D = geometry_msgs::msg::Pose2D;
using Twist = geometry_msgs::msg::Twist;
using GoToPose = robot_patrol::action::GoToPose;
using Odometry = nav_msgs::msg::Odometry;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
using namespace std::placeholders;

class GoToPoseServer : public rclcpp::Node {
public:
    GoToPoseServer() : rclcpp::Node::Node("go_to_pose_action") {
        this->subscription_ = this->create_subscription<Odometry>("/odom", 10, std::bind(&GoToPoseServer::odom_callback, this, _1));
        this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "/go_to_pose",
            std::bind(&GoToPoseServer::handle_goal, this, _1, _2),
            std::bind(&GoToPoseServer::handle_cancel, this, _1),
            std::bind(&GoToPoseServer::handle_accepted, this, _1)
        );
        this->publisher_ = this->create_publisher<Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Launched /go_to_pose action server.");
    }
private:
    // Attributes
    Pose2D desired_pos_;
    Pose2D current_pos_;
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    rclcpp::Subscription<Odometry>::SharedPtr subscription_;
    rclcpp::Publisher<Twist>::SharedPtr publisher_;

    // Methods
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPose::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), 
        "Received goal with x: %.2f y: %.2f theta: %.2f", goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        const auto goal = goal_handle->get_goal();
        desired_pos_.x = goal->goal_pos.x;
        desired_pos_.y = goal->goal_pos.y;
        desired_pos_.theta = goal->goal_pos.theta;
        std::thread{std::bind(&GoToPoseServer::execute, this, _1), goal_handle}.detach();
        
    }
    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal: x: %.2f, y: %.2f, theta: %.2f",
            desired_pos_.x, desired_pos_.y, desired_pos_.theta);

        // Orientation
        float angle_delta = INFINITY;
        float goal_direction = computeVectorAngle(
            desired_pos_.x - current_pos_.x, 
            desired_pos_.y - current_pos_.y);

        // Feedback
        auto feedback = std::make_shared<GoToPose::Feedback>();
        auto &current_distance = feedback->current_pos;
        current_distance.x = current_pos_.x ;
        current_distance.y = current_pos_.y;
        current_distance.theta = current_pos_.theta;

        // Rotate to target
        auto vel_msg = Twist();
        while (std::abs(angle_delta) > M_PI / 90) {
            goal_handle->publish_feedback(feedback);
            RCLCPP_DEBUG(this->get_logger(), "Goal direction: %.2f", goal_direction);
            RCLCPP_DEBUG(this->get_logger(), "Current orientation: %.2f", current_pos_.theta);
            angle_delta = goal_direction - current_pos_.theta;
            RCLCPP_DEBUG(this->get_logger(), "Angle delta: %.2f", angle_delta);
            vel_msg.angular.z = angle_delta;
            publisher_->publish(vel_msg);
        }
        RCLCPP_INFO(this->get_logger(), "Orientation to goal completed.");
        
        // Move to target
        float distance_delta = INFINITY;
        while (distance_delta > 0.01) {
            goal_handle->publish_feedback(feedback);
            goal_direction = computeVectorAngle(
                desired_pos_.x - current_pos_.x, 
                desired_pos_.y - current_pos_.y);
            angle_delta = goal_direction - current_pos_.theta;
            distance_delta = std::sqrt(
                std::pow(desired_pos_.x - current_pos_.x, 2) + 
                std::pow(desired_pos_.y - current_pos_.y, 2));
            RCLCPP_DEBUG(this->get_logger(), "Distance delta: %.2f", distance_delta);
            vel_msg.linear.x = std::min(float(0.2), distance_delta);
            vel_msg.angular.z = angle_delta;
            publisher_->publish(vel_msg);
        }
        RCLCPP_INFO(this->get_logger(), "Translation to goal completed.");

        // Final orientation pose
        angle_delta = INFINITY;
        vel_msg.linear.x = 0;
        while (std::abs(angle_delta) > M_PI / 360) {
            goal_handle->publish_feedback(feedback);
            angle_delta = desired_pos_.theta * M_PI / 180 - current_pos_.theta;
            RCLCPP_DEBUG(this->get_logger(), "Angle delta: %.2f", angle_delta);
            vel_msg.angular.z = angle_delta;
            publisher_->publish(vel_msg);
        }
        vel_msg.angular.z = 0;
        RCLCPP_INFO(this->get_logger(), "Final goal pose completed.");

        // Send result
        auto result = std::make_shared<GoToPose::Result>();
        result->status = true;
        goal_handle->succeed(result);
    }
    void odom_callback(const Odometry::SharedPtr msg) {
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;
        current_pos_.theta = computeRotationAngleZ(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    }
    float computeRotationAngleZ(const float z, const float w) {
        // Compute the rotation angle in radians
        float thetaRadians = 2 * atan2(z, w);
        return thetaRadians;
    }
    float computeVectorAngle(const float x, const float y) {
        // Compute the angle in radians
        float thetaRadians = atan2(y, x);
        return thetaRadians;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPoseServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}