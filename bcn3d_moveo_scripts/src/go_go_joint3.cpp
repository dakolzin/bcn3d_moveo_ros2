#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <signal.h>
#include <atomic>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> // Для подписки на топик с позициями суставов

std::atomic<bool> shutdown_requested(false);

void signalHandler(int signum) {
    shutdown_requested.store(true);
}

void move_to_position(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group, const std::vector<double>& joint_values, const std::string& message) {
    RCLCPP_INFO(node->get_logger(), "%s", message.c_str());

    move_group.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    bool joint_success = (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing plan %s", joint_success ? "SUCCESS" : "FAILED");

    if (joint_success) {
        RCLCPP_INFO(node->get_logger(), "Executing the joint trajectory.");
        move_group.execute(joint_plan);
    }

    RCLCPP_INFO(node->get_logger(), "Sleeping between moves.");
    rclcpp::sleep_for(std::chrono::seconds(3));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);

    auto node = rclcpp::Node::make_shared("move_group_interface_example");
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

    double velocity_scaling_factor = 0.8;
    move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);

    auto joint_positions_callback = [&move_group, node](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::vector<double> joint_positions = msg->data;
        move_to_position(node, move_group, joint_positions, "Received new position from topic.");
    };

    auto subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_positions_ml", 10, joint_positions_callback);

    RCLCPP_INFO(node->get_logger(), "Robot is ready to receive joint positions.");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
