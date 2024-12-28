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

std::atomic<bool> shutdown_requested(false);

void signalHandler(int signum) {
    shutdown_requested.store(true);
}

void move_to_position(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group, const std::vector<double>& joint_values, const std::string& message) {
    RCLCPP_INFO(node->get_logger(), message.c_str());

    move_group.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    bool joint_success = (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing plan %s", joint_success ? "" : "FAILED");

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
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_group_node");
    const std::string ARM_PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, ARM_PLANNING_GROUP);

    double velocity_scaling_factor = 0.8;
    move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);

    std::vector<double> home_position = {0.0, 0.645772, 0.558505, -0.418879, 0.907571};
    std::vector<double> left_position = {0.994838, 0.994838, 0.558505, -0.418879, 0.907571};
    std::vector<double> right_position = {-0.994838, -0.994838, 0.558505, -0.418879, 0.907571};

    while (rclcpp::ok() && !shutdown_requested.load() ) {
        move_to_position(node, move_group, home_position, "/======================== Returning home =======================/");
        move_to_position(node, move_group, left_position, "/======================== Returning left =======================/");
        move_to_position(node, move_group, home_position, "/======================== Returning home =======================/");
        move_to_position(node, move_group, right_position, "/======================== Returning right =======================/");
    }

    rclcpp::shutdown();
    return 0;
}
