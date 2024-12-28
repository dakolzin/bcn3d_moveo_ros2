#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

bool move_to_pose(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::msg::Pose& target_pose, const std::string& message) {
    RCLCPP_INFO(node->get_logger(), message.c_str());
    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(10.0);
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    bool pose_success = (move_group.plan(pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Visualizing plan %s", pose_success ? "" : "FAILED");
    if (pose_success) {
        move_group.execute(pose_plan);
    }
    rclcpp::sleep_for(std::chrono::seconds(3));

    return pose_success;
}

bool move_to_pick_point(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group) {
    auto clock = node->get_clock();
    tf2_ros::Buffer tf_buffer(clock);
    tf2_ros::TransformListener tf_listener(tf_buffer);

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        // Подождите, пока преобразование станет доступно
        transform_stamped = tf_buffer.lookupTransform("Link_0", "Pick_point", tf2::TimePointZero, tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node->get_logger(), "Could not get transform: %s", ex.what());
        return false;
    }

    // Преобразование TF в Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = transform_stamped.transform.translation.x;
    target_pose.position.y = transform_stamped.transform.translation.y;
    target_pose.position.z = transform_stamped.transform.translation.z;
    target_pose.orientation = transform_stamped.transform.rotation;

    // Задаем цель и планируем движение
    return move_to_pose(node, move_group, target_pose, "Moving to Pick_point...");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_to_pick_point");

    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    
    if (!move_to_pick_point(node, move_group)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to Pick_point");
    }

    rclcpp::shutdown();
    return 0;
}
