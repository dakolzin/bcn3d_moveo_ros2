#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// Глобальные переменные для отслеживания состояния
std::atomic<bool> first_movement_executed(false);
std::atomic<bool> second_movement_requested(false);

void move_to_position(moveit::planning_interface::MoveGroupInterface& move_group, 
                      const std::vector<double>& joint_values, 
                      const std::string& message) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", message.c_str());

    move_group.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    bool joint_success = (move_group.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing plan %s", joint_success ? "SUCCESS" : "FAILED");

    if (joint_success) {
        move_group.execute(joint_plan);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing the joint trajectory.");
    }
}


void joint_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, 
                              moveit::planning_interface::MoveGroupInterface& move_group) {
    if (!first_movement_executed.load()) {
        // Выполнение первого движения
        move_to_position(move_group, msg->data, "Performing first movement.");
        first_movement_executed.store(true);
        second_movement_requested.store(true); // Установка флага для выполнения второго движения
    } else if (second_movement_requested.load()) {
        // Выполнение второго движения
        move_to_position(move_group, msg->data, "Performing second movement.");
        second_movement_requested.store(false); // После выполнения второго движения сброс флага
        rclcpp::shutdown(); // Завершение работы узла после выполнения двух движений
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_group_interface_example");
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "arm");

    auto subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_positions_ml", 10, 
        [&move_group](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            std::vector<double> joint_positions(msg->data.begin(), msg->data.end());
            move_to_position(*move_group, joint_positions, "Received new position from topic.");
        });

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Robot is ready to receive joint positions.");

    rclcpp::spin(node);

    return 0;
}
