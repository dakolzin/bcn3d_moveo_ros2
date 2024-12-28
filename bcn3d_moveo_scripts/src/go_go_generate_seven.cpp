#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <fstream>
#include <random>
#include <signal.h>
#include <atomic>
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::atomic<bool> shutdown_requested(false);

void signalHandler(int signum) {
    shutdown_requested.store(true);
}

void generate_random_joint_values(std::vector<double>& joint_values) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis1(-2.09439510239, 2.09439510239);
    std::uniform_real_distribution<> dis2(-1.48, 1.48);
    std::uniform_real_distribution<> dis3(-1.57, 1.57);
    std::uniform_real_distribution<> dis4(-1.57, 1.57);
    std::uniform_real_distribution<> dis5(-0.785, 0.785);

    joint_values.resize(5);  
    joint_values[0] = dis1(gen);
    joint_values[1] = dis2(gen);
    joint_values[2] = dis3(gen);
    joint_values[3] = dis4(gen);
    joint_values[4] = dis5(gen);
}

bool getTransform(const std::string& target_frame, const std::string& source_frame, rclcpp::Node::SharedPtr node, geometry_msgs::msg::TransformStamped &transform)
{
    tf2_ros::Buffer tfBuffer(node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);

    try
    {
        // Получаем текущую трансформацию без ожидания
        transform = tfBuffer.lookupTransform(target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(5, 0));
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(node->get_logger(), "%s", ex.what());
        return false;
    }
}

void move_to_position(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& move_group, const std::vector<double>& joint_values, const std::string& message, std::ofstream& data_file) {
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
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::sleep_for(std::chrono::seconds(1));
    if (getTransform("Link_0", "EE", node, transform))
    {
        // Запись значений суставов
        for(const auto& value : joint_values)
            data_file << std::setprecision(4) << std::fixed << value << " ";

        // Запись трансформации
        data_file << std::setprecision(4) << std::fixed << transform.transform.translation.x << " " 
                  << transform.transform.translation.y << " " 
                  << transform.transform.translation.z << " "
                  << transform.transform.rotation.x << " " 
                  << transform.transform.rotation.y << " " 
                  << transform.transform.rotation.z << " " 
                  << transform.transform.rotation.w << std::endl;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_group_node");
    const std::string ARM_PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, ARM_PLANNING_GROUP);

    std::ofstream data_file("/ros2_bcn3d_moveo/src/data.txt");  // Файл для записи данных

    double velocity_scaling_factor = 1.0;
    move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);
    
    while (rclcpp::ok() && !shutdown_requested.load() ) {
        std::vector<double> joint_values;
        generate_random_joint_values(joint_values);
        move_to_position(node, move_group, joint_values, "/======================== Moving to random joint values =======================/", data_file);
    }

    data_file.close();  // Закрываем файл данных

    rclcpp::shutdown();
    return 0;
}
