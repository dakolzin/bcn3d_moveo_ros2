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

    double velocity_scaling_factor = 0.5;
    move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);

    std::vector<double> position_one = {-0.6653, -0.9057, -0.6047, 1.4261, -0.7028};
    std::vector<double> position_two = {0.5815, -1.2123, 0.6183, -0.4060, -0.4375};
    std::vector<double> position_three = {1.7300, 1.1001, 1.0758, 0.1817, -0.1874};
    std::vector<double> position_four = {-0.3560, -0.9011, -0.8853, 0.5992, 0.0257};
    std::vector<double> position_five = {1.7709, -0.3986, 1.0142, -1.2267, 0.3384};
    std::vector<double> position_six = {-1.0985, 0.1350, -0.7114, -0.5853, 0.4294};
    std::vector<double> position_seven = {1.2472,  0.7518,  1.0599, -1.4877,  0.7517};

    std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    std::vector<double> position_one_ml = {-0.75781119, -0.99384326, -0.6524241,   1.54935598, -0.75852185};
    std::vector<double> position_two_ml = {0.67169005, -1.0837419,   0.67124188, -0.46058482, -0.46116164};
    std::vector<double> position_three_ml = {1.67633533,  1.18677866,  1.21883595,  0.21770142, -0.24970673};
    std::vector<double> position_four_ml = {-0.41288486, -0.91801023, -0.92327142,  0.5983988,   0.03300513};
    std::vector<double> position_five_ml = { 1.70009995, -0.38959593,  0.82402927, -1.17983043,  0.29172063};
    std::vector<double> position_six_ml = {-1.03046823,  0.17030787, -0.55733603, -0.6043002,   0.34988472};
    std::vector<double> position_seven_ml = {0.59103936,  0.51609838,  0.38142109, -0.65852827,  0.65025467};

    std::vector<double> position_one_ml_lstm = {-0.67482221, -0.88891697, -0.54768032,  1.34601223, -0.6913864};
    std::vector<double> position_two_ml_lstm = {0.63754374, -1.17551982,  0.52219993, -0.32940358, -0.36022735};
    std::vector<double> position_three_ml_lstm = {1.7302345,   1.09028208,  1.02912152,  0.20322351, -0.12544556};
    std::vector<double> position_four_ml_lstm = {-0.36365357, -0.86787242, -0.84162647,  0.63228536,  0.0088427};
    std::vector<double> position_five_ml_lstm = {1.91389847, -0.44455704,  0.98546189, -1.22224474,  0.30686867};
    std::vector<double> position_six_ml_lstm = {-1.0275048,   0.12331457, -0.71804118, -0.60634899,  0.38383964};
    std::vector<double> position_seven_ml_lstm = {0.94871801,  0.91185313,  0.88323039, -1.53016281,  0.66917706};

    while (rclcpp::ok() && !shutdown_requested.load() ) {
        move_to_position(node, move_group, position_seven, "/======================== Returning real =======================/");
        sleep(3);
        move_to_position(node, move_group, position_seven_ml, "/======================== Returning ml =======================/");
        sleep(3);
        move_to_position(node, move_group, home_position, "/======================== Returning home =======================/");
    }

    rclcpp::shutdown();
    return 0;
}
