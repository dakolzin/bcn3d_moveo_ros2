#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "bcn3d_moveo_scripts/msg/arm_joint_state.hpp"
#include "cmath"

using namespace std::chrono_literals;

bcn3d_moveo_scripts::msg::ArmJointState arm_steps;
bcn3d_moveo_scripts::msg::ArmJointState total;
int stepsPerRevolution[6] = {-64000, 4400, -92000, -800, 7200, 0}; // microsteps/revolution (using 16ths) from observation, for each motor
int joint_status = 0;
double cur_angle[6];
int joint_step[6];
double prev_angle[6] = {0, 0, 0, 0, 0, 0};
double init_angle[6] = {0, 0, 0, 0, 0, 0};
double total_steps[6] = {0, 0, 0, 0, 0, 0};
int count = 0;

void cmd_cb(const sensor_msgs::msg::JointState::SharedPtr cmd_arm)
{
    if (count == 0)
    {
        prev_angle[0] = cmd_arm->position[0];
        prev_angle[1] = cmd_arm->position[1];
        prev_angle[2] = cmd_arm->position[2];
        prev_angle[3] = cmd_arm->position[3];
        prev_angle[4] = cmd_arm->position[4];
        prev_angle[5] = cmd_arm->position[5];

        init_angle[0] = cmd_arm->position[0];
        init_angle[1] = cmd_arm->position[1];
        init_angle[2] = cmd_arm->position[2];
        init_angle[3] = cmd_arm->position[3];
        init_angle[4] = cmd_arm->position[4];
        init_angle[5] = cmd_arm->position[5];
    }

    arm_steps.position1 = static_cast<int>((cmd_arm->position[0] - prev_angle[0]) * stepsPerRevolution[0] / (2 * M_PI));
    arm_steps.position2 = static_cast<int>((cmd_arm->position[1] - prev_angle[1]) * stepsPerRevolution[1] / (2 * M_PI));
    arm_steps.position3 = static_cast<int>((cmd_arm->position[2] - prev_angle[2]) * stepsPerRevolution[2] / (2 * M_PI));
    arm_steps.position4 = static_cast<int>((cmd_arm->position[3] - prev_angle[3]) * stepsPerRevolution[3] / (2 * M_PI));
    arm_steps.position5 = static_cast<int>((cmd_arm->position[4] - prev_angle[4]) * stepsPerRevolution[4] / (2 * M_PI));
    arm_steps.position6 = static_cast<int>((cmd_arm->position[5] - prev_angle[5]) * stepsPerRevolution[5] / (2 * M_PI));

    if (count != 0)
    {
        prev_angle[0] = cmd_arm->position[0];
        prev_angle[1] = cmd_arm->position[1];
        prev_angle[2] = cmd_arm->position[2];
        prev_angle[3] = cmd_arm->position[3];
        prev_angle[4] = cmd_arm->position[4];
        prev_angle[5] = cmd_arm->position[5];
    }

    total.position1 += arm_steps.position1;
    total.position2 += arm_steps.position2;
    total.position3 += arm_steps.position3;
    total.position4 += arm_steps.position4;
    total.position5 += arm_steps.position5;

    joint_status = 1;
    count = 1;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bcn3d_moveo_scripts");
    RCLCPP_INFO(node->get_logger(), "In main function");

    auto cmd_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/move_group/fake_controller_joint_states", 1000, cmd_cb);
    auto pub = node->create_publisher<bcn3d_moveo_scripts::msg::ArmJointState>("joint_steps", 50);

    rclcpp::WallRate loop_rate(20);

    while (rclcpp::ok())
    {
        if (joint_status == 1)
        {
            joint_status = 0;
            pub->publish(total);
            RCLCPP_INFO(node->get_logger(), "Published to /joint_steps");
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}