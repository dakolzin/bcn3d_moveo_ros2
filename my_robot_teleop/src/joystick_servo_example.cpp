#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <thread>
#include <chrono>
#include <unordered_map>

const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "EE";
const std::string BASE_FRAME_ID = "Link_0";

enum Axis { LEFT_STICK_X, LEFT_STICK_Y, LEFT_TRIGGER, RIGHT_STICK_X, RIGHT_STICK_Y, RIGHT_TRIGGER, D_PAD_X, D_PAD_Y };
enum Button { A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, CHANGE_VIEW, MENU, HOME, LEFT_STICK_CLICK, RIGHT_STICK_CLICK };

std::unordered_map<Axis, double> AXIS_DEFAULTS = {{LEFT_TRIGGER, 1.0}, {RIGHT_TRIGGER, 1.0}};

void addJointCommand(std::unique_ptr<control_msgs::msg::JointJog>& joint, const std::string& name, double velocity) {
    joint->joint_names.push_back(name);
    joint->velocities.push_back(velocity);
}

bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint, const rclcpp::Logger& logger) {
    if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y]) {
        addJointCommand(joint, "Joint_1", axes[D_PAD_X]);
        addJointCommand(joint, "Joint_3", axes[D_PAD_Y]);
        addJointCommand(joint, "Joint_4", buttons[B] - buttons[X]);
        addJointCommand(joint, "Joint_5", buttons[Y] - buttons[A]);
        return false;
    }

    twist->twist.linear.z = axes[RIGHT_STICK_Y];
    twist->twist.linear.y = axes[RIGHT_STICK_X];
    twist->twist.linear.x = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS[RIGHT_TRIGGER]) +
                             0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS[LEFT_TRIGGER]);
    twist->twist.angular.y = axes[LEFT_STICK_Y];
    twist->twist.angular.x = axes[LEFT_STICK_X];
    twist->twist.angular.z = buttons[RIGHT_BUMPER] - buttons[LEFT_BUMPER];

    return true;
}

void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons) {
    if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
        frame_name = BASE_FRAME_ID;
    else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
        frame_name = EEF_FRAME_ID;
}

namespace my_robot_teleop {
class JoyToServoPub : public rclcpp::Node {
  public:
    JoyToServoPub(const rclcpp::NodeOptions& options)
      : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID), is_manual_control_active_(true) {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { joyCB(msg); });

        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
        joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());

        servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        servo_start_client_->wait_for_service(std::chrono::seconds(1));
        servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }

    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
        static std::unordered_map<int, std::chrono::steady_clock::time_point> last_pressed;
        static std::unordered_map<int, bool> last_state;
        std::chrono::milliseconds debounce_duration(200);

        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            auto now = std::chrono::steady_clock::now();
            if (msg->buttons[i] != last_state[i] && now - last_pressed[i] > debounce_duration) {
                last_pressed[i] = now;
                last_state[i] = msg->buttons[i];

                if (i == HOME && msg->buttons[i]) {
                    is_manual_control_active_ = !is_manual_control_active_;
                    RCLCPP_INFO(this->get_logger(), "Manual control %s", is_manual_control_active_ ? "activated" : "deactivated");
                }
            }
        }

        if (is_manual_control_active_) {
            auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

            updateCmdFrame(frame_to_publish_, msg->buttons);

            if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg, this->get_logger())) {
                twist_msg->header.frame_id = frame_to_publish_;
                twist_msg->header.stamp = this->now();
                twist_pub_->publish(std::move(twist_msg));
            } else {
                joint_msg->header.stamp = this->now();
                joint_msg->header.frame_id = "Joint_2";
                joint_pub_->publish(std::move(joint_msg));
            }
        }
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
    
    std::string frame_to_publish_;
    bool is_manual_control_active_;
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(my_robot_teleop::JoyToServoPub)
