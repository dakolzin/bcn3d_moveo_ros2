#include <memory>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MoveToPreGrabPoint : public rclcpp::Node {
public:
    MoveToPreGrabPoint()
        : Node("move_to_pre_grab_point") {
        // Отложенная инициализация для избежания std::bad_weak_ptr
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Задержка перед инициализацией для уверенности в полном создании узла
            std::bind(&MoveToPreGrabPoint::init, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    void init() {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        auto shared_ptr_node = shared_from_this();

        moveit::planning_interface::MoveGroupInterface move_group(shared_ptr_node, "arm");

        // Получаем трансформацию для pre_grab_point
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("Link_0", "object_point", tf2::TimePointZero, tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = transformStamped.transform.translation.x;
        target_pose.position.y = transformStamped.transform.translation.y;
        target_pose.position.z = transformStamped.transform.translation.z;
        target_pose.orientation = transformStamped.transform.rotation;

        /* std::vector<double> position = {-0.1938, 0.3124, 0.5303};
        std::vector<double> orientation = {-0.2835, 0.1637, 0.1264, 0.9364};

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = position[0];
        target_pose.position.y = position[1];
        target_pose.position.z = position[2];ros2

        target_pose.orientation.x = orientation[0];
        target_pose.orientation.y = orientation[1];
        target_pose.orientation.z = orientation[2];
        target_pose.orientation.w = orientation[3]; */
              
        move_group.setPoseTarget(target_pose);

        //move_group.setGoalPositionTolerance(0.01); // допуск по позиции, м
        move_group.setGoalOrientationTolerance(0.1); // допуск по ориентации, радианы
        // для лучшей работы закомментировать строку выше, с этой строкой есть возможность отсутствия решения ОЗК

        // Установка времени планирования и количества попыток планирования
        move_group.setPlanningTime(10.0); // Время на планирование, секунды
        move_group.setNumPlanningAttempts(10); // Количество попыток планирования

        move_group.setMaxVelocityScalingFactor(0.5); // Устанавливает 50% от максимальной скорости
        move_group.setMaxAccelerationScalingFactor(0.5); // Устанавливает 50% от максимального ускорения

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            move_group.move();
            RCLCPP_INFO(this->get_logger(), "Movement to pre_grab_point succeeded.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Movement to pre_grab_point failed.");
        }

        timer_->cancel(); // Отключаем таймер после инициализации
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToPreGrabPoint>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
