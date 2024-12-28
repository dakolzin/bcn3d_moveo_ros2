import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import logging

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Подписка на joint_states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/manipulator_joint_trajectory_controller/joint_trajectory',
            10)

        # Определение порядка суставов для синхронизации
        self.joint_order = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']

    def listener_callback(self, msg):
        # Проверка порядка суставов и перестановка, если необходимо
        ordered_positions = self.reorder_joint_positions(msg.name, msg.position)

        # Создание JointTrajectory
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = self.joint_order

        # Заполнение точки траектории
        point = JointTrajectoryPoint()
        point.positions = ordered_positions
        point.time_from_start.sec = 1  

        joint_trajectory.points = [point]
        self.publisher.publish(joint_trajectory)

        self.get_logger().info(f'Published joint trajectory: {joint_trajectory}')

    def reorder_joint_positions(self, names, positions):
        ordered_positions = [0] * len(self.joint_order)
        for name, position in zip(names, positions):
            if name in self.joint_order:
                index = self.joint_order.index(name)
                ordered_positions[index] = position
        return ordered_positions

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except Exception as e:
        joint_state_publisher.get_logger().error(f'Exception in JointStatePublisher: {e}')
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
