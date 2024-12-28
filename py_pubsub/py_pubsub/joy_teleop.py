import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ModifiedJoyTeleop(Node):

    def __init__(self):
        super().__init__('joy_teleop')
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/command', 10)
        self.joint_names = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]
        self.current_joint_positions = [0.0]*5
        self.position_step = 0.05  # Define a step size for position control

    def joint_state_callback(self, joint_state_msg):
        for name, position in zip(joint_state_msg.name, joint_state_msg.position):
            if name in self.joint_names:
                index = self.joint_names.index(name)
                self.current_joint_positions[index] = position

    def joy_callback(self, joy_msg):
        # Log joystick input
        # self.get_logger().info(f"Joystick Input: Axes: {joy_msg.axes}, Buttons: {joy_msg.buttons}")
        
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        joint_trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        
        # Control joints based on joystick input (position control)
        position_changes = [
            joy_msg.axes[0] * self.position_step,  # Joint_1
            joy_msg.axes[1] * self.position_step,  # Joint_2
            joy_msg.axes[3] * self.position_step,  # Joint_3
            joy_msg.axes[4] * self.position_step,  # Joint_4
            1.0 * self.position_step if joy_msg.buttons[2] else (-1.0 * self.position_step if joy_msg.buttons[0] else 0.0)  # Joint_5
        ]
        
        new_positions = [curr + change for curr, change in zip(self.current_joint_positions, position_changes)]
        point.positions = new_positions
        point.time_from_start.sec = 1  # Duration to execute the command
        
        joint_trajectory.points = [point]
        
        # Log the command being sent to the manipulator
        # self.get_logger().info(f"Sending command to manipulator: {point.positions}")
        
        self.joint_trajectory_publisher.publish(joint_trajectory)

def main(args=None):
    rclpy.init(args=args)
    joy_teleop_node = ModifiedJoyTeleop()
    rclpy.spin(joy_teleop_node)
    joy_teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
