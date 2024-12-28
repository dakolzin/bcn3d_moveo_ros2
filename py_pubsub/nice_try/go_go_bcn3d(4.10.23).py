import rclpy
from sensor_msgs.msg import JointState
from bcn3d_moveo_scripts.msg import ArmJointState
from rclpy.node import Node
import serial
import math
import time

class JointStateConverter(Node):
    def __init__(self):
        super().__init__('joint_state_converter')
        
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
        self.ser.flush()
        self.last_sent_data = None
        self.initial_message_logged = False
        self.last_data_received = None
        self.retry_count = 0

        self.declare_parameter("steps_per_revolution", [-64000, -4400, -92000, -800, -7200])
        self.steps_per_revolution = self.get_parameter("steps_per_revolution").value

        self.joint_steps_pub = self.create_publisher(ArmJointState, 'joint_steps', 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        if not self.R():
            self.get_logger().error("Failed to establish handshake with Arduino")
            rclpy.shutdown()

    def R(self):
        for _ in range(3):  
            self.ser.write("R?\n".encode())
            response = self.ser.readline().decode().strip()
            self.get_logger().info(f"Connection")
            if response == "READY":
                self.get_logger().info(f"============== Ready to use ==============")
                return True
            time.sleep(1)  # ждать 1 секунду перед повторной попыткой
        return False

    def joint_state_callback(self, msg):
        #self.get_logger().info(f"Received JointState: {msg}")  # Выводим принятые данные

        arm_steps = ArmJointState()
        arm_steps.position1 = int((msg.position[5]) * self.steps_per_revolution[0] / (2 * math.pi))
        arm_steps.position2 = int((msg.position[2]) * self.steps_per_revolution[1] / (2 * math.pi))
        arm_steps.position3 = int((msg.position[0]) * self.steps_per_revolution[2] / (2 * math.pi))
        arm_steps.position4 = int((msg.position[1]) * self.steps_per_revolution[3] / (2 * math.pi))
        arm_steps.position5 = int((msg.position[3]) * self.steps_per_revolution[4] / (2 * math.pi))

        self.joint_steps_pub.publish(arm_steps)

        # Отправить данные на Arduino через последовательное соединение
        data_to_send = f"{arm_steps.position1},{arm_steps.position2},{arm_steps.position3},{arm_steps.position4},{arm_steps.position5}\n"
        
        if not self.initial_message_logged:
            self.last_sent_data = data_to_send
            send_data = False
            self.retry_count += 1
            self.initial_message_logged = True
            self.get_logger().info(f"Initial message: {data_to_send}")
            self.get_logger().info(f"###############################################")
        elif data_to_send != self.last_sent_data:
            self.retry_count = 0
            send_data = True
        elif (self.retry_count < 1):
            self.retry_count += 1
            send_data = True
        else:
            send_data = False

        if send_data:
            self.last_sent_data = data_to_send
            self.ser.write(data_to_send.encode())
            self.get_logger().info(f"Sending data to Arduino: {data_to_send.strip()}")  
            self.get_logger().info(f"===============================================")
            time.sleep(0.09)
            while self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)
            data_received = self.ser.readline().decode().strip()  # Чтение данных от Arduino
            self.get_logger().info(f"Received from Arduino: {data_received}")  
            self.get_logger().info(f"###############################################")
            self.last_data_received = data_received
        else:
            # Логирование того, что данные не были отправлены из-за совпадения с предыдущими
            self.get_logger().info(f"Data not sent (matches previous): {data_to_send}")
            self.get_logger().info(f"###############################################")

def main(args=None):
    rclpy.init(args=args)
    joint_state_converter = JointStateConverter()
    rclpy.spin(joint_state_converter)
    joint_state_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()