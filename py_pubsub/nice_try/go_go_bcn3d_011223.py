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

        # Initialize serial connection with Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=2)
        self.ser.flush()

        self.last_sent_data = None
        self.initial_message_logged = False
        self.last_data_received = None
        self.retry_count = 0
        self.last_position3 = None

        default_steps = [-64000, -4400, -92000, -800, -7200]
        self.declare_parameter("steps_per_revolution", default_steps)
        self.steps_per_revolution = self.get_parameter("steps_per_revolution").value

        self.joint_steps_pub = self.create_publisher(ArmJointState, 'joint_steps', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'filtered_joint_states', self.joint_state_callback, 10)

        if not self.check_arduino_connection():
            self.get_logger().error("Failed to establish handshake with Arduino")
            rclpy.shutdown()
        
    def check_arduino_connection(self):
        for _ in range(3):
            self.ser.write("R?\n".encode())
            response = self.ser.readline().decode().strip()
            self.get_logger().info(f"Connection")
            if response == "READY":
                self.get_logger().info(f"============== Ready to use ==============")
                return True
            time.sleep(1)  # Wait for 1 second before retry
        return False

    def joint_state_callback(self, msg):
        arm_steps = self.calculate_arm_steps(msg)

        self.joint_steps_pub.publish(arm_steps)

        data_to_send = self.construct_data_to_send(arm_steps)
        self.handle_data_sending(data_to_send)

    def calculate_arm_steps(self, msg):
        arm_steps = ArmJointState()
        positions = msg.position
        steps = self.steps_per_revolution
        pi_factor = 2 * math.pi

        arm_steps.position1 = int(positions[4] * steps[0] / pi_factor)
        arm_steps.position2 = int(positions[1] * steps[1] / pi_factor)
        arm_steps.position3 = int(positions[0] * steps[2] / pi_factor)
        arm_steps.position4 = int(positions[7] * steps[3] / pi_factor)
        arm_steps.position5 = int(positions[2] * steps[4] / pi_factor)
        arm_steps.position6 = int(positions[3] * 180 / math.pi)
        
        """ 
        if self.last_position3 is None or self.last_position3 == positions[3]:
            arm_steps.position6 = int(positions[3] * 180 / math.pi)
        else:
            if positions[3] > self.last_position3:
                arm_steps.position6 = 80
            else:
                arm_steps.position6 = 0
        
        self.last_position3 = positions[3] 
        """

        return arm_steps

    def construct_data_to_send(self, arm_steps):
        return f"{- arm_steps.position1},{arm_steps.position2},{arm_steps.position3},{arm_steps.position4},{arm_steps.position5},{arm_steps.position6}\n"

    def handle_data_sending(self, data_to_send):
        should_send_data = self.should_send_data(data_to_send)
        if should_send_data:
            self.send_data_to_arduino(data_to_send)
        else:
            self.get_logger().info(f"Data not sent (matches previous or exceeded retry limit): {data_to_send.strip()}")
            self.get_logger().info(f"###############################################")

    def should_send_data(self, data_to_send):
        if not self.initial_message_logged:
            self.last_sent_data = data_to_send
            self.retry_count += 4
            self.initial_message_logged = True
            return False
        if data_to_send != self.last_sent_data:
            self.retry_count = 0
            return True
        if self.retry_count < 4 and (not self.last_data_received or self.last_data_received.strip() != self.last_sent_data.strip()):
            time.sleep(1)
            self.retry_count += 1
            return True
        return False

    def send_data_to_arduino(self, data_to_send):
        self.last_sent_data = data_to_send
        #self.get_logger().info(f"Timestamp before sending: {time.time()}")
        self.ser.write(data_to_send.encode())
        self.get_logger().info(f"Sending data to Arduino: {data_to_send.strip()}")
        self.get_logger().info(f"===============================================")
        while self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)
            #self.get_logger().info(f"Timestamp after receiving: {time.time()}")
        self.last_data_received = self.ser.readline().decode().strip()
        self.get_logger().info(f"Received from Arduino: {self.last_data_received}")
        self.get_logger().info(f"###############################################")

def main(args=None):
    rclpy.init(args=args)
    joint_state_converter = JointStateConverter()
    rclpy.spin(joint_state_converter)
    joint_state_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()