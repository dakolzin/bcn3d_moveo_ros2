import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
import socket
import threading
import math
import time

'''
Этот код — часть системы на основе ROS 2 (Robot Operating System), предназначенный для считывания состояний суставов манипулятора (например, роботизированной руки), 
вычисления соответствующих шагов для двигателей и отправки этих данных на микроконтроллер (Arduino Mega) через TCP/IP соединение. 
'''

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.subscription = self.create_subscription(
            JointState,
            'fsr_joint_states',
            self.listener_callback,
            10,
            callback_group=ReentrantCallbackGroup())
        self.subscription
        self.server_address = ('192.168.1.177', 8001)
        self.latest_positions = None
        self.last_sent_positions = None  
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(self.server_address) 
        self.get_logger().info("Connected to Arduino")

        timer_period = 0.04  # seconds (20 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.steps_per_revolution = [64000, -4400, 92000, 800, 7200]

    def __del__(self):
        self.sock.close()

    def listener_callback(self, msg):
        with self.lock:
            self.latest_positions = self.calculate_arm_steps(msg.position[:7])
    
    def calculate_arm_steps(self, positions):
        steps = self.steps_per_revolution
        pi_factor = 2 * math.pi
        arm_steps = [
            int(positions[6] * steps[0] / pi_factor),
            int(positions[1] * steps[1] / pi_factor),
            int(positions[0] * steps[2] / pi_factor),
            int(positions[3] * steps[3] / pi_factor),
            int(positions[2] * steps[4] / pi_factor),
            int(positions[5] * 180 / math.pi)
        ]
        return arm_steps

    def timer_callback(self):
        with self.lock:
            if self.latest_positions is not None and self.latest_positions != self.last_sent_positions:
                try:
                    self.sock.settimeout(5.0)  # Установка таймаута для операций с сокетом
                    data = ','.join([str(pos) for pos in self.latest_positions]) + '\n'
                    self.sock.sendall(data.encode())
                    self.get_logger().info(f"Sent: {data}")
                    self.last_sent_positions = self.latest_positions

                    response = self.sock.recv(256).decode().strip()
                    self.get_logger().info(f"Received: {response}")
                    self.sock.settimeout(None)  # Сброс таймаута
                except socket.timeout:
                    self.get_logger().error("Socket timeout, trying to reconnect...")
                    self.reconnect()
                except Exception as e:
                    self.get_logger().error(f"Communication error: {e}")
                    self.reconnect()

    def reconnect(self):
        self.get_logger().info("Attempting to reconnect to Arduino...")
        self.sock.close()
        time.sleep(1)  
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(self.server_address)
            self.get_logger().info("Reconnected to Arduino")
        except Exception as e:
            self.get_logger().error(f"Reconnection failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

