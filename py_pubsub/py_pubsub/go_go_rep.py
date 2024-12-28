import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
import socket
import threading

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
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect(self.server_address)  # Открытие соединения
        self.get_logger().info("Connected to Arduino")

        # Создание таймера
        timer_period = 0.1  # seconds (20Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __del__(self):
        self.sock.close()  # Закрытие соединения при уничтожении объекта

    def listener_callback(self, msg):
        with self.lock:
            self.latest_positions = msg.position[:6]

    def timer_callback(self):
        with self.lock:
            if self.latest_positions is not None:
                try:
                    positions = ['{:.4f}'.format(pos) for pos in self.latest_positions]
                    data = ','.join(positions) + '\n'
                    self.sock.sendall(data.encode())
                    self.get_logger().info(f"Sent: {data}")

                    response = self.sock.recv(256).decode().strip()
                    self.get_logger().info(f"Received: {response}")
                except Exception as e:
                    self.get_logger().error(f"Error: {e}")
                    # Попытка переподключения в случае ошибки
                    self.sock.close()
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.connect(self.server_address)
                    self.get_logger().info("Reconnected to Arduino")

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)  # Вращение ноды (ожидание обратного вызова)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

