import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial

class JointStateFilter(Node):
    def __init__(self):
        super().__init__('joint_state_filter')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.publisher = self.create_publisher(JointState, 'fsr_joint_states', 10)

        try:
            self.ser_uno = serial.Serial('/dev/ttyACM1', 115200, timeout=2)
            self.ser_uno.flush()
            self.get_logger().info("Successfully connected to Arduino Uno")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino Uno: {e}")

        self.FSR_THRESHOLD = 5
        self.last_grip_position = None

    def joint_state_callback(self, msg):
        fsr1, fsr2 = self.read_from_arduino_uno()

        # Проверяем, не пытаемся ли мы открыть схват (уменьшить позицию)
        trying_to_open = self.last_grip_position is not None and msg.position[3] < self.last_grip_position

        if fsr1 is not None and fsr2 is not None:
            if fsr1 > self.FSR_THRESHOLD or fsr2 > self.FSR_THRESHOLD:
                if not trying_to_open:  # Блокируем закрытие, если порог превышен
                    msg.position[3] = self.last_grip_position
            self.last_grip_position = msg.position[3]
        else:
            if self.last_grip_position is not None:
                #self.get_logger().info("Using last valid grip position due to invalid FSR data")
                msg.position[3] = self.last_grip_position

        self.publisher.publish(msg)
        #self.log_grip_position(msg.position[3])


    def log_grip_position(self, grip_position):
        self.get_logger().info(f"Grip position: {grip_position:.2f}")

    def read_from_arduino_uno(self):
        while self.ser_uno.in_waiting > 0:
            try:
                line = self.ser_uno.readline().decode('utf-8').strip()
                line = line.replace('\r', '').replace('\n', '')
                if ',' in line:
                    fsr1_value, fsr2_value = [int(val) for val in line.split(',')]
                    self.get_logger().info(f"Read FSR values: FSR1 = {fsr1_value}, FSR2 = {fsr2_value}")
                    return fsr1_value, fsr2_value
            except (ValueError, UnicodeDecodeError) as e:
                self.get_logger().error(f"Error processing FSR data: {e}")
        return None, None


def main(args=None):
    rclpy.init(args=args)
    joint_state_filter = JointStateFilter()
    rclpy.spin(joint_state_filter)
    joint_state_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
