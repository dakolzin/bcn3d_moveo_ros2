import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from tf2_ros import Buffer, TransformListener
import numpy as np
import joblib
from tensorflow.keras.models import load_model

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'joint_positions_ml', 10)
        self.flag_subscriber = self.create_subscription(Bool, 'joint_positions_done', self.flag_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.model = load_model('/home/danil/ros2_bcn3d_moveo/src/ml/ik_solver_lstm')
        self.scaler_x = joblib.load('/home/danil/ros2_bcn3d_moveo/src/ml/scaler_x_lstm.gz')
        self.scaler_y = joblib.load('/home/danil/ros2_bcn3d_moveo/src/ml/scaler_y_lstm.gz')
        
        self.timer = self.create_timer(2.0, self.timer_callback)  # 2 seconds

    def flag_callback(self, msg):
        flag = msg.data
        if flag:
            self.get_logger().info('Flag received, changing the point')
            # Получаем текущую целевую точку
            input_data = self.get_target_transform()
            if input_data is not None:
                # Модифицируем позицию по оси Z, добавляя 0.06
                modified_input_data = input_data.copy()  # Создаем копию, чтобы не изменять оригинальный список
                #modified_input_data[2] += 0.06  # input_data[2] это Z координата

                # Используем модифицированную точку для предсказания новых значений суставов
                joint_values = self.predict_joint_values(modified_input_data)
                # Публикуем новые значения суставов
                msg = Float64MultiArray()
                msg.data = joint_values.tolist()
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing new joint positions: {msg.data}')

    def get_target_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform('Link_0', 'pre_grab_point', rclpy.time.Time())
            position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            orientation = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            return position + orientation
        except Exception as e:
            self.get_logger().error('Failed to get transform: %s' % str(e))
            return None

    def predict_joint_values(self, input_data):
        input_scaled = self.scaler_x.transform([input_data])
        input_scaled = np.expand_dims(input_scaled, axis=1)
        predictions_scaled = self.model.predict(input_scaled)
        predictions = self.scaler_y.inverse_transform(predictions_scaled)
        return predictions.flatten()

    def timer_callback(self):
        input_data = self.get_target_transform()
        if input_data is not None:
            joint_values = self.predict_joint_values(input_data)
            msg = Float64MultiArray()
            msg.data = joint_values.tolist()
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing joint positions: %s' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

