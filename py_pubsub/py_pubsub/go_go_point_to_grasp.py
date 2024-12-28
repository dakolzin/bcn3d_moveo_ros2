#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_transformations
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Quaternion

'''
Этот код представляет собой ROS 2 узел, который вычисляет и публикует точку захвата для роботизированного манипулятора, используя информацию о местоположении двух ArUco маркеров. 
Узел называется GrabPointCalculator и выполняет следующие функции.

Узел создаёт буфер и слушатель для трансформаций (tf2_ros), а также транслятор для публикации трансформаций. Также инициализируется публикатор для визуализации маркеров в Rviz.

Регулярно (каждую секунду), узел пытается вычислить точку захвата, основываясь на текущем местоположении двух ArUco маркеров относительно начальной точки манипулятора (Link_0). Это делается путём получения трансформаций для каждого из маркеров, коррекции их положения на основе заданного смещения по оси Z, и последующего вычисления средней точки между скорректированными положениями маркеров.

Узел публикует преобразования для вычисленной точки захвата (object_point) и дополнительной точки подготовки захвата (pre_grab_point), которая находится на фиксированном расстоянии от точки захвата и используется для позиционирования манипулятора перед собственно захватом объекта.

Узел регулярно публикует маркеры для визуализации в Rviz, что помогает в отладке и демонстрации процесса определения точек захвата и подготовки захвата.
'''

class GrabPointCalculator(Node):
    def __init__(self):
        super().__init__('object_point_calculator')
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.calculate_and_publish_object_point)
        self.marker_timer = self.create_timer(0.5, self.publish_marker)

    def calculate_and_publish_object_point(self):
        self.get_logger().info("Trying to compute and publish object point...")
        for marker_id in ['aruco_marker_42', 'aruco_marker_58']:
            if not self.tf_buffer.can_transform('Link_0', marker_id, rclpy.time.Time()):
                self.get_logger().info(f"Transform from 'Link_0' to '{marker_id}' is not yet available.")
                return

        # Получение положения и ориентации маркера 42 и 58
        tvec_42, quat_42 = self.get_transform_vector_and_orientation('Link_0', 'aruco_marker_42')
        tvec_58, quat_58 = self.get_transform_vector_and_orientation('Link_0', 'aruco_marker_58')

        if tvec_42 is not None and tvec_58 is not None:
            distance_to_42 = -0.033  # Смещение вдоль локальной оси Z маркера 42
            distance_to_58 = -0.015  # Смещение вдоль локальной оси Z маркера 58
            
            # Вращаем вектор смещения вдоль локальной оси Z с учетом ориентации маркера 42
            local_z_offset_42 = [0, 0, distance_to_42]
            global_z_offset_42 = self.rotate_vector_by_quaternion(local_z_offset_42, quat_42)
            
            # Вращаем вектор смещения вдоль локальной оси Z с учетом ориентации маркера 58
            local_z_offset_58 = [0, 0, distance_to_58]
            global_z_offset_58 = self.rotate_vector_by_quaternion(local_z_offset_58, quat_58)

            # Применяем глобальное смещение к положениям маркеров
            adjusted_tvec_42 = tvec_42 + global_z_offset_42
            adjusted_tvec_58 = tvec_58 + global_z_offset_58

            # Используем среднюю точку между скорректированными положениями маркеров 42 и 58 как точку захвата
            object_point = (adjusted_tvec_42 + adjusted_tvec_58) / 2

            self.publish_object_point(object_point)
            self.publish_pre_grab_point(object_point)

    def rotate_vector_by_quaternion(self, vector, quaternion):
        q_vec = np.array(vector + [0.0])  # Превращаем вектор в кватернион с w=0
        rotated_vec = tf_transformations.quaternion_multiply(
            tf_transformations.quaternion_multiply(quaternion, q_vec),
            tf_transformations.quaternion_conjugate(quaternion)
        )[:3]  # Игнорируем четвертую компоненту
        return np.array(rotated_vec)

    def get_transform_vector_and_orientation(self, from_frame, to_frame):
        try:
            trans = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            translation = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            orientation = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            return translation, orientation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get transform from '{from_frame}' to '{to_frame}': {str(e)}")
            return None, None

    def publish_object_point(self, object_point):
        '''
        Функция для отрисовки точки центра объекта 
        '''
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'Link_0'
        transform.child_frame_id = 'object_point'
        transform.transform.translation.x = object_point[0]
        transform.transform.translation.y = object_point[1]
        transform.transform.translation.z = object_point[2]
        # Инвертируем ориентацию оси Z
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 1.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 0.0  # Поворот на 180 градусов вокруг оси Y

        self.broadcaster.sendTransform(transform)
        self.get_logger().info("Published object point transform.")


    def publish_pre_grab_point(self, object_point):
        '''
        Функция для отрисовки точки подготовки захватыания 
        '''
        pre_grab_point_distance = -0.08  # Расстояние по оси Z от object_point

        # Здесь мы задаем pre_grab_point относительно object_point, поэтому координаты X и Y будут 0,
        # а Z будет равен pre_grab_point_distance, так как мы смещаемся только по оси Z.
        pre_grab_point = np.array([0, 0, pre_grab_point_distance])

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'object_point'  # Определяем относительно object_point
        transform.child_frame_id = 'pre_grab_point'
        transform.transform.translation.x = pre_grab_point[0]
        transform.transform.translation.y = pre_grab_point[1]
        transform.transform.translation.z = pre_grab_point[2]
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # Без поворота

        self.broadcaster.sendTransform(transform)
        self.get_logger().info("Published pre_grab_point transform.")

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "aruco_marker_58"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = Pose(position=Point(x=0.0, y=0.0, z=-0.015),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        marker.scale.x = 0.08
        marker.scale.y = 0.066
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = GrabPointCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
