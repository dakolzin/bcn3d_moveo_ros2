import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial import Delaunay
from sklearn.neighbors import NearestNeighbors
import struct

class SurfacePublisher(Node):
    def __init__(self):
        super().__init__('surface_publisher')
        self.publisher_ = self.create_publisher(Marker, 'surface', 10)
        self.timer = self.create_timer(2.0, self.publish_surface)

    def publish_surface(self):
        # Загрузите данные точек вашего рабочего пространства
        data = np.loadtxt('/home/danil/danil/ml/training/third_tom.txt', usecols=(5, 6, 7))

        # Инициализация модели для поиска ближайших соседей
        nbrs = NearestNeighbors(n_neighbors=2, algorithm='ball_tree').fit(data)

        # Вычисление расстояний и индексов ближайших соседей для каждой точки
        distances, indices = nbrs.kneighbors(data)
        
        # Это значение зависит от конкретных ограничений вашего манипулятора
        threshold_distance = 0.028 # Примерное значение

        # Фильтрация точек, расстояние до которых больше порога
        filtered_points = data[distances[:, 1] > threshold_distance]

        # Применяем триангуляцию Делоне к точкам (используем только X, Y, Z для триангуляции)
        tri = Delaunay(filtered_points[:, :3])

        # Получаем все точки треугольников в виде плоского списка
        all_points = tri.simplices.flatten()

        while len(all_points) % 3 != 0:
            all_points = all_points[:-1]  # Убираем последнюю точку

        # Создание Marker сообщения для публикации меша
        marker = Marker()
        marker.header.frame_id = "Link_0"  # Указываем систему координат для маркера
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "surface"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0  # Необходимо для корректного отображения
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.1  # Прозрачность
        marker.color.r = 0.5  # Цвет меша: полупрозрачный красный
        marker.color.g = 0.0
        marker.color.b = 0.5

        # Теперь, когда у нас есть корректное количество точек, добавляем их в marker
        marker.points = []  # Очищаем список точек на всякий случай
        for idx in all_points:
            p = Point(x=filtered_points[idx, 0], y=filtered_points[idx, 1], z=filtered_points[idx, 2])
            marker.points.append(p)

        # Публикация маркера
        self.publisher_.publish(marker)
        self.get_logger().info('Publishing surface')

        # Публикация маркера
        self.publisher_.publish(marker)
        self.get_logger().info('Publishing surface')

def main(args=None):
    rclpy.init(args=args)
    node = SurfacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
