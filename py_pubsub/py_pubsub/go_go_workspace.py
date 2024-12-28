import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial import ConvexHull
import struct

'''
Этот код представляет собой Python скрипт, использующий ROS 2 для публикации облака точек рабочей зоны в топик ROS. 
'''

class WorkspacePublisher(Node):

    def __init__(self):
        super().__init__('workspace_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'workspace_points', 10)
        self.timer = self.create_timer(2.0, self.publish_workspace_points)

    def publish_workspace_points(self):
        # Чтение данных из файла
        data = np.loadtxt('/home/danil/danil/ml/training/third_tom.txt', usecols=(5, 6, 7))

        # Вычисление выпуклой оболочки
        hull = ConvexHull(data)

        # Получение вершин выпуклой оболочки
        hull_points = data[hull.vertices]

        self.get_logger().info(f'Filtered points count: {len(hull_points)}')

        # Формирование PointCloud2 сообщения
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'Link_0'
        
        # Создание fields для PointCloud2, включая RGB
        fields = [pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1)]
        
        # Преобразование точек и RGB в список
        points_with_rgb = []
        for point in hull_points:
            r, g, b = 255, 0, 0  # Красный цвет
            alpha = 128  # Устанавливаем прозрачность на 50%
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, alpha))[0]  # BGR формат + альфа-канал
            points_with_rgb.append((point[0], point[1], point[2], rgb))


        cloud = pc2.create_cloud(header, fields, points_with_rgb)

        self.publisher_.publish(cloud)
        self.get_logger().info('Publishing workspace points')


def main(args=None):
    rclpy.init(args=args)
    node = WorkspacePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
