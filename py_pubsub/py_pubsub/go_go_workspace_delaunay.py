import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from scipy.spatial import Delaunay
from scipy.spatial import ConvexHull

class SurfacePublisher(Node):
    def __init__(self):
        super().__init__('surface_publisher')
        self.publisher_ = self.create_publisher(Marker, 'surface_outside', 10)
        self.timer = self.create_timer(2.0, self.publish_surface)

    def publish_surface(self):
        data = np.loadtxt('/home/danil/danil/ml/training/third_tom.txt', usecols=(5, 6, 7))
        hull = ConvexHull(data)
        hull_points = data[hull.vertices]
        self.get_logger().info(f'Filtered points count: {len(hull_points)}')
        tri = Delaunay(hull_points[:, :3])
        simplices = tri.simplices

        if len(simplices.flatten()) % 3 != 0:
            points_to_remove = len(simplices.flatten()) % 3
            simplices = simplices[:-points_to_remove//3]

        # Создание Marker сообщения для публикации меша
        marker = Marker()
        marker.header.frame_id = "Link_0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "surface_outside"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.01
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        # Добавление точек треугольников в маркер
        for simplex in simplices:
            for i in simplex:
                p = Point(x=hull_points[i, 0], y=hull_points[i, 1], z=hull_points[i, 2])
                marker.points.append(p)

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
