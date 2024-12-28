#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
import tf2_ros
import tf_transformations
import math

class MarkerAndPickPointPublisher(Node):
    def __init__(self):
        super().__init__('marker_and_pick_point_publisher')
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker_timer = self.create_timer(0.5, self.publish_marker)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pick_point_timer = self.create_timer(1.0, self.publish_pick_point_tf)

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "aruco_marker_42"
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

    def publish_pick_point_tf(self):
        try:
            trans_aruco = self.tf_buffer.lookup_transform('QR_code', 'aruco_marker_on_object', rclpy.time.Time())

            pick_point_tf = TransformStamped()
            pick_point_tf.header.stamp = self.get_clock().now().to_msg()
            pick_point_tf.header.frame_id = 'QR_code'
            pick_point_tf.child_frame_id = 'Pick_point'
            
            pick_point_tf.transform.translation.x = trans_aruco.transform.translation.x
            pick_point_tf.transform.translation.y = trans_aruco.transform.translation.y
            pick_point_tf.transform.translation.z = trans_aruco.transform.translation.z

            quaternion = tf_transformations.quaternion_from_euler(0, math.pi, 0)
            pick_point_tf.transform.rotation.x = quaternion[0]
            pick_point_tf.transform.rotation.y = quaternion[1]
            pick_point_tf.transform.rotation.z = quaternion[2]
            pick_point_tf.transform.rotation.w = quaternion[3]
            pick_point_tf.transform.translation.z += 0.11  
            self.broadcaster.sendTransform(pick_point_tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Error getting TF transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MarkerAndPickPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Node stopped cleanly')
    except Exception as e:
        print(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
