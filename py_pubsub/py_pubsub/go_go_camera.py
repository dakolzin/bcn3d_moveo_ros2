#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
import math
import sys  

'''
Этот скрипт Python является узлом ROS 2, предназначенным для публикации трансформаций (TF) относительно положения QR-кода и камеры в системе координат робота. 
Узел называется CameraTFPublisher и регулярно (каждые 0.1 секунды) публикует две трансформации: 
одну от Link_0 к QR_code и другую от QR_code к Camera_frame.
'''

class CameraTFPublisher(Node):
    def __init__(self):
        super().__init__('camera_tf_publisher')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_tfs)

    def publish_tfs(self):
        self.publish_qr_tf()
        self.publish_camera_tf()

    def publish_qr_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'Link_0'
        t.child_frame_id = 'QR_code'
        t.transform.translation.x = -0.29
        t.transform.translation.y = 0.163
        t.transform.translation.z = -0.058

        roll = 0
        pitch = 0
        yaw = 0
        q = self.euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation = q

        self.broadcaster.sendTransform(t)

    def publish_camera_tf(self):
        t = TransformStamped()
        x = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'QR_code'
        t.child_frame_id = 'Camera_frame'
        t.transform.translation.x = 0.36202859   
        t.transform.translation.y = 0.07030083  
        t.transform.translation.z = 0.18097959
  
        q = Quaternion()
        q.x = 0.47258011
        q.y = 0.6669814
        q.z = -0.46384182
        q.w = -0.34154738
        t.transform.rotation = q

        self.broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    camera_tf_publisher = CameraTFPublisher()

    try:
        rclpy.spin(camera_tf_publisher)
    except KeyboardInterrupt:
        print('Camera TF publisher stopped cleanly')
    except Exception as e:  
        print('Exception in Camera TF publisher:', e, file=sys.stderr)
    finally:
        camera_tf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

