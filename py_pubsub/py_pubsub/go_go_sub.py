#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import zmq
import numpy as np
import base64
import time
import rclpy
from rclpy.node import Node
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped, Quaternion
from scipy.spatial.transform import Rotation as R
from tf_transformations import quaternion_from_euler

class CreateSocket:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.context = zmq.Context()

    def create_server_socket(self):
        self.server_socket = self.context.socket(zmq.REP)
        self.server_socket.bind("tcp://{}:{}".format(self.ip, self.port))

    def create_client_socket(self):
        self.client_socket = self.context.socket(zmq.REQ)
        self.client_socket.connect("tcp://{}:{}".format(self.ip, self.port))

    def close_socket(self):
        if self.server_socket is not None:
            self.server_socket.close()
        if self.client_socket is not None:
            self.client_socket.close()
        
    def term_context(self):
        self.context.term()

    def pub_multipart_data(self, translation, rotation):
        print ('pub_multipart_data')
        active_socket = self.client_socket if self.client_socket is not None else self.server_socket
        if active_socket is not None:
            active_socket.send_multipart(["pose", translation, rotation])
        else:
            raise Exception("No active socket for sending multipart data")
        time.sleep(0.1)

    def pub_string_data(self, string):
        print ('pub_data')
        active_socket = self.client_socket if self.client_socket is not None else self.server_socket
        if active_socket is not None:
            active_socket.send_string(string)
        else:
            raise Exception("No active socket for sending string data")

    def sub_string_data(self, flags=0):
        print ('sub_data')
        active_socket = self.client_socket if self.client_socket is not None else self.server_socket
        if active_socket is not None:
            return active_socket.recv_string(flags=flags)
        else:
            raise Exception("No active socket for receiving data")
    
    def sub_multipart_data(self, flags=0):
        print ('sub_multipart_data')
        active_socket = self.client_socket if self.client_socket is not None else self.server_socket
        if active_socket is not None:
            return active_socket.recv_multipart(flags=flags)
        else:
            raise Exception("No active socket for receiving multipart data")

def decode_data(caught_translation, caught_rotation):
    decode_translation = base64.b64decode(caught_translation)
    translation = np.frombuffer(decode_translation, dtype=np.float64)

    decode_rotation = base64.b64decode(caught_rotation)
    rotation = np.frombuffer(decode_rotation, dtype=np.float64)

    return translation, rotation

class TF2Broadcaster(Node):
    def __init__(self):
        super().__init__('tf2_broadcaster')
        self.br = tf2_ros.TransformBroadcaster(self)

    def send_transform(self, translation, rotation):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'Camera_frame'
        t.child_frame_id = 'object'
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        q = self.euler_to_quaternion(rotation[0], rotation[1], rotation[2])

        #print(rotation[0], rotation[1], rotation[2])        
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        #print(q)

        self.br.sendTransform(t)

    def timer_callback(self):        
        if self.translation is not None and self.rotation is not None:
            self.send_transform(self.translation, self.rotation)
        else:
            self.get_logger().info('Ожидание данных...')

    def euler_to_quaternion(self, roll, pitch, yaw):        
        return quaternion_from_euler(roll, pitch, yaw)

def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = TF2Broadcaster()

    client_socket = CreateSocket(ip='192.168.1.34', port='5556')
    client_socket.create_client_socket()

    poller = zmq.Poller()
    poller.register(client_socket.client_socket, zmq.POLLIN)

    while True:
        client_socket.pub_string_data("go")

        if poller.poll(1000):  
            message = client_socket.sub_string_data()
            if message == "receiv":
                client_socket.pub_string_data("confirmed")
                break

    client_socket.close_socket()
    client_socket.term_context()

    server_socket = CreateSocket(ip='*', port='5557')
    server_socket.create_server_socket()

    poller = zmq.Poller()
    poller.register(server_socket.server_socket, zmq.POLLIN)
    print("Waiting for object position data")

    translation = None
    rotation = None
   
    received = False
    while not received and rclpy.ok():
        if poller.poll(1000):  
            [address, caught_translation, caught_rotation] = server_socket.sub_multipart_data()
            if address == b"pose":
                translation, rotation = decode_data(caught_translation, caught_rotation)
                print(translation, rotation)
                server_socket.pub_string_data('exit')
                
                confirmation = server_socket.sub_string_data()
                if confirmation == "confirmed":
                    received = True

    # Сохранение полученных данных в объекте tf_broadcaster
    tf_broadcaster.translation = translation
    tf_broadcaster.rotation = rotation

    # Закрытие сокета и контекста после получения данных
    server_socket.close_socket()
    server_socket.term_context()

    # Создание таймера для периодической публикации трансформаций
    timer_period = 0.1  # seconds (10Hz)
    tf_broadcaster.create_timer(timer_period, tf_broadcaster.timer_callback)

    # Зацикливание публикации фреймов
    rclpy.spin(tf_broadcaster)

    # Завершение работы
    tf_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

