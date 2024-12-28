import cv2
import threading
import time
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

'''
Этот код предназначен для обнаружения ArUco маркеров с использованием камеры Intel RealSense, оценки их позиций в пространстве, и публикации этих позиций как преобразований в системе ROS 2. 
Код делится на несколько основных частей.

Инициализация ROS 2 и RealSense камеры: 
создаётся узел ROS 2 ArucoPosePublisher, который будет публиковать преобразования маркеров. Также настраивается поток данных с камеры RealSense.

Обнаружение маркеров и оценка позиции: 
в бесконечном цикле изображения с камеры анализируются на предмет наличия ArUco маркеров. При обнаружении маркеров оценивается их позиция относительно камеры с использованием функций OpenCV для обнаружения маркеров и оценки их позиций.

Сбор данных: 
для каждого маркера (с ID 42 и 58) собираются векторы вращения и векторы смещения. Данные собираются до достижения определённого количества (500) для каждого типа маркера.

Вычисление средних значений: 
после сбора достаточного количества данных для каждого маркера вычисляются средние векторы вращения и смещения.

Публикация трансформаций в ROS 2: 
для каждого маркера на основе средних значений создаются и публикуются трансформации в ROS 2, что позволяет другим узлам в системе использовать эту информацию.

Многопоточная публикация: 
публикация трансформаций для каждого маркера осуществляется в отдельном потоке, что позволяет одновременно транслировать позиции нескольких маркеров.
'''

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('object_pose_publisher')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
    
    def publish_aruco_tf(self, rvec, tvec, marker_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'Camera_frame'  
        t.child_frame_id = f'aruco_marker_{marker_id}'  # Использование marker_id для определения child_frame_id
        
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        
        r = R.from_rotvec(rvec)
        q = r.as_quat()
        
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self.broadcaster.sendTransform(t)

def publish_pose(aruco_pose_publisher, rvec, tvec, marker_id):
    while rclpy.ok():
        aruco_pose_publisher.publish_aruco_tf(rvec, tvec, marker_id)
        time.sleep(1)

def detect_aruco_and_estimate_pose(frame, camera_matrix, dist_coeffs, rvecs_list_58, tvecs_list_58, rvecs_list_42, tvecs_list_42):
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    # Размеры маркеров
    marker_sizes = {42: 0.027, 58: 0.0585}  # Порядок здесь не влияет на логику

    # Проверяем, были ли обнаружены какие-либо маркеры
    if ids is not None:
        for marker_id, marker_size in marker_sizes.items():
            # Проверяем наличие каждого маркера
            if np.any(ids == marker_id):
                # Фильтруем углы и ID для маркера
                target_indices = np.where(ids == marker_id)[0]
                target_corners = [corners[i] for i in target_indices]

                # Оцениваем позу для обнаруженного маркера
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(target_corners, marker_size, camera_matrix, dist_coeffs)
                
                # Отрисовываем оси и маркеры для всех обнаруженных маркеров и собираем данные
                for idx in range(len(rvec)):
                    cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec[idx], tvec[idx], 0.03)
                    cv2.aruco.drawDetectedMarkers(frame, target_corners, ids[target_indices])
                    print(len(rvecs_list_58))
                    print(len(rvecs_list_42))

                    # Собираем данные для маркера 58
                    if marker_id == 58 and len(rvecs_list_58) < 500:
                        rvecs_list_58.append(rvec[idx][0])
                        tvecs_list_58.append(tvec[idx][0])
                    # Добавляем условие для сбора данных маркера 42
                    elif marker_id == 42 and len(rvecs_list_42) < 500:
                        rvecs_list_42.append(rvec[idx][0])
                        tvecs_list_42.append(tvec[idx][0])


def calculate_average(rvecs_list, tvecs_list):
    average_rvec = np.mean(np.array(rvecs_list), axis=0)
    average_tvec = np.mean(np.array(tvecs_list), axis=0)
    return average_rvec, average_tvec

def main():
    rclpy.init()
    aruco_pose_publisher = ArucoPosePublisher()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(config)

    rvecs_list_58, tvecs_list_58 = [], []
    rvecs_list_42, tvecs_list_42 = [], []

    frame_count = 0

    try:
        profile = pipeline.get_active_profile()
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        color_intrinsics = color_profile.get_intrinsics()
        camera_matrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],
                                  [0, color_intrinsics.fy, color_intrinsics.ppy],
                                  [0, 0, 1]], dtype=float)
        dist_coeffs = np.array(color_intrinsics.coeffs, dtype=float)

        collecting = True  # Флаг для продолжения сбора данных
        while collecting:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame or frame_count < 50:
                frame_count += 1
                continue

            color_image = np.asanyarray(color_frame.get_data())
            detect_aruco_and_estimate_pose(color_image, camera_matrix, dist_coeffs, rvecs_list_58, tvecs_list_58, rvecs_list_42, tvecs_list_42)
            
            cv2.imshow('RealSense', color_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Проверка условий для продолжения сбора данных
            if len(rvecs_list_58) >= 500 and len(rvecs_list_42) >= 500:
                collecting = False  # Оба маркера достигли 500 кадров
                print(" ALL ")
            elif len(rvecs_list_58) >= 500 and len(rvecs_list_42) > 0:
                collecting = len(rvecs_list_42) < 500  # Продолжаем сбор для маркера 42
                print(" READY: 58 ")
            elif len(rvecs_list_42) >= 500 and len(rvecs_list_58) > 0:
                collecting = len(rvecs_list_58) < 500  # Продолжаем сбор для маркера 58
                print(" READY: 42 ")
            elif (len(rvecs_list_58) == 0 or len(rvecs_list_42) == 0) and (len(rvecs_list_58) >= 500 or len(rvecs_list_42) >= 500):
                collecting = False  # Прекращаем попытки если один из маркеров обладает количеством кадров = 0 
                print("zero")

        # Publishing logic
        if len(rvecs_list_58) >= 500:
            average_rvec_58, average_tvec_58 = calculate_average(rvecs_list_58, tvecs_list_58)
            print("Average rotation vector for marker 58:", average_rvec_58)
            print("Average translation vector for marker 58:", average_tvec_58)
            publish_thread_58 = threading.Thread(target=publish_pose, args=(aruco_pose_publisher, average_rvec_58, average_tvec_58, 58))
            publish_thread_58.start()

        if len(rvecs_list_42) >= 500:
            average_rvec_42, average_tvec_42 = calculate_average(rvecs_list_42, tvecs_list_42)
            print("Average rotation vector for marker 42:", average_rvec_42)
            print("Average translation vector for marker 42:", average_tvec_42)
            publish_thread_42 = threading.Thread(target=publish_pose, args=(aruco_pose_publisher, average_rvec_42, average_tvec_42, 42))
            publish_thread_42.start()

        if 'publish_thread_58' in locals():
            publish_thread_58.join()
        if 'publish_thread_42' in locals():
            publish_thread_42.join()

    except Exception as e:
        print(e)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()