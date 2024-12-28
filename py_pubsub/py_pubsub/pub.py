import pyrealsense2 as rs
import socket
import hashlib
import numpy as np

# Настройка камеры
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# Начало захвата
pipeline.start(config)

# Настройка сокета
server_ip = '192.168.1.34'  # IP адрес компьютера №2
server_port = 6666         # Порт для отправки данных
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((server_ip, server_port))

try:
    while True:
        # Чтение кадра
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame_hash = hashlib.md5(color_frame.get_data()).hexdigest()
        #print("Отправленный хеш:", frame_hash)
        # Получение данных изображения
        frame_data = color_frame.get_data()
        image = np.asanyarray(frame_data)  
        print(image.shape)  
        print(image)
        
        
        #print(len(frame_data))
        # Отправка данных через сокет
        sock.sendall(frame_data)

finally:
    # Завершение работы
    pipeline.stop()
    sock.close()
