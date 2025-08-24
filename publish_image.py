#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/panel_image', 10)
        
        # Создаем изображение 25x5 с градиентом
        img = np.zeros((5, 25, 3), dtype=np.uint8)
        for x in range(25):
            intensity = int(255 * x / 24)
            img[:, x, 0] = intensity  # Красный канал
        
        # Создаем сообщение
        msg = Image()
        msg.header.frame_id = 'main_display'
        msg.height = 5
        msg.width = 25
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = 75
        msg.data = img.tobytes()
        
        # Публикуем сообщение
        self.publisher_.publish(msg)
        self.get_logger().info('Image published successfully!')
        
        # Немедленно уничтожаем ноду
        self.destroy_node()

def main(args=None):
    # Инициализируем rclpy
    rclpy.init(args=args)
    
    try:
        # Создаем и запускаем ноду
        image_publisher = ImagePublisher()
        
        # Даем время на публикацию сообщения
        import time
        time.sleep(0.1)
        
    finally:
        # Корректно завершаем rclpy
        rclpy.shutdown()

if __name__ == '__main__':
    main()