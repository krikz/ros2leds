#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import math
import time

class PlasmaPublisher(Node):
    def __init__(self):
        super().__init__('plasma_publisher')
        self.publisher_ = self.create_publisher(Image, '/panel_image', 10)
        
        # Параметры изображения
        self.width = 25
        self.height = 5
        
        # Параметры анимации
        self.time_offset = 0.0
        self.time_step = 0.1
        
        # Таймер для регулярной публикации
        self.timer = self.create_timer(0.1, self.timer_callback)  # Каждые 0.5 секунды
        
        self.get_logger().info('Plasma publisher started, publishing every 0.5 seconds')
    
    def generate_plasma(self):
        """Генерирует эффект плазмы"""
        # Создаем пустое изображение
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Базовые параметры плазмы
        for y in range(self.height):
            for x in range(self.width):
                # Разные синусоидальные волны для создания эффекта плазмы
                value1 = math.sin(x * 0.4 + self.time_offset)
                value2 = math.sin(math.sqrt((x - self.width/2)**2 + (y - self.height/2)**2) * 0.8 + self.time_offset)
                value3 = math.sin(x * 0.2 + y * 0.4 + self.time_offset)
                value4 = math.sin(y * 0.3 + self.time_offset * 0.5)
                
                # Комбинируем волны
                plasma_value = (value1 + value2 + value3 + value4) / 4.0  # Нормализуем
                
                # Преобразуем в диапазон 0-255
                color_value = int((plasma_value + 1) * 127.5)
                
                # Используем разные цветовые схемы для разных координат
                r = int(color_value * (1 + math.sin(self.time_offset * 0.3)) / 2)
                g = int(color_value * (1 + math.cos(self.time_offset * 0.2)) / 2)
                b = int(color_value * (1 + math.sin(self.time_offset * 0.1 + 1.0)) / 2)
                if y == 2:
                    r = g = b = 0
                img[y, x] = [r, g, b]
        
        return img
    
    def timer_callback(self):
        """Вызывается каждые 0.5 секунды для публикации нового кадра"""
        # Генерируем новое изображение плазмы
        img = self.generate_plasma()
        
        # Создаем сообщение ROS 2
        msg = Image()
        msg.header.frame_id = 'main_display'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = img.tobytes()
        
        # Публикуем сообщение
        self.publisher_.publish(msg)
        self.get_logger().info(f'Plasma frame published (time offset: {self.time_offset:.2f})')
        
        # Увеличиваем временной сдвиг для следующего кадра
        self.time_offset += self.time_step

def main(args=None):
    # Инициализируем rclpy
    rclpy.init(args=args)
    
    try:
        # Создаем и запускаем ноду
        plasma_publisher = PlasmaPublisher()
        
        # Запускаем цикл обработки
        rclpy.spin(plasma_publisher)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Корректно завершаем
        plasma_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()