#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import os
import cv2
import numpy as np
from pathlib import Path
import time
import re

class DirectoryImagePublisher(Node):
    def __init__(self, image_directory):
        super().__init__('directory_image_publisher')
        
        # Путь к директории с изображениями
        self.image_directory = Path(image_directory)
        
        # Создаем паблишер для топика panel_image
        self.image_publisher = self.create_publisher(Image, 'panel_image', 10)
        
        # Параметры изображения для LED-матрицы
        self.led_width = 25
        self.led_height = 5
        
        # Словарь для хранения групп изображений
        self.image_groups = {}
        
        # Таймер для периодической публикации
        self.timer = None
        
        self.get_logger().info(f'Initializing directory image publisher for: {self.image_directory}')
        
        # Загружаем изображения
        self.load_images()
        
        if not self.image_groups:
            self.get_logger().error(f'No images found in directory: {self.image_directory}')
            return
        
        self.get_logger().info(f'Loaded {sum(len(v) for v in self.image_groups.values())} images in {len(self.image_groups)} groups')
        
        # Запускаем публикацию для всех групп
        self.start_publishing()
    
    def extract_group_and_number(self, filename):
        """Извлекает группу и номер кадра из имени файла"""
        # Поддерживаем форматы: group_N.png, group_N.jpg, group_N.jpeg
        pattern = r'([a-zA-Z_]+)_(\d+)\.(png|jpg|jpeg)$'
        match = re.match(pattern, filename, re.IGNORECASE)
        
        if match:
            group = match.group(1).lower()  # Группа (например, main_display, left_eye)
            number = int(match.group(2))    # Номер кадра
            return group, number
        else:
            return None, None
    
    def load_images(self):
        """Загружаем и сортируем изображения из директории по группам"""
        try:
            # Находим все изображения
            image_patterns = ['*.png', '*.jpg', '*.jpeg']
            all_files = []
            for pattern in image_patterns:
                all_files.extend(self.image_directory.glob(pattern))
            
            # Группируем изображения по префиксу
            for file_path in all_files:
                group, number = self.extract_group_and_number(file_path.name)
                
                if group is not None:
                    if group not in self.image_groups:
                        self.image_groups[group] = []
                    
                    self.image_groups[group].append({
                        'path': file_path,
                        'number': number
                    })
            
            # Сортируем изображения в каждой группе по номеру
            for group in self.image_groups:
                self.image_groups[group].sort(key=lambda x: x['number'])
                self.get_logger().info(f'Group "{group}": {len(self.image_groups[group])} images')
                
        except Exception as e:
            self.get_logger().error(f'Error loading images: {e}')
    
    def publish_image(self, image_path, frame_id):
        """Публикуем изображение"""
        try:
            # Загружаем изображение с помощью OpenCV
            image = cv2.imread(str(image_path))
            if image is None:
                self.get_logger().error(f'Failed to load image: {image_path}')
                return False
            
            # Конвертируем BGR в RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Изменяем размер до размеров LED-матрицы
            image = cv2.resize(image, (self.led_width, self.led_height), interpolation=cv2.INTER_AREA)
            
            # Создаем сообщение Image
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id  # Используем имя группы как frame_id
            
            msg.height = self.led_height
            msg.width = self.led_width
            msg.encoding = 'rgb8'
            msg.is_bigendian = 0
            msg.step = self.led_width * 3
            msg.data = image.tobytes()
            
            # Публикуем
            self.image_publisher.publish(msg)
            self.get_logger().info(f'Published image: {image_path.name} to frame: {frame_id}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image {image_path}: {e}')
            return False
    
    def start_publishing(self):
        """Запускаем публикацию всех групп"""
        if not self.image_groups:
            self.get_logger().error('No images to publish')
            return
        
        # Создаем таймер для периодической публикации
        # Публикуем кадры с частотой 10 Гц (каждые 0.1 секунды)
        self.timer = self.create_timer(0.1, self.publish_next_frames)
        
        self.get_logger().info('Started publishing images in loop')
    
    def publish_next_frames(self):
        """Публикуем следующий кадр для каждой группы"""
        for group_name, images in self.image_groups.items():
            if images:
                # Выбираем следующий кадр (в бесконечном цикле)
                # Можно использовать глобальный счетчик или индекс в группе
                current_time = self.get_clock().now().nanoseconds
                frame_index = (current_time // 100000000) % len(images)  # Каждые 0.1 секунды меняем кадр
                
                image_info = images[frame_index]
                self.publish_image(image_info['path'], group_name)
    
    def get_status(self):
        """Получаем информацию о текущем состоянии"""
        if not self.image_groups:
            return "No images loaded"
        
        status = f"Publishing {sum(len(v) for v in self.image_groups.values())} images in {len(self.image_groups)} groups:\n"
        for group_name, images in self.image_groups.items():
            status += f"  - Group '{group_name}': {len(images)} images\n"
            # Показываем первые и последние имена файлов
            if len(images) > 5:
                status += f"    {images[0]['path'].name}, {images[1]['path'].name}, ..., {images[-2]['path'].name}, {images[-1]['path'].name}\n"
            else:
                for img in images:
                    status += f"    {img['path'].name}\n"
        return status

def main(args=None):
    rclpy.init(args=args)
    
    # Путь к директории с изображениями
    # Замените на ваш путь
    image_directory = "test_frames"  # Например: "/home/user/animation_frames"
    
    publisher = DirectoryImagePublisher(image_directory)
    
    try:
        # Выводим статус
        print(publisher.get_status())
        
        # Запускаем цикл обработки
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        pass
    finally:
        if publisher.timer:
            publisher.timer.cancel()
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()