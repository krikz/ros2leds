#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Image
import numpy as np

class LEDMatrixCompositor(Node):
    def __init__(self):
        super().__init__('led_matrix_compositor')
        
        # Определяем конфигурацию панелей напрямую в коде
        # Физические панели (их реальное подключение)
        self.physical_panels = [
            # 5 панелей 5x5, соединенных в одну линию
            {'width': 5, 'height': 5},
            {'width': 5, 'height': 5},
            {'width': 5, 'height': 5},
            {'width': 5, 'height': 5},
            {'width': 5, 'height': 5}
        ]
        
        # Логические группы панелей
        self.logical_groups = [
            # Объединенная матрица из 5 панелей 5x5 (5x25)
            {
                'name': 'main_display',
                'physical_indices': [0, 1, 2, 3, 4],
                'arrangement': [5, 1]  # 5 панелей в горизонтальном ряду
            }
        ]
        
        # Топики
        self.output_topic = 'led_matrix/data'
        self.input_topic = 'panel_image'
        
        # Проверяем корректность конфигурации
        if not self.physical_panels:
            self.get_logger().error("No physical panels configured.")
            raise ValueError("No physical panels configured")
        
        # Вычисляем смещения для физических панелей в общей цепочке
        self.physical_offsets = []
        self.total_leds = 0
        
        for panel in self.physical_panels:
            width = panel['width']
            height = panel['height']
            leds_in_panel = width * height
            self.physical_offsets.append(self.total_leds)
            self.total_leds += leds_in_panel
        
        self.get_logger().info(f"Configured {len(self.physical_panels)} physical panels with total {self.total_leds} LEDs")
        for i, offset in enumerate(self.physical_offsets):
            width = self.physical_panels[i]['width']
            height = self.physical_panels[i]['height']
            self.get_logger().info(f"  Physical Panel {i}: {width}x{height}, offset={offset}")
        
        # Создаем и индексируем логические группы
        self.logical_groups_map = {}
        for group in self.logical_groups:
            name = group['name']
            self.logical_groups_map[name] = group
            
            # Вычисляем размеры логической группы
            arrangement = group['arrangement']
            rows = arrangement[1]
            cols = arrangement[0]
            
            # Проверяем, что количество панелей соответствует расположению
            if len(group['physical_indices']) != rows * cols:
                self.get_logger().error(
                    f"Group '{name}' has {len(group['physical_indices'])} physical panels, "
                    f"but arrangement {cols}x{rows} requires {rows * cols} panels"
                )
                raise ValueError(f"Incorrect panel count for group '{name}'")
            
            # Вычисляем размеры
            group_width = 0
            group_height = 0
            
            # Вычисляем ширину
            for r in range(rows):
                row_width = 0
                for c in range(cols):
                    idx = group['physical_indices'][r * cols + c]
                    row_width += self.physical_panels[idx]['width']
                group_width = max(group_width, row_width)
            
            # Вычисляем высоту
            for c in range(cols):
                col_height = 0
                for r in range(rows):
                    idx = group['physical_indices'][r * cols + c]
                    col_height += self.physical_panels[idx]['height']
                group_height = max(group_height, col_height)
            
            group['width'] = group_width
            group['height'] = group_height
            
            self.get_logger().info(f"  Logical Group '{name}': {group_width}x{group_height} ({cols}x{rows} arrangement)")
        
        # Создаем общий буфер (RGB для каждого светодиода)
        self.buffer = bytearray(self.total_leds * 3)
        
        # Публикуем в топик драйвера
        self.output_publisher = self.create_publisher(Int8MultiArray, self.output_topic, 10)
        
        # Подписываемся на топик с изображениями для логических групп
        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10)
        
        self.get_logger().info(f"Listening for panel images on topic: {self.input_topic}")
        self.get_logger().info(f"Publishing combined data to topic: {self.output_topic}")
        self.get_logger().info("LED Matrix Compositor is ready")
    
    def image_callback(self, msg):
        """Обрабатывает изображение для логической группы панелей"""
        try:
            # Имя логической группы хранится в header.frame_id
            group_name = msg.header.frame_id
            
            # Проверяем, что группа существует
            if group_name not in self.logical_groups_map:
                self.get_logger().warn(f"Unknown logical group: '{group_name}'")
                return
            
            group = self.logical_groups_map[group_name]
            
            # Проверяем размеры изображения
            expected_width = group['width']
            expected_height = group['height']
            expected_size = expected_width * expected_height * 3
            
            if msg.width != expected_width or msg.height != expected_height:
                self.get_logger().warn(
                    f"Image size {msg.width}x{msg.height} doesn't match group '{group_name}' size {expected_width}x{expected_height}"
                )
                return
            
            # Проверяем формат изображения
            if msg.encoding != 'rgb8':
                self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}. Expected 'rgb8'")
                return
            
            # Проверяем длину данных
            if len(msg.data) != expected_size:
                self.get_logger().warn(f"Image data size {len(msg.data)} doesn't match expected size {expected_size}")
                return
            
            # Обрабатываем изображение для логической группы
            self._handle_logical_group(group, msg.data)
        
        except Exception as e:
            self.get_logger().error(f"Error processing panel image: {str(e)}")
    
    def _handle_logical_group(self, group, image_data):
        """Обрабатывает изображение для логической группы панелей"""
        arrangement = group['arrangement']
        cols = arrangement[0]
        rows = arrangement[1]
        physical_indices = group['physical_indices']
        
        # Обрабатываем каждую физическую панель в группе
        for r in range(rows):
            for c in range(cols):
                # Индекс физической панели в группе
                group_index = r * cols + c
                physical_index = physical_indices[group_index]
                
                # Получаем размеры физической панели
                panel = self.physical_panels[physical_index]
                panel_width = panel['width']
                panel_height = panel['height']
                
                # Вычисляем смещение в логической группе
                x_offset = 0
                for i in range(c):
                    x_offset += self.physical_panels[physical_indices[r * cols + i]]['width']
                
                y_offset = 0
                for i in range(r):
                    y_offset += self.physical_panels[physical_indices[i * cols + c]]['height']
                
                # Создаем буфер для этой панели
                panel_buffer = bytearray(panel_width * panel_height * 3)
                
                # Копируем данные из общего изображения в буфер панели
                for y in range(panel_height):
                    for x in range(panel_width):
                        src_idx = ((y + y_offset) * group['width'] + (x + x_offset)) * 3
                        dst_idx = (y * panel_width + x) * 3
                        panel_buffer[dst_idx:dst_idx+3] = image_data[src_idx:src_idx+3]
                
                # Обновляем общий буфер
                panel_offset = self.physical_offsets[physical_index] * 3
                self.buffer[panel_offset:panel_offset + len(panel_buffer)] = panel_buffer
        
        # Публикуем обновленный буфер
        self.publish_buffer()
    
    def publish_buffer(self):
        """Публикует общий буфер в топик драйвера"""
        msg = Int8MultiArray()
        
        try:
            # Преобразуем буфер в список целых чисел в диапазоне [-128, 127]
            # Если значение > 127, преобразуем в диапазон [0, 127]
            processed_data = []
            for value in self.buffer:
                # Преобразуем в int, если это байт
                if isinstance(value, (bytes, bytearray)):
                    val = int.from_bytes(value, 'big')
                else:
                    val = int(value)
                
                # Ограничиваем диапазон [-128, 127]
                # Для LED матриц используем [0, 127] (яркость не может быть отрицательной)
                clamped_value = max(0, min(127, val))
                processed_data.append(clamped_value)
            
            msg.data = processed_data
            self.output_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing buffer: {e}")
            self.get_logger().error(f"Buffer sample: {list(self.buffer[:10])}")
    
    def clear_group(self, group_name):
        """Очищает логическую группу (заливает черным)"""
        if group_name not in self.logical_groups_map:
            self.get_logger().warn(f"Unknown logical group: '{group_name}'")
            return
        
        group = self.logical_groups_map[group_name]
        width = group['width']
        height = group['height']
        size = width * height * 3
        
        # Создаем черное изображение
        black_image = bytearray(size)
        
        # Отправляем изображение в группу
        self._handle_logical_group(group, black_image)
    
    def clear_all(self):
        """Очищает все логические группы (заливает черным)"""
        for group in self.logical_groups:
            self.clear_group(group['name'])


def main(args=None):
    rclpy.init(args=args)
    led_matrix_compositor = LEDMatrixCompositor()
    
    try:
        rclpy.spin(led_matrix_compositor)
    except KeyboardInterrupt:
        pass
    finally:
        led_matrix_compositor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()