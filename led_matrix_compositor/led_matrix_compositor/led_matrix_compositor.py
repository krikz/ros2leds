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
        # Физические панели в порядке их подключения в цепочке SPI:
        # [0-4]: Main Display (5× 5×5 = 125 LEDs)
        # Фары пока физически не подключены
        # Итого: 125 LEDs
        self.physical_panels = [
            # Main Display: 5 панелей 5×5
            {'width': 5, 'height': 5, 'snake_connection': True},  # Panel 0
            {'width': 5, 'height': 5, 'snake_connection': True},  # Panel 1
            {'width': 5, 'height': 5, 'snake_connection': True},  # Panel 2
            {'width': 5, 'height': 5, 'snake_connection': True},  # Panel 3
            {'width': 5, 'height': 5, 'snake_connection': True},  # Panel 4
        ]
        
        # Логические группы панелей
        self.logical_groups = [
            # Главный дисплей (5×25)
            {
                'name': 'main_display',
                'physical_indices': [0, 1, 2, 3, 4],
                'arrangement': [5, 1],  # 5 панелей в ряд, 1 ряд
                'flip_x': False,
                'flip_y': True,
                'snake_arrangement': False
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
            snake = self.physical_panels[i].get('snake_connection', False)
            self.get_logger().info(f"  Physical Panel {i}: {width}x{height}, offset={offset}, snake={snake}")
        
        # Создаем и индексируем логические группы
        self.logical_groups_map = {}
        # Отслеживаем неизвестные группы, для которых уже показано предупреждение
        self.warned_unknown_groups = set()
        for group in self.logical_groups:
            name = group['name']
            self.logical_groups_map[name] = group
            
            # Установка значений по умолчанию
            group.setdefault('flip_x', False)
            group.setdefault('flip_y', False)
            group.setdefault('snake_arrangement', False)
            
            arrangement = group['arrangement']
            rows = arrangement[1]
            cols = arrangement[0]
            
            if len(group['physical_indices']) != rows * cols:
                self.get_logger().error(
                    f"Group '{name}' has {len(group['physical_indices'])} physical panels, "
                    f"but arrangement {cols}x{rows} requires {rows * cols} panels"
                )
                raise ValueError(f"Incorrect panel count for group '{name}'")
            
            # Вычисляем размеры
            group_width = 0
            group_height = 0
            
            for r in range(rows):
                row_width = 0
                for c in range(cols):
                    # Учитываем змейку в расположении
                    if group['snake_arrangement'] and r % 2 == 1:
                        c_actual = cols - 1 - c
                    else:
                        c_actual = c
                    idx = group['physical_indices'][r * cols + c_actual]
                    row_width += self.physical_panels[idx]['width']
                group_width = max(group_width, row_width)
            
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
            group_name = msg.header.frame_id
            
            if group_name not in self.logical_groups_map:
                # Показываем предупреждение только один раз для каждой неизвестной группы
                if group_name not in self.warned_unknown_groups:
                    self.get_logger().warn(f"Unknown logical group: '{group_name}'")
                    self.warned_unknown_groups.add(group_name)
                return
            
            group = self.logical_groups_map[group_name]
            
            expected_width = group['width']
            expected_height = group['height']
            expected_size = expected_width * expected_height * 3
            
            if msg.width != expected_width or msg.height != expected_height:
                self.get_logger().warn(
                    f"Image size {msg.width}x{msg.height} doesn't match group '{group_name}' size {expected_width}x{expected_height}"
                )
                return
            
            if msg.encoding != 'rgb8':
                self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}. Expected 'rgb8'")
                return
            
            if len(msg.data) != expected_size:
                self.get_logger().warn(f"Image data size {len(msg.data)} doesn't match expected size {expected_size}")
                return
            
            self._handle_logical_group(group, msg.data)
        
        except Exception as e:
            self.get_logger().error(f"Error processing panel image: {str(e)}")
    
    def _handle_logical_group(self, group, image_data):
        """Обрабатывает изображение для логической группы панелей с учётом flip и snake"""
        arrangement = group['arrangement']
        cols = arrangement[0]
        rows = arrangement[1]
        physical_indices = group['physical_indices']
        flip_x = group['flip_x']
        flip_y = group['flip_y']
        snake_arrangement = group['snake_arrangement']
        
        for r in range(rows):
            # Определяем порядок колонок в строке (змейка)
            if snake_arrangement and r % 2 == 1:
                c_range = range(cols - 1, -1, -1)
            else:
                c_range = range(cols)
            
            for c in c_range:
                group_index = r * cols + c
                physical_index = physical_indices[group_index]
                
                panel = self.physical_panels[physical_index]
                panel_width = panel['width']
                panel_height = panel['height']
                snake_connection = panel.get('snake_connection', False)
                
                # Вычисляем смещение в логической группе
                x_offset = 0
                for i in range(c):
                    i_actual = i if not (snake_arrangement and r % 2 == 1) else (cols - 1 - i)
                    idx = physical_indices[r * cols + i_actual]
                    x_offset += self.physical_panels[idx]['width']
                
                y_offset = 0
                for i in range(r):
                    for j in range(cols):
                        idx = physical_indices[i * cols + j]
                        y_offset += self.physical_panels[idx]['height']
                    break  # высота одинаковая по строке
                
                # Создаем буфер для этой панели
                panel_buffer = bytearray(panel_width * panel_height * 3)
                
                # Копируем данные из общего изображения в буфер панели
                for y in range(panel_height):
                    py = y if not flip_y else (panel_height - 1 - y)
                    src_y = y_offset + py
                    for x in range(panel_width):
                        px = x if not flip_x else (panel_width - 1 - x)
                        src_x = x_offset + px
                        src_idx = (src_y * group['width'] + src_x) * 3
                        dst_idx = (y * panel_width + x) * 3
                        panel_buffer[dst_idx:dst_idx+3] = image_data[src_idx:src_idx+3]
                
                # Учитываем змейку в физической панели
                if snake_connection:
                    temp_buffer = bytearray(panel_width * panel_height * 3)
                    for y in range(panel_height):
                        row_start = y * panel_width * 3
                        pixel_row = panel_buffer[row_start:row_start + panel_width * 3]
                        
                        # Разбиваем на пиксели
                        pixels = [pixel_row[i:i+3] for i in range(0, len(pixel_row), 3)]
                        
                        # Переворачиваем строку пикселей для нечётных строк
                        if y % 2 == 1:
                            pixels = pixels[::-1]
                        
                        # Собираем обратно
                        flattened = bytearray()
                        for pixel in pixels:
                            flattened.extend(pixel)
                        
                        temp_buffer[row_start:row_start + panel_width * 3] = flattened
                    
                    panel_buffer = temp_buffer
                
                # Обновляем общий буфер
                panel_offset = self.physical_offsets[physical_index] * 3
                self.buffer[panel_offset:panel_offset + len(panel_buffer)] = panel_buffer
        
        self.publish_buffer()
    
    def publish_buffer(self):
        """Публикует общий буфер в топик драйвера"""
        msg = Int8MultiArray()
        
        try:
            processed_data = []
            for value in self.buffer:
                val = int(value)
                clamped_value = val - 128
                processed_data.append(clamped_value)
            
            msg.data = processed_data
            self.output_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing buffer: {e}")
            self.get_logger().error(f"Buffer sample: {list(self.buffer[:10])}")
    
    def clear_group(self, group_name):
        """Очищает логическую группу (заливает черным)"""
        if group_name not in self.logical_groups_map:
            # Показываем предупреждение только один раз для каждой неизвестной группы
            if group_name not in self.warned_unknown_groups:
                self.get_logger().warn(f"Unknown logical group: '{group_name}'")
                self.warned_unknown_groups.add(group_name)
            return
        
        group = self.logical_groups_map[group_name]
        width = group['width']
        height = group['height']
        size = width * height * 3
        
        black_image = bytearray(size)
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