#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from sensor_msgs.msg import Image
import numpy as np
import yaml

class LEDMatrixCompositor(Node):
    def __init__(self):
        super().__init__('led_matrix_compositor')
        
        # Объявляем параметры
        self.declare_parameters(
            namespace='',
            parameters=[
                ('physical_panels', []),
                ('logical_groups', []),
                ('output_topic', 'led_matrix/data'),
                ('input_topic', 'panel_image')
            ]
        )
        
        # Получаем параметры
        physical_panels_param = self.get_parameter('physical_panels')
        logical_groups_param = self.get_parameter('logical_groups')
        output_topic_param = self.get_parameter('output_topic')
        input_topic_param = self.get_parameter('input_topic')
        
        # Выводим отладочную информацию о параметрах
        self.get_logger().info(f"=== Отладочная информация о параметрах ===")
        self.get_logger().info(f"Тип physical_panels: {type(physical_panels_param.value)}")
        self.get_logger().info(f"Значение physical_panels: {physical_panels_param.value}")
        self.get_logger().info(f"Длина physical_panels: {len(physical_panels_param.value) if hasattr(physical_panels_param.value, '__len__') else 'N/A'}")
        
        self.get_logger().info(f"Тип logical_groups: {type(logical_groups_param.value)}")
        self.get_logger().info(f"Значение logical_groups: {logical_groups_param.value}")
        self.get_logger().info(f"Длина logical_groups: {len(logical_groups_param.value) if hasattr(logical_groups_param.value, '__len__') else 'N/A'}")
        
        self.get_logger().info(f"output_topic: {output_topic_param.value}")
        self.get_logger().info(f"input_topic: {input_topic_param.value}")
        self.get_logger().info(f"========================================")
        
        # Получаем параметры
        self.physical_panels = physical_panels_param.value
        self.logical_groups = logical_groups_param.value
        self.output_topic = output_topic_param.value
        self.input_topic = input_topic_param.value
        
        # Проверяем корректность конфигурации
        if not self.physical_panels:
            self.get_logger().error("No physical panels configured. Please specify physical_panels in the configuration file.")
            # Дополнительная отладочная информация
            self.get_logger().error(f"Тип physical_panels: {type(self.physical_panels)}")
            self.get_logger().error(f"Значение physical_panels: {self.physical_panels}")
            self.get_logger().error(f"Длина physical_panels: {len(self.physical_panels) if hasattr(self.physical_panels, '__len__') else 'N/A'}")
            raise ValueError("No physical panels configured")
        
        # Вычисляем смещения для физических панелей в общей цепочке
        self.physical_offsets = []
        self.total_leds = 0
        
        for i, panel in enumerate(self.physical_panels):
            self.get_logger().info(f"Анализ панели {i}: {panel}")
            if not isinstance(panel, dict):
                self.get_logger().error(f"Панель {i} не является словарем! Тип: {type(panel)}")
                continue
                
            if 'width' not in panel or 'height' not in panel:
                self.get_logger().error(f"Панель {i} не содержит width или height: {panel}")
                continue
                
            width = panel['width']
            height = panel['height']
            
            # Попробуем преобразовать в int, если это строка
            try:
                if isinstance(width, str):
                    width = int(width)
                if isinstance(height, str):
                    height = int(height)
            except ValueError as e:
                self.get_logger().error(f"Не удалось преобразовать width/height в int: {e}")
                continue
                
            leds_in_panel = width * height
            self.physical_offsets.append(self.total_leds)
            self.total_leds += leds_in_panel
        
        self.get_logger().info(f"Configured {len(self.physical_panels)} physical panels with total {self.total_leds} LEDs")
        for i, offset in enumerate(self.physical_offsets):
            if i < len(self.physical_panels):
                panel = self.physical_panels[i]
                width = panel['width']
                height = panel['height']
                
                # Преобразуем в int если нужно
                try:
                    if isinstance(width, str):
                        width = int(width)
                    if isinstance(height, str):
                        height = int(height)
                except:
                    width = 0
                    height = 0
                    
                self.get_logger().info(f"  Physical Panel {i}: {width}x{height}, offset={offset}")
        
        # Создаем и индексируем логические группы
        self.logical_groups_map = {}
        for group in self.logical_groups:
            self.get_logger().info(f"Анализ логической группы: {group}")
            if not isinstance(group, dict):
                self.get_logger().error(f"Группа не является словарем! Тип: {type(group)}")
                continue
                
            if 'name' not in group:
                self.get_logger().error(f"Группа не содержит name: {group}")
                continue
                
            name = group['name']
            self.logical_groups_map[name] = group
            
            if 'arrangement' not in group:
                self.get_logger().error(f"Группа {name} не содержит arrangement: {group}")
                continue
                
            if 'physical_indices' not in group:
                self.get_logger().error(f"Группа {name} не содержит physical_indices: {group}")
                continue
                
            # Преобразуем arrangement в список целых чисел
            try:
                arrangement = []
                for val in group['arrangement']:
                    if isinstance(val, str):
                        arrangement.append(int(val))
                    else:
                        arrangement.append(int(val))
                rows = arrangement[1]
                cols = arrangement[0]
            except Exception as e:
                self.get_logger().error(f"Не удалось обработать arrangement для группы {name}: {e}")
                continue
            
            # Преобразуем physical_indices в список целых чисел
            try:
                physical_indices = []
                for val in group['physical_indices']:
                    if isinstance(val, str):
                        physical_indices.append(int(val))
                    else:
                        physical_indices.append(int(val))
            except Exception as e:
                self.get_logger().error(f"Не удалось обработать physical_indices для группы {name}: {e}")
                continue
            
            # Проверяем, что количество панелей соответствует расположению
            if len(physical_indices) != rows * cols:
                self.get_logger().error(
                    f"Group '{name}' has {len(physical_indices)} physical panels, "
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
                    idx = physical_indices[r * cols + c]
                    if idx < len(self.physical_panels):
                        panel = self.physical_panels[idx]
                        try:
                            panel_width = int(panel['width']) if isinstance(panel['width'], str) else panel['width']
                            row_width += panel_width
                        except:
                            pass
                group_width = max(group_width, row_width)
            
            # Вычисляем высоту
            for c in range(cols):
                col_height = 0
                for r in range(rows):
                    idx = physical_indices[r * cols + c]
                    if idx < len(self.physical_panels):
                        panel = self.physical_panels[idx]
                        try:
                            panel_height = int(panel['height']) if isinstance(panel['height'], str) else panel['height']
                            col_height += panel_height
                        except:
                            pass
                group_height = max(group_height, col_height)
            
            group['width'] = group_width
            group['height'] = group_height
            
            self.get_logger().info(f"  Logical Group '{name}': {group_width}x{group_height} ({cols}x{rows} arrangement)")
        
        # Создаем общий буфер (RGB для каждого светодиода)
        self.buffer = bytearray(self.total_leds * 3)
        
        # Публикуем в топик драйвера
        self.output_publisher = self.create_publisher(ByteMultiArray, self.output_topic, 10)
        
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
        try:
            # Преобразуем arrangement в целые числа
            arrangement = []
            for val in group['arrangement']:
                if isinstance(val, str):
                    arrangement.append(int(val))
                else:
                    arrangement.append(int(val))
            cols = arrangement[0]
            rows = arrangement[1]
            
            # Преобразуем physical_indices в целые числа
            physical_indices = []
            for val in group['physical_indices']:
                if isinstance(val, str):
                    physical_indices.append(int(val))
                else:
                    physical_indices.append(int(val))
        except Exception as e:
            self.get_logger().error(f"Ошибка при обработке параметров группы: {e}")
            return
        
        # Обрабатываем каждую физическую панель в группе
        for r in range(rows):
            for c in range(cols):
                # Индекс физической панели в группе
                group_index = r * cols + c
                if group_index >= len(physical_indices):
                    continue
                physical_index = physical_indices[group_index]
                
                if physical_index >= len(self.physical_panels):
                    self.get_logger().error(f"Physical index {physical_index} out of range")
                    continue
                
                # Получаем размеры физической панели
                panel = self.physical_panels[physical_index]
                try:
                    panel_width = int(panel['width']) if isinstance(panel['width'], str) else panel['width']
                    panel_height = int(panel['height']) if isinstance(panel['height'], str) else panel['height']
                except Exception as e:
                    self.get_logger().error(f"Ошибка при получении размеров панели: {e}")
                    continue
                
                # Вычисляем смещение в логической группе
                x_offset = 0
                for i in range(c):
                    idx = physical_indices[r * cols + i]
                    if idx < len(self.physical_panels):
                        try:
                            width = int(self.physical_panels[idx]['width']) if isinstance(self.physical_panels[idx]['width'], str) else self.physical_panels[idx]['width']
                            x_offset += width
                        except:
                            pass
                
                y_offset = 0
                for i in range(r):
                    idx = physical_indices[i * cols + c]
                    if idx < len(self.physical_panels):
                        try:
                            height = int(self.physical_panels[idx]['height']) if isinstance(self.physical_panels[idx]['height'], str) else self.physical_panels[idx]['height']
                            y_offset += height
                        except:
                            pass
                
                # Создаем буфер для этой панели
                panel_buffer = bytearray(panel_width * panel_height * 3)
                
                # Копируем данные из общего изображения в буфер панели
                for y in range(panel_height):
                    for x in range(panel_width):
                        src_idx = ((y + y_offset) * group['width'] + (x + x_offset)) * 3
                        dst_idx = (y * panel_width + x) * 3
                        if src_idx < len(image_data) and dst_idx < len(panel_buffer):
                            panel_buffer[dst_idx:dst_idx+3] = image_data[src_idx:src_idx+3]
                
                # Обновляем общий буфер
                if physical_index < len(self.physical_offsets):
                    panel_offset = self.physical_offsets[physical_index] * 3
                    self.buffer[panel_offset:panel_offset + len(panel_buffer)] = panel_buffer
        
        # Публикуем обновленный буфер
        self.publish_buffer()
    
    def publish_buffer(self):
        """Публикует общий буфер в топик драйвера"""
        msg = ByteMultiArray()
        msg.data = self.buffer
        self.output_publisher.publish(msg)
    
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
            if 'name' in group:
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