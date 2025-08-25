#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import math
import time
import random

class RobotMouthPublisher(Node):
    def __init__(self):
        super().__init__('robot_mouth_publisher')
        self.publisher_ = self.create_publisher(Image, '/panel_image', 10)
        
        # Параметры дисплея
        self.width = 25
        self.height = 5
        
        # Управление анимацией
        self.time_offset = 0.0
        self.time_step = 0.05  # Скорость анимации
        
        # Управление режимами
        self.modes = [
            "Sound Wave Mouth",
            "Cyberpunk Display",
            "Plasma Fire Mouth",
            "Audio Equalizer",
            "Holographic Mouth"
        ]
        self.current_mode = 0
        self.last_mode_switch = time.time()
        self.mode_switch_interval = 10.0  # Переключение каждые 10 сек
        
        # Для плавного изменения амплитуды
        self.smoothed_amplitude = 0.0
        
        # Таймеры
        self.create_timer(0.033, self.frame_callback)  # 30 FPS
        self.create_timer(1.0, self.debug_callback)    # Отладочная информация
        
        self.get_logger().info(f'Robot mouth publisher started with {len(self.modes)} modes')
        self.get_logger().info(f'Current mode: {self.modes[self.current_mode]} (will switch in {self.mode_switch_interval:.1f}s)')

    def generate_amplitude(self):
        """Генерирует сложный амплитудный сигнал как сумму синусоид"""
        # Основная низкочастотная волна (медленные изменения)
        base = math.sin(self.time_offset * 0.5) * 0.5 + 0.5
        
        # Дополнительные высокочастотные компоненты (детали)
        detail1 = math.sin(self.time_offset * 3.0) * 0.2
        detail2 = math.sin(self.time_offset * 7.0 + 1.0) * 0.15
        
        # Случайные "всплески" (имитация речи)
        if random.random() < 0.02:
            detail2 += 0.3 * random.random()
        
        # Комбинируем и нормализуем
        raw_amplitude = base + detail1 + detail2
        raw_amplitude = max(0.0, min(1.0, raw_amplitude))  # Ограничиваем [0, 1]
        
        # Плавное сглаживание
        self.smoothed_amplitude = 0.9 * self.smoothed_amplitude + 0.1 * raw_amplitude
        return self.smoothed_amplitude

    def generate_sound_mouth(self, amplitude):
        """Режим 1: Звуковая волна в форме рта"""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        A = min(1.0, max(0.0, amplitude))
        
        # Формируем U-образный рот
        mouth_height = max(1, int(3 * A * (1 + 0.5 * math.sin(self.time_offset * 2.0))))
        center_x = self.width // 2
        
        for x in range(self.width):
            # Параболическая форма рта
            dx = abs(x - center_x) / center_x
            wave_height = int(mouth_height * (1 - dx * dx * 0.7))
            
            # Зубы (только при амплитуде > 0.3)
            if A > 0.3 and (x < 4 or x > self.width - 5):
                img[4, x] = [200, 200, 255]
            
            # Губы
            img[4, x] = [100, 0, 0]
            
            # Внутренность рта с эффектом волны
            for y in range(4 - wave_height, 4):
                r = int(200 * A * (0.7 + 0.3 * math.sin(self.time_offset + y)))
                g = int(100 * A * (y - (4 - wave_height)) / wave_height)
                img[y, x] = [r, g, 0]
        
        return img

    def generate_cyber_mouth(self, amplitude):
        """Режим 2: Киберпанк-дисплей"""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        A = min(1.0, max(0.0, amplitude))
        offset = int((self.time_offset * 10) % self.width)
        
        for x in range(self.width):
            # Спектрограмма
            bar_height = int(3 * A * (1 - abs(x/self.width - 0.5)*1.5))
            
            # Зубы как разделители
            if x % 5 == 0:
                img[4, x] = [255, 255, 255]
            
            # Светодиодные столбики
            for y in range(4 - bar_height, 4):
                intensity = 150 + int(100 * A)
                img[y, x] = [0, intensity, intensity]
            
            # Бегущая строка
            if (x + offset) % 5 == 0 and A > 0.4:
                img[0, x] = [255, 255, 0]
        
        # Эффект сканирования
        scan_line = int((self.time_offset * 5) % self.height)
        if scan_line < 4:
            for x in range(self.width):
                img[scan_line, x] = [50, 255, 50]
        
        return img

    def generate_plasma_mouth(self, amplitude):
        """Режим 3: Плазменный огонь"""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        A = min(1.0, max(0.0, amplitude))
        
        for y in range(self.height):
            for x in range(self.width):
                # Формируем область рта
                in_mouth = (y > 1) and (abs(x - 12) < 10 - y*2)
                if not in_mouth:
                    continue
                
                # Плазменные волны
                value1 = math.sin(x * 0.2 + self.time_offset * A)
                value2 = math.sin(y * 0.5 + self.time_offset * 2)
                value3 = math.sin(math.sqrt(x*x + y*y) * 0.3 + self.time_offset * 0.5)
                plasma = (value1 + value2 + value3) / 3.0
                
                # Цветовая схема (красный -> белый)
                r = int(200 * (0.5 + plasma/2) * A * 1.2)
                g = int(100 * (0.5 + plasma/2) * A * 0.7)
                b = int(50 * (0.5 + plasma/2) * A * 0.3)
                img[y, x] = [min(255, r), min(255, g), min(255, b)]
        
        # Зубы с эффектом свечения
        for x in [2, 3, 22, 23]:
            glow = int(100 + 155 * math.sin(self.time_offset * 3))
            img[4, x] = [glow, glow, 255]
        
        return img

    def generate_equalizer_mouth(self, amplitude):
        """Режим 4: Аудио эквалайзер"""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        A = min(1.0, max(0.0, amplitude))
        
        # Генерируем колебания
        for x in range(self.width):
            # Базовая высота столбика
            base_height = 1 + int(3 * A * (1 - abs(x/self.width - 0.5)))
            
            # Добавляем "дрожание" от амплитуды
            noise = int(1 * A * math.sin(self.time_offset * 5 + x))
            bar_height = max(1, min(4, base_height + noise))
            
            # Зубы (высокие пики)
            if bar_height == 4 and x % 4 == 0:
                img[4, x] = [255, 255, 255]
            
            # Рот из столбиков
            for y in range(4 - bar_height, 4):
                # Градиент: темнее у основания
                intensity = 100 + int(155 * (y - (4 - bar_height)) / bar_height)
                img[y, x] = [intensity, 0, 0]
        
        # Эффект "ударной волны" при высокой амплитуде
        if A > 0.8:
            wave_pos = int((self.time_offset * 15) % self.width)
            for y in range(self.height):
                if 0 <= wave_pos < self.width:
                    img[y, wave_pos] = [255, 255, 255]
        
        return img

    def generate_hologram_mouth(self, amplitude):
        """Режим 5: Голографический рот"""
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        A = min(1.0, max(0.0, amplitude))
        
        # Генерируем "голографический шум"
        for y in range(self.height):
            for x in range(self.width):
                # Форма рта
                in_mouth = (y > 1) and (x > 3 and x < 22) and (abs(x-12) < 8 - y)
                
                # Стабильность проекции зависит от амплитуды
                if in_mouth and random.random() < A * 0.8:
                    # Голубой градиент
                    intensity = 100 + int(155 * A)
                    img[y, x] = [0, intensity//2, intensity]
                    
                    # Эффект "дрожания" при низкой амплитуде
                    if A < 0.4 and int(x + y + self.time_offset*10) % 3 == 0:
                        img[y, x] = [0, 0, 0]  # Прорывы
        
        # Контурные линии
        for x in range(0, self.width, 4):
            if random.random() < A:
                img[0, x] = [0, 255, 255]
        
        # Эффект интерференции
        interference_y = int((self.time_offset * 3) % self.height)
        if interference_y < 4:
            for x in range(self.width):
                if x % 3 == 0:
                    img[interference_y, x] = [255, 255, 255]
        
        return img

    def frame_callback(self):
        """Основной callback для генерации кадров"""
        # Обновляем временной сдвиг
        self.time_offset += self.time_step
        
        # Генерируем амплитуду
        amplitude = self.generate_amplitude()
        
        # Проверяем необходимость смены режима
        if time.time() - self.last_mode_switch > self.mode_switch_interval:
            self.current_mode = (self.current_mode + 1) % len(self.modes)
            self.last_mode_switch = time.time()
            self.get_logger().info(f'Mode switched to: {self.modes[self.current_mode]}')
        
        # Генерируем изображение в зависимости от режима
        if self.current_mode == 0:
            img = self.generate_sound_mouth(amplitude)
        elif self.current_mode == 1:
            img = self.generate_cyber_mouth(amplitude)
        elif self.current_mode == 2:
            img = self.generate_plasma_mouth(amplitude)
        elif self.current_mode == 3:
            img = self.generate_equalizer_mouth(amplitude)
        else:
            img = self.generate_hologram_mouth(amplitude)
        
        # Создаем и публикуем сообщение
        msg = Image()
        msg.header.frame_id = 'robot_mouth'
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = img.tobytes()
        
        self.publisher_.publish(msg)

    def debug_callback(self):
        """Отладочный callback для информации в лог"""
        remaining = max(0, self.mode_switch_interval - (time.time() - self.last_mode_switch))
        self.get_logger().debug(
            f'Mode: {self.modes[self.current_mode]} | '
            f'Amplitude: {self.smoothed_amplitude:.2f} | '
            f'Next switch: {remaining:.1f}s'
        )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RobotMouthPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()