#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import numpy as np
import random

class SpiralFillPublisher(Node):
    def __init__(self):
        super().__init__('spiral_fill_publisher')

        # Параметры матрицы
        self.width = 25
        self.height = 5

        # Паблишер
        self.publisher = self.create_publisher(Image, 'panel_image', 10)

        # Изображение: RGB, 3 канала, uint8
        self.image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Начальный цвет
        self.color = self.random_color()

        # Текущая позиция
        self.x, self.y = 0, 0

        # Направления: вправо, вниз, влево, вверх
        self.dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        self.current_dir = 0  # начинаем — вправо

        # Таймер — 10 Гц
        self.create_timer(0.1, self.step)

        # Публикуем стартовую точку
        self.image[self.y, self.x] = self.color
        self.get_logger().info(f"Started spiral at (0,0) with color {self.color.tolist()}")

    def random_color(self):
        """Случайный яркий цвет"""
        return np.array([
            random.randint(0, 255),
            random.randint(0, 255),
            random.randint(0, 255)
        ], dtype=np.uint8)

    def publish_image(self):
        """Публикуем текущее изображение"""
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "main_display"
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = self.image.tobytes()
        self.publisher.publish(msg)

    def can_move_forward(self):
        """Можно ли шагнуть вперёд в текущем направлении?"""
        dx, dy = self.dirs[self.current_dir]
        nx, ny = self.x + dx, self.y + dy

        # Проверяем границы
        if not (0 <= nx < self.width and 0 <= ny < self.height):
            return False

        # Проверяем, не является ли следующая клетка уже закрашенной текущим цветом
        if np.array_equal(self.image[ny, nx], self.color):
            return False

        return True

    def turn_right(self):
        """Поворот направо (по часовой)"""
        self.current_dir = (self.current_dir + 1) % 4

    def step(self):
        """Один шаг спирали"""
        # Публикуем текущее состояние
        self.publish_image()

        # Проверяем, можем ли идти прямо
        if self.can_move_forward():
            dx, dy = self.dirs[self.current_dir]
            self.x += dx
            self.y += dy
            self.image[self.y, self.x] = self.color
        else:
            # Не можем идти прямо — пытаемся повернуть
            old_dir = self.current_dir
            self.turn_right()
            if self.can_move_forward():
                # После поворота можем идти — идём
                dx, dy = self.dirs[self.current_dir]
                self.x += dx
                self.y += dy
                self.image[self.y, self.x] = self.color
            else:
                # После поворота всё равно некуда — тупик
                self.get_logger().info(f"Dead end at ({self.x}, {self.y}). Restarting from (0,0) with new color.")

                # Меняем цвет
                self.color = self.random_color()

                # Сбрасываем позицию и направление
                self.x, self.y = 0, 0
                self.current_dir = 0  # снова вправо

                # Перекрашиваем (0,0)
                self.image[self.y, self.x] = self.color


def main(args=None):
    rclpy.init(args=args)
    node = SpiralFillPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()