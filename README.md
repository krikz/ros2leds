# ROS2 LED Matrix Packages

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

ROS 2 пакеты для управления светодиодными матрицами на Raspberry Pi 5 с поддержкой как отдельных панелей, так и объединенных логических групп.

![LED Matrix Example](https://i.imgur.com/placeholder.jpg)

## Обзор

Этот репозиторий содержит два ROS 2 пакета для работы со светодиодными матрицами:

1. **led_matrix_driver** - низкоуровневый драйвер для управления светодиодной цепочкой через SPI
2. **led_matrix_compositor** - высокий уровень для объединения физических панелей в логические группы с произвольным расположением

Эти пакеты позволяют:
- Управлять отдельными светодиодными панелями (например, фарами ровера)
- Объединять несколько физических панелей в логические группы (например, 5 панелей 5x5 как одну матрицу 5x25)
- Отправлять изображения напрямую на логические группы без ручного разбиения
- Работать с панелями разного размера в единой системе

## Требования

- Raspberry Pi 5 (или совместимый)
- ROS 2 Humble Hawksbill (или более новая версия)
- Библиотека pi5neo: `pip3 install pi5neo --break-system-packages`
- SPI должен быть включен в Raspberry Pi

## Установка

1. Клонируйте репозиторий в ваш рабочий каталог ROS 2:
```bash
cd ~/ros2_ws/src
git clone https://github.com/krikz/ros2leds.git
```

2. Установите зависимости:
```bash
cd ~/ros2_ws
sudo apt update
sudo apt install python3-pip
pip3 install pi5neo --break-system-packages
```

3. Соберите пакеты:
```bash
colcon build --packages-select led_matrix_driver led_matrix_compositor
source install/setup.bash
```

## Конфигурация

### 1. led_matrix_driver

Драйвер управляет физической цепочкой светодиодов. Основные параметры в `config/led_matrix_driver.yaml`:

```yaml
led_matrix_driver:
  ros__parameters:
    num_leds: 125        # Общее количество светодиодов
    spi_speed_khz: 1200  # Скорость SPI в кГц
    spi_device: '/dev/spidev0.0'  # SPI устройство
    input_topic: 'led_matrix/data'  # Топик для приема данных
```

### 2. led_matrix_compositor

Композитор объединяет физические панели в логические группы. Основные параметры в `config/led_matrix_compositor.yaml`:

```yaml
led_matrix_compositor:
  ros__parameters:
    physical_panels:
      # 4 отдельные панели 8x8 (фары)
      - width: 8
        height: 8
      - width: 8
        height: 8
      - width: 8
        height: 8
      - width: 8
        height: 8
      # 5 панелей 5x5, соединенных в одну линию
      - width: 5
        height: 5
      - width: 5
        height: 5
      - width: 5
        height: 5
      - width: 5
        height: 5
      - width: 5
        height: 5
    
    logical_groups:
      # Отдельные панели (фары)
      - name: "headlight_front_left"
        physical_indices: [0]
        arrangement: [1, 1]
        
      - name: "headlight_front_right"
        physical_indices: [1]
        arrangement: [1, 1]
        
      - name: "headlight_rear_left"
        physical_indices: [2]
        arrangement: [1, 1]
        
      - name: "headlight_rear_right"
        physical_indices: [3]
        arrangement: [1, 1]
        
      # Объединенная матрица из 5 панелей 5x5 (5x25)
      - name: "main_display"
        physical_indices: [4, 5, 6, 7, 8]
        arrangement: [5, 1]
```

## Использование

### Запуск пакетов

1. Запустите драйвер:
```bash
ros2 launch led_matrix_driver led_matrix_driver_launch.py
```

2. Запустите композитор:
```bash
ros2 launch led_matrix_compositor led_matrix_compositor_launch.py
```

### Отправка изображений

Отправьте изображение на логическую группу, указав имя группы в `header.frame_id`:

```bash
ros2 topic pub --once /panel_image sensor_msgs/Image "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'main_display'
height: 5
width: 25
encoding: 'rgb8'
is_bigendian: 0
step: 75
 [0,0,0, 10,0,0, 20,0,0, ...]"
```

### Примеры использования

#### Отправка шахматной доски на фару
```bash
ros2 topic pub --once /panel_image sensor_msgs/Image "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'headlight_front_left'
height: 8
width: 8
encoding: 'rgb8'
is_bigendian: 0
step: 24
 [255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0,
  0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255,
  255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0,
  0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255,
  255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0,
  0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255,
  255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0,
  0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255, 0,0,0, 255,255,255]"
```

#### Отправка градиента на основной дисплей
```bash
ros2 topic pub --once /panel_image sensor_msgs/Image "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'main_display'
height: 5
width: 25
encoding: 'rgb8'
is_bigendian: 0
step: 75
 [0,0,0, 10,0,0, 20,0,0, 30,0,0, 40,0,0, 50,0,0, 60,0,0, 70,0,0, 80,0,0, 90,0,0,
  100,0,0, 110,0,0, 120,0,0, 130,0,0, 140,0,0, 150,0,0, 160,0,0, 170,0,0, 180,0,0, 190,0,0,
  200,0,0, 210,0,0, 220,0,0, 230,0,0, 240,0,0, 250,0,0, 255,0,0, 250,0,0, 240,0,0, 230,0,0,
  220,0,0, 210,0,0, 200,0,0, 190,0,0, 180,0,0, 170,0,0, 160,0,0, 150,0,0, 140,0,0, 130,0,0,
  120,0,0, 110,0,0, 100,0,0, 90,0,0, 80,0,0, 70,0,0, 60,0,0, 50,0,0, 40,0,0, 30,0,0]"
```

## Поддерживаемые конфигурации

### Одномерное расположение
- Горизонтальное: `arrangement: [N, 1]` (N панелей в ряд)
- Вертикальное: `arrangement: [1, N]` (N панелей в столбец)

### Двумерное расположение
- Прямоугольная сетка: `arrangement: [столбцы, строки]`
- Пример: `arrangement: [3, 2]` для 3 столбцов и 2 строк

## Пример клиентского узла

Создайте узел для отправки изображений:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class PanelImagePublisher(Node):
    def __init__(self):
        super().__init__('panel_image_publisher')
        self.publisher_ = self.create_publisher(Image, 'panel_image', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.frame = 0

    def create_gradient_image(self, width, height):
        """Создает градиентное изображение"""
        image = np.zeros((height, width, 3), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                r = int(255 * x / width)
                g = int(255 * y / height)
                b = 0
                image[y, x] = [r, g, b]
        return image.tobytes()

    def timer_callback(self):
        # Отправляем градиент на основной дисплей
        msg = Image()
        msg.header.frame_id = "main_display"
        msg.height = 5
        msg.width = 25
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 25 * 3
        msg.data = self.create_gradient_image(25, 5)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = PanelImagePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Лицензия

Этот проект лицензирован в соответствии с MIT License - подробности см. в файле [LICENSE](LICENSE).

## Автор

[krikz](https://github.com/krikz) - разработчик пакетов

## Схема подключения

Для подключения светодиодных панелей к Raspberry Pi 5:

```
Raspberry Pi 5   ->   Светодиодные панели
----------------------------------------
GPIO 10 (MOSI)   ->   DIN панели 0
GPIO 8 (SPI CE0) ->   CS (если требуется)
3.3V             ->   VCC (5V для некоторых панелей)
GND              ->   GND
```

Примечание: Для WS2812B панелей обычно требуется только DIN, GND и питание (5V).