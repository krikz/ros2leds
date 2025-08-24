# Используем базовый образ ROS 2 Humble для Raspberry Pi
FROM introlab3it/rtabmap_ros:humble-latest

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    build-essential \
    libffi-dev \
    python3-dev \
    python3-venv \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Создаем рабочую область ROS 2
WORKDIR /ws
RUN mkdir -p src

# Копируем пакеты из локальной директории
COPY led_matrix_compositor src/led_matrix_compositor
COPY led_matrix_driver src/led_matrix_driver

# Создаем и активируем виртуальное окружение
RUN python3 -m venv /opt/ros2leds_venv

# Устанавливаем Python-зависимости для матрицы в виртуальное окружение
RUN . /opt/ros2leds_venv/bin/activate && \
    pip install --upgrade pip && \
    pip install --no-cache-dir \
    Pi5Neo \
    spidev \
    rpi_ws281x

# Обновляем зависимости перед сборкой
RUN . /opt/ros/humble/setup.sh && \
    . /opt/ros2leds_venv/bin/activate && \
    echo "Обновление зависимостей перед сборкой..." && \
    pip list --outdated --format=json | python3 -c "
import json, sys
data = json.load(sys.stdin)
if data:
    packages = [item['name'] for item in data]
    if packages:
        print('Обновление пакетов:', ' '.join(packages))
        import subprocess
        subprocess.check_call(['pip', 'install', '--upgrade'] + packages)
    else:
        print('Нет устаревших пакетов')
else:
    print('Нет устаревших пакетов')
"

# Сборка рабочей области
RUN . /opt/ros/humble/setup.sh && \
    . /opt/ros2leds_venv/bin/activate && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Источнирование окружения
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros2leds_venv/bin/activate" >> /root/.bashrc

# Экспорт переменной ROS_DISTRO
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0

# Команда по умолчанию
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch led_matrix_compositor led_matrix_compositor_launch.py"]