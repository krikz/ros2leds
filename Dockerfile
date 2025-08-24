# Используем базовый образ ROS 2 Humble для Raspberry Pi
FROM introlab3it/rtabmap_ros:humble-latest

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    libffi-dev \
    python3-dev \
    python3-venv \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Создаем рабочую область ROS 2
RUN mkdir -p /ws/src

# Клонируем репозиторий с временной меткой для отключения кэширования
# Добавляем временный параметр, который делает команду уникальной каждый раз
RUN echo "Cloning repository at $(date)" && \
    git clone https://github.com/krikz/ros2leds.git /tmp/ros2leds && \
    cp -r /tmp/ros2leds/led_matrix_compositor /ws/src/ && \
    cp -r /tmp/ros2leds/led_matrix_driver /ws/src/ && \
    rm -rf /tmp/ros2leds

# Устанавливаем Python-зависимости для матрицы
RUN pip3 install --break-system-packages --no-cache-dir \
    Pi5Neo \
    spidev \
    rpi_ws281x

# Устанавливаем рабочую директорию перед сборкой
WORKDIR /ws

# Сборка рабочей области
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Источнирование окружения
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# Экспорт переменной ROS_DISTRO
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0

# Команда по умолчанию
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch led_matrix_compositor led_matrix_compositor_launch.py"]