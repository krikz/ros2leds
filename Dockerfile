# Используем базовый образ ROS 2 Humble для Raspberry Pi
FROM introlab3it/rtabmap_ros:humble-latest

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    build-essential \
    libffi-dev \
    python3-dev \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Создаем рабочую область ROS 2
WORKDIR /ws
RUN mkdir -p src

# Устанавливаем Python-зависимости для матрицы
RUN pip3 install --no-cache-dir \
    Pi5Neo \
    spidev \
    rpi_ws281x

# Копируем пакеты из локальной директории
COPY led_matrix_compositor src/led_matrix_compositor
COPY led_matrix_driver src/led_matrix_driver

# Сборка рабочей области
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Источнирование окружения
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc && 

# Экспорт переменных ROS 2 для работы в сети
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0

# Команда по умолчанию
CMD ["bash", "-c", "source /ws/install/setup.bash && ros2 launch led_matrix_compositor led_matrix_compositor_launch.py"]