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

# Клонируем репозиторий и правильно организуем структуру
RUN git clone https://github.com/krikz/ros2leds.git /tmp/ros2leds && \
    cp -r /tmp/ros2leds/led_matrix_compositor /ws/src/ && \
    cp -r /tmp/ros2leds/led_matrix_driver /ws/src/ && \
    rm -rf /tmp/ros2leds

# Устанавливаем Python-зависимости для матрицы
RUN pip3 install --no-cache-dir \
    Pi5Neo \
    spidev \
    rpi_ws281x

# Устанавливаем рабочую директорию перед сборкой
WORKDIR /ws

# Сборка рабочей области
RUN . /opt/ros/humble/setup.sh && \
    echo "Запуск сборки..." && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Проверяем структуру установленных пакетов
RUN . /opt/ros/humble/setup.sh && \
    . /ws/install/setup.bash && \
    echo "Проверка структуры пакетов..." && \
    ls -R /ws/install/led_matrix_driver && \
    ls -R /ws/install/led_matrix_compositor

# Проверяем, что пакеты установлены после сборки
RUN . /opt/ros/humble/setup.sh && \
    . /ws/install/setup.bash && \
    echo "Проверка установленных пакетов..." && \
    ros2 pkg list | grep -E 'led_matrix_(compositor|driver)' && \
    echo "Проверка исполняемых файлов..." && \
# Источнирование окружения
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

# Экспорт переменной ROS_DISTRO
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0

# Создаем скрипт для запуска
RUN echo '#!/bin/bash' > /ws/run_leds.sh && \
    echo 'echo "Проверка доступа к SPI..."' >> /ws/run_leds.sh && \
    echo 'if [ ! -c "/dev/spidev0.0" ]; then' >> /ws/run_leds.sh && \
    echo '  echo "Ошибка: SPI устройство не найдено. Убедитесь, что SPI включен в raspi-config и контейнер запущен с --device /dev/spidev0.0:/dev/spidev0.0"' >> /ws/run_leds.sh && \
    echo '  exit 1' >> /ws/run_leds.sh && \
    echo 'fi' >> /ws/run_leds.sh && \
    echo '' >> /ws/run_leds.sh && \
    echo 'echo "Запуск LED матрицы..."' >> /ws/run_leds.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /ws/run_leds.sh && \
    echo 'source /ws/install/setup.bash' >> /ws/run_leds.sh && \
    echo '' >> /ws/run_leds.sh && \
    echo '# Запускаем driver в фоне' >> /ws/run_leds.sh && \
    echo 'echo "Проверка наличия исполняемых файлов..."' >> /ws/run_leds.sh && \
    echo 'if ros2 pkg executables led_matrix_driver | grep -q "led_matrix_driver"; then' >> /ws/run_leds.sh && \
    echo '  echo "Запуск через ros2 run led_matrix_driver led_matrix_driver"' >> /ws/run_leds.sh && \
    echo '  ros2 run led_matrix_driver led_matrix_driver &' >> /ws/run_leds.sh && \
    echo 'else' >> /ws/run_leds.sh && \
    echo '  echo "Запуск через прямой вызов Python"' >> /ws/run_leds.sh && \
    echo '  python3 /ws/install/led_matrix_driver/lib/led_matrix_driver/led_matrix_driver.py &' >> /ws/run_leds.sh && \
    echo 'fi' >> /ws/run_leds.sh && \
    echo '' >> /ws/run_leds.sh && \
    echo '# Запускаем compositor' >> /ws/run_leds.sh && \
    echo 'if ros2 pkg executables led_matrix_compositor | grep -q "led_matrix_compositor"; then' >> /ws/run_leds.sh && \
    echo '  echo "Запуск через ros2 run led_matrix_compositor led_matrix_compositor"' >> /ws/run_leds.sh && \
    echo '  ros2 run led_matrix_compositor led_matrix_compositor' >> /ws/run_leds.sh && \
    echo 'else' >> /ws/run_leds.sh && \
    echo '  echo "Запуск через прямой вызов Python"' >> /ws/run_leds.sh && \
    echo '  python3 /ws/install/led_matrix_compositor/lib/led_matrix_compositor/led_matrix_compositor.py' >> /ws/run_leds.sh && \
    echo 'fi' >> /ws/run_leds.sh && \
    echo '' >> /ws/run_leds.sh && \
    echo '# Ждем завершения' >> /ws/run_leds.sh && \
    echo 'wait' >> /ws/run_leds.sh && \
    chmod +x /ws/run_leds.sh
    
# Устанавливаем рабочую директорию по умолчанию
WORKDIR /ws

# Команда по умолчанию
CMD ["/ws/run_leds.sh"]
