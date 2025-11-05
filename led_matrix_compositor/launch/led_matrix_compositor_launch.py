from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Конфиг драйвера - приоритет volume > install
    driver_config_volume = '/config/led_matrix/led_matrix_driver.yaml'
    driver_config_install = '/ws/install/led_matrix_driver/share/led_matrix_driver/config/led_matrix_driver.yaml'
    driver_config = driver_config_volume if os.path.exists(driver_config_volume) else driver_config_install
    
    return LaunchDescription([
        # Запускаем драйвер
        Node(
            package='led_matrix_driver',
            executable='led_matrix_driver',
            name='led_matrix_driver',
            output='screen',
            parameters=[driver_config]
        ),
        
        # Запускаем композитор БЕЗ конфига (использует хардкод из Python кода)
        Node(
            package='led_matrix_compositor',
            executable='led_matrix_compositor',
            name='led_matrix_compositor',
            output='screen'
        )
    ])