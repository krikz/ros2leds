from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Принудительно указываем путь к конфигурационному файлу
    config_path = '/ws/install/led_matrix_compositor/share/led_matrix_compositor/config/led_matrix_compositor.yaml'
    
    # Проверяем существование файла
    if not os.path.exists(config_path):
        print(f"WARNING: Config file not found at {config_path}")
        # Создаем временный файл, если не существует
        import yaml
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        default_config = {
            'led_matrix_compositor': {
                'ros__parameters': {
                    'physical_panels': [
                        {'width': 5, 'height': 5},
                        {'width': 5, 'height': 5},
                        {'width': 5, 'height': 5},
                        {'width': 5, 'height': 5},
                        {'width': 5, 'height': 5}
                    ],
                    'logical_groups': [
                        {
                            'name': 'main_display',
                            'physical_indices': [0, 1, 2, 3, 4],
                            'arrangement': [5, 1]
                        }
                    ],
                    'output_topic': 'led_matrix/data',
                    'input_topic': 'panel_image'
                }
            }
        }
        with open(config_path, 'w') as f:
            yaml.dump(default_config, f)
        print(f"Created default config file at {config_path}")
    
    return LaunchDescription([
        # Запускаем драйвер
        Node(
            package='led_matrix_driver',
            executable='led_matrix_driver',
            name='led_matrix_driver',
            output='screen',
            parameters=['/ws/install/led_matrix_driver/share/led_matrix_driver/config/led_matrix_driver.yaml']
        ),
        
        # Запускаем композитор с явным указанием пути к конфигурации
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'led_matrix_compositor', 'led_matrix_compositor',
                '--ros-args', '--params-file', config_path
            ],
            output='screen'
        )
    ])