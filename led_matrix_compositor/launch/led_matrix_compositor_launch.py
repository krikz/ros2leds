from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Путь к конфигурационному файлу композитора
    compositor_config_path = PathJoinSubstitution([
        FindPackageShare('led_matrix_compositor'),
        'config',
        'led_matrix_compositor.yaml'
    ])
    
    # Путь к конфигурационному файлу драйвера
    driver_config_path = PathJoinSubstitution([
        FindPackageShare('led_matrix_driver'),
        'config',
        'led_matrix_driver.yaml'
    ])
    
    return LaunchDescription([
        # Запускаем драйвер матрицы
        Node(
            package='led_matrix_driver',
            executable='led_matrix_driver',
            name='led_matrix_driver',
            output='screen',
            parameters=[driver_config_path]
        ),
        
        # Запускаем композитор матрицы
        Node(
            package='led_matrix_compositor',
            executable='led_matrix_compositor',
            name='led_matrix_compositor',
            output='screen',
            parameters=[compositor_config_path]
        )
    ])