from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Путь к конфигурационному файлу
    config_path = PathJoinSubstitution([
        FindPackageShare('led_matrix_compositor'),
        'config',
        'led_matrix_compositor.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='led_matrix_compositor',
            executable='led_matrix_compositor',
            name='led_matrix_compositor',
            output='screen',
            parameters=[config_path]
        )
    ])