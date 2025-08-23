from setuptools import setup

package_name = 'led_matrix_compositor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/led_matrix_compositor_launch.py']),
        ('share/' + package_name + '/config', ['config/led_matrix_compositor.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS 2 compositor for multiple LED matrix panels with logical grouping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_matrix_compositor = led_matrix_compositor.led_matrix_compositor:main',
        ],
    },
)