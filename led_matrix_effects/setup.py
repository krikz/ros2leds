from setuptools import setup

package_name = 'led_matrix_effects'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='krikz',
    maintainer_email='krikz@users.noreply.github.com',
    description='Pure numpy LED matrix effects for ros2leds panels: canvas, backgrounds, pulses, signals, lamps',
    license='MIT',
    tests_require=['pytest'],
)
