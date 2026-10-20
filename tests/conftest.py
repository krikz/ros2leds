"""Заглушки ROS 2 и pi5neo, чтобы гонять ноды драйвера и композитора без робота.

Подменяются только те части API, которыми пользуются ноды. Тесты, которым
нужен реальный ROS, сюда не относятся — это проверки логики кадра.
"""

import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
for pkg in ('led_matrix_driver', 'led_matrix_compositor'):
    sys.path.insert(0, str(ROOT / pkg))

PARAM_OVERRIDES = {}


class _Logger:
    def __init__(self):
        self.records = []

    def _log(self, level, msg):
        self.records.append((level, msg))

    def info(self, msg):
        self._log('info', msg)

    def warn(self, msg):
        self._log('warn', msg)

    warning = warn

    def error(self, msg):
        self._log('error', msg)

    def debug(self, msg):
        self._log('debug', msg)


class _Publisher:
    def __init__(self, topic, depth):
        self.topic = topic
        self.depth = depth
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class Node:
    def __init__(self, name, **_kw):
        self.node_name = name
        self._params = {}
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self._logger = _Logger()

    def declare_parameter(self, name, value=None, descriptor=None):
        self._params.setdefault(name, PARAM_OVERRIDES.get(name, value))

    def declare_parameters(self, namespace, parameters):
        for p in parameters:
            self.declare_parameter(p[0], p[1])

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params[name])

    def create_publisher(self, _type, topic, depth):
        pub = _Publisher(topic, depth)
        self.publishers.append(pub)
        return pub

    def create_subscription(self, _type, topic, callback, depth):
        sub = SimpleNamespace(topic=topic, callback=callback, depth=depth)
        self.subscriptions.append(sub)
        return sub

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self._logger

    def destroy_node(self):
        pass


class Int8MultiArray:
    def __init__(self):
        self.data = []


class Image:
    def __init__(self):
        self.header = SimpleNamespace(frame_id='')
        self.width = 0
        self.height = 0
        self.encoding = 'rgb8'
        self.data = b''


class ParameterDescriptor:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class FakePi5Neo:
    """Повторяет поля pi5neo 1.1.0, которые трогает драйвер; пишет в список, а не в SPI."""

    instances = []

    def __init__(self, spi_device, num_leds, spi_speed_khz):
        self.num_leds = num_leds
        self.bytes_per_led = 24
        self.pixel_type = SimpleNamespace(name='GRB')
        self.raw_data = [0] * (num_leds * 24)
        self.led_state = [(0, 0, 0)] * num_leds
        self.sent = []
        self.update_calls = []
        FakePi5Neo.instances.append(self)

    def send_spi_data(self):
        self.sent.append(list(self.raw_data))

    def set_led_color(self, index, red, green, blue, white=0):
        self.led_state[index] = (red, green, blue)

    def update_strip(self, sleep_duration=0.1):
        self.update_calls.append(sleep_duration)

    def clear_strip(self):
        self.led_state = [(0, 0, 0)] * self.num_leds

    def fill_strip(self, *a):
        self.clear_strip()

    def open_spi_device(self, path):
        return True

    def close(self):
        pass


def _install_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = Node
    rclpy.node = rclpy_node

    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.Int8MultiArray = Int8MultiArray
    std_msgs.msg = std_msgs_msg

    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msgs_msg.Image = Image
    sensor_msgs.msg = sensor_msgs_msg

    rcl_interfaces = types.ModuleType('rcl_interfaces')
    rcl_interfaces_msg = types.ModuleType('rcl_interfaces.msg')
    rcl_interfaces_msg.ParameterDescriptor = ParameterDescriptor
    rcl_interfaces.msg = rcl_interfaces_msg

    pi5neo = types.ModuleType('pi5neo')
    pi5neo.Pi5Neo = FakePi5Neo

    sys.modules.update({
        'rclpy': rclpy, 'rclpy.node': rclpy_node,
        'std_msgs': std_msgs, 'std_msgs.msg': std_msgs_msg,
        'sensor_msgs': sensor_msgs, 'sensor_msgs.msg': sensor_msgs_msg,
        'rcl_interfaces': rcl_interfaces, 'rcl_interfaces.msg': rcl_interfaces_msg,
        'pi5neo': pi5neo,
    })


_install_stubs()


@pytest.fixture
def params():
    """Параметры ROS-ноды для одного теста (как из YAML)."""
    PARAM_OVERRIDES.clear()
    FakePi5Neo.instances.clear()
    yield PARAM_OVERRIDES
    PARAM_OVERRIDES.clear()
