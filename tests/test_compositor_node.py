"""Композитор: одна запись ленты на кадр лица, раскладка из YAML = раскладке в коде."""

import random
from pathlib import Path

from conftest import Image

from led_matrix_compositor import led_matrix_compositor as comp

PKG_YAML = Path(__file__).resolve().parents[1] / 'led_matrix_compositor' / 'config' / 'led_matrix_compositor.yaml'
GROUPS = {
    'main_display': (25, 5),
    'wheel_front_left': (8, 8),
    'wheel_front_right': (8, 8),
    'wheel_rear_left': (8, 8),
    'wheel_rear_right': (8, 8),
}


def make(params, config_file=''):
    params['config_file'] = config_file
    node = comp.LEDMatrixCompositor()
    return node, node.publishers[0]


def image(group, seed):
    w, h = GROUPS[group]
    rng = random.Random(seed)
    msg = Image()
    msg.header.frame_id = group
    msg.width, msg.height = w, h
    msg.data = bytes(rng.randrange(256) for _ in range(w * h * 3))
    return msg


def feed_face(node, seed=0):
    for i, group in enumerate(GROUPS):
        node.image_callback(image(group, seed + i))


def test_five_panel_images_become_one_strip_write(params):
    node, pub = make(params, str(PKG_YAML))
    before = len(pub.sent)                 # начальный чёрный буфер
    feed_face(node)
    assert len(pub.sent) == before         # на каждую панель ленту не пишем
    node.timers[0].callback()
    assert len(pub.sent) == before + 1     # один кадр — одна запись
    node.timers[0].callback()
    assert len(pub.sent) == before + 1     # ничего не менялось — не пишем


def test_publish_rate_cap_from_config(params):
    node, _ = make(params, str(PKG_YAML))
    assert abs(node.timers[0].period - 1 / 60.0) < 1e-9


def test_zero_rate_restores_publish_per_image(params, tmp_path):
    cfg = tmp_path / 'c.yaml'
    cfg.write_text(PKG_YAML.read_text(encoding='utf-8').replace('max_publish_hz: 60.0', 'max_publish_hz: 0'),
                   encoding='utf-8')
    node, pub = make(params, str(cfg))
    before = len(pub.sent)
    feed_face(node)
    assert node.timers == []
    assert len(pub.sent) == before + len(GROUPS)


def test_package_yaml_maps_pixels_exactly_like_embedded_defaults(params):
    """TD-7 перевёл композитор на YAML; живой робот работал на раскладке из кода."""
    from_yaml, pub_yaml = make(params, str(PKG_YAML))
    from_code, pub_code = make(params, '')
    assert from_yaml.total_leds == from_code.total_leds == 381
    feed_face(from_yaml, seed=10)
    feed_face(from_code, seed=10)
    from_yaml.timers[0].callback()
    from_code.timers[0].callback()
    assert list(pub_yaml.sent[-1].data) == list(pub_code.sent[-1].data)
