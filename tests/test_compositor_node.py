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


# Раскладка, на которой робот работал до 20.10.2026 (ros2leds afb263d / main 8b27154).
PRE_PHOTO_GROUPS = {
    'main_display': ([0, 1, 2, 3, 4], [5, 1], False, True),
    'wheel_front_left': ([5], [1, 1], False, False),
    'wheel_front_right': ([6], [1, 1], True, True),
    'wheel_rear_left': ([7], [1, 1], False, False),
    'wheel_rear_right': ([8], [1, 1], False, False),
}


def led_index_map(node):
    """(group, y, x) -> индекс светодиода в ленте."""
    out = {}
    for g, (w, h) in GROUPS.items():
        for y in range(h):
            for x in range(w):
                node.buffer[:] = bytes(len(node.buffer))
                data = bytearray(w * h * 3)
                data[(y * w + x) * 3:(y * w + x) * 3 + 3] = b'\xff\xff\xff'
                node._handle_logical_group(node.logical_groups_map[g], bytes(data))
                lit = {k // 3 for k, v in enumerate(node.buffer) if v}
                assert len(lit) == 1
                out[(g, y, x)] = lit.pop()
    return out


def test_front_is_the_pre_photo_layout_turned_180_degrees(params, tmp_path):
    """Тест-кадр с буквой F 20.10.2026: при прежней раскладке весь перед был
    повёрнут на 180°, а wheel_front_left стояла на другом борту, чем
    wheel_rear_left. Новая раскладка = прежняя с поворотом переда и обменом фар."""
    import yaml
    cfg = yaml.safe_load(PKG_YAML.read_text(encoding='utf-8'))
    groups = cfg['led_matrix_compositor']['ros__parameters']['logical_groups']
    for g in groups:
        idx, arr, fx, fy = PRE_PHOTO_GROUPS[g['name']]
        g.update(physical_indices=idx, arrangement=arr, flip_x=fx, flip_y=fy)
    old_yaml = tmp_path / 'pre_photo.yaml'
    old_yaml.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding='utf-8')

    old, _ = make(params, str(old_yaml))
    new, _ = make(params, str(PKG_YAML))
    old_map, new_map = led_index_map(old), led_index_map(new)
    swap = {'wheel_front_left': 'wheel_front_right', 'wheel_front_right': 'wheel_front_left'}
    for (g, y, x), led in new_map.items():
        w, h = GROUPS[g]
        if g == 'main_display' or g in swap:
            want = old_map[(swap.get(g, g), h - 1 - y, w - 1 - x)]
        else:
            want = old_map[(g, y, x)]
        assert led == want, (g, y, x)


# --- #3733: один Image на кадр, без повторов, без таймера вхолостую ---------------------------

def frame_msg(groups, seed=0):
    """Весь кадр одним Image: frame_id = группы через запятую, data = картинки подряд."""
    parts = [image(g, seed + i) for i, g in enumerate(groups)]
    msg = Image()
    msg.header.frame_id = ','.join(groups)
    msg.data = b''.join(bytes(p.data) for p in parts)
    msg.width, msg.height = len(msg.data) // 3, 1
    return msg, parts


def settle(node):
    """Прошлая запись была давно: лимит max_publish_hz кадру не мешает."""
    node._last_publish_at -= 1.0


def test_one_frame_message_gives_the_same_strip_as_separate_panel_images(params):
    sep, pub_sep = make(params, str(PKG_YAML))
    one, pub_one = make(params, str(PKG_YAML))
    names = list(GROUPS)
    msg, parts = frame_msg(names, seed=21)
    for p in parts:
        sep.image_callback(p)
    sep.timers[0].callback()
    settle(one)
    one.image_callback(msg)
    assert list(pub_one.sent[-1].data) == list(pub_sep.sent[-1].data)
    assert len(pub_one.sent) == 2             # чёрный старт + кадр, без ожидания таймера


def test_frame_message_may_carry_only_some_groups(params):
    node, pub = make(params, str(PKG_YAML))
    ref, pub_ref = make(params, str(PKG_YAML))
    msg, parts = frame_msg(['main_display', 'wheel_rear_left'], seed=3)
    node.image_callback(msg)
    for p in parts:
        ref.image_callback(p)
    ref.timers[0].callback()
    assert list(pub.sent[-1].data) == list(pub_ref.sent[-1].data)


def test_bad_frame_message_changes_nothing(params):
    node, pub = make(params, str(PKG_YAML))
    msg, _ = frame_msg(['main_display', 'wheel_rear_left'])
    msg.data = msg.data[:-3]
    node.image_callback(msg)
    unknown, _ = frame_msg(['main_display', 'wheel_rear_left'])
    unknown.header.frame_id = 'main_display,nope'
    node.image_callback(unknown)
    assert len(pub.sent) == 1 and not any(node.buffer)


def test_static_scene_publishes_nothing_after_the_first_frame(params):
    node, pub = make(params, str(PKG_YAML))
    msg, _ = frame_msg(list(GROUPS), seed=5)
    settle(node)
    node.image_callback(msg)
    sent = len(pub.sent)
    for _ in range(100):                      # лицо повторяет тот же кадр
        node.image_callback(msg)
        node.timers[0].callback()
    assert len(pub.sent) == sent


def test_animated_scene_is_one_strip_write_per_frame(params):
    node, pub = make(params, str(PKG_YAML))
    settle(node)
    node.image_callback(frame_msg(list(GROUPS), seed=1)[0])
    before = len(pub.sent)
    for i in range(1, 26):
        settle(node)                          # кадры идут реже 60 Гц
        node.image_callback(frame_msg(list(GROUPS), seed=1 + i)[0])
    assert len(pub.sent) - before == 25


def test_flush_timer_is_idle_until_an_image_arrives_and_disarms_itself(params):
    node, pub = make(params, str(PKG_YAML))
    flush = node.timers[0]
    assert flush.armed is False               # вхолостую не тикает
    node.image_callback(image('main_display', 1))
    assert flush.armed is True and flush.resets == 1
    flush.callback()
    assert flush.armed is False


def test_rate_cap_defers_a_too_early_frame_to_the_one_shot_timer(params):
    node, pub = make(params, str(PKG_YAML))
    settle(node)
    node.image_callback(frame_msg(list(GROUPS), seed=1)[0])
    n = len(pub.sent)
    node.image_callback(frame_msg(list(GROUPS), seed=2)[0])   # через микросекунды: лимит 60 Гц
    assert len(pub.sent) == n and node.timers[0].armed
    node.timers[0].callback()
    assert len(pub.sent) == n + 1 and not node.timers[0].armed


def test_keepalive_repeats_the_same_strip_rarely(params):
    node, pub = make(params, str(PKG_YAML))
    keep = node.timers[1]
    assert abs(keep.period - 2.0) < 1e-9
    settle(node)
    node.image_callback(frame_msg(list(GROUPS), seed=1)[0])
    n = len(pub.sent)
    keep.callback()                           # только что писали: рано
    assert len(pub.sent) == n
    node._last_publish_at -= 3.0
    keep.callback()
    assert len(pub.sent) == n + 1
    assert list(pub.sent[-1].data) == list(pub.sent[-2].data)
