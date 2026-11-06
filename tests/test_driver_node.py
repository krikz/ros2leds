"""Нода драйвера: без сна 100 мс, очередь 1, ограничитель тока, прежняя картинка."""

import array
import random
import time
from types import SimpleNamespace

from conftest import FakePi5Neo

from led_matrix_driver import frame_math
from led_matrix_driver import led_matrix_driver as drv

N = 381


def make_driver(params, **overrides):
    params.update({'num_leds': N})
    params.update(overrides)
    node = drv.LEDMatrixSimple()
    return node, FakePi5Neo.instances[-1]


def frame_msg(int8_values):
    return SimpleNamespace(data=array.array('b', int8_values))


def random_int8(seed):
    rng = random.Random(seed)
    return [rng.randint(-128, 127) for _ in range(N * 3)]


def old_driver_spi(int8_values, brightness):
    """То, что отправлял прежний драйвер: формула яркости + кодирование pi5neo."""
    vals = bytes(((v + 128) * brightness) // 255 for v in int8_values)
    return list(frame_math.encode_grb(vals))


def test_subscription_keeps_only_latest_frame(params):
    node, _ = make_driver(params)
    assert node.subscriptions[0].topic == 'led_matrix/data'
    assert node.subscriptions[0].depth == 1


def test_frame_at_brightness_25_is_byte_identical_to_old_driver(params):
    node, neo = make_driver(params, brightness=25)
    data = random_int8(4)
    node.data_callback(frame_msg(data))
    assert neo.sent == [old_driver_spi(data, 25)]


def test_no_100ms_sleep_after_write(params):
    node, neo = make_driver(params)
    t0 = time.monotonic()
    for i in range(10):
        node.data_callback(frame_msg(random_int8(50 + i)))
    elapsed = time.monotonic() - t0
    assert len(neo.sent) == 10
    assert neo.update_calls == []          # быстрый путь не зовёт update_strip()
    assert elapsed < 0.5, f'10 frames took {elapsed:.3f}s; old driver needed >= 1 s'


def test_fallback_path_disables_pi5neo_sleep(params):
    node, neo = make_driver(params)
    del neo.raw_data                       # другая версия pi5neo без raw_data
    node.data_callback(frame_msg(random_int8(6)))
    assert neo.update_calls == [None]


def test_current_limiter_caps_full_white_at_full_brightness(params):
    node, neo = make_driver(params, brightness=255)
    node.data_callback(frame_msg([127] * (N * 3)))       # 255 на каждом канале
    # восстановить яркость каналов из SPI-потока: 8 SPI-байт на бит канала
    spi = neo.sent[-1]
    values = bytes(
        sum(1 << (7 - k) for k in range(8) if spi[i * 8 + k] == 0xF8)
        for i in range(N * 3)
    )
    assert frame_math.estimate_current_ma(values, N, 20.0, 1.0) <= 4500.0
    assert any('Current limiter' in m for lvl, m in node.get_logger().records if lvl == 'warn')


def _limiter_warns(node):
    return [m for lvl, m in node.get_logger().records
            if lvl == 'warn' and 'Current limiter' in m]


def test_limiter_warns_on_first_frame_even_right_after_boot(params, monkeypatch):
    """Аптайм 30 с: time.monotonic() < 60 не должен глотать первое предупреждение.

    Раньше _limit_log_at стартовал с 0.0, и «now - 0.0 < 60» подавляло warn
    в первую минуту после загрузки машины (CI-раннер, Vision Pi).
    """
    clock = [30.0]
    monkeypatch.setattr(drv.time, 'monotonic', lambda: clock[0])
    node, _ = make_driver(params, brightness=255, min_frame_interval_s=0.0)
    full_white = frame_msg([127] * (N * 3))

    node.data_callback(full_white)
    assert len(_limiter_warns(node)) == 1

    # троттлинг не сломан: в пределах 60 с второе предупреждение не пишется
    clock[0] = 30.0 + 59.0
    node.data_callback(full_white)
    assert len(_limiter_warns(node)) == 1

    # а через 60 с от прошлого - пишется снова, со счётчиком накопленных кадров
    clock[0] = 30.0 + 60.0
    node.data_callback(full_white)
    warns = _limiter_warns(node)
    assert len(warns) == 2
    assert '2 frame(s)' in warns[1]


def test_wrong_length_frame_is_skipped(params):
    node, neo = make_driver(params)
    node.data_callback(frame_msg([0] * 30))
    assert neo.sent == []


def test_same_frame_is_not_written_twice_but_refreshed_on_schedule(params, monkeypatch):
    """#3733: повтор того же кадра не гонит SPI; раз в repeat_refresh_s лента всё же обновляется."""
    clock = [100.0]
    monkeypatch.setattr(drv.time, 'monotonic', lambda: clock[0])
    node, neo = make_driver(params, repeat_refresh_s=2.0, min_frame_interval_s=0.0)
    data = random_int8(7)
    node.data_callback(frame_msg(data))
    for _ in range(20):
        clock[0] += 0.04
        node.data_callback(frame_msg(data))
    assert len(neo.sent) == 1
    node.data_callback(frame_msg(random_int8(8)))               # другой кадр пишется сразу
    assert len(neo.sent) == 2
    clock[0] += 2.5
    node.data_callback(frame_msg(random_int8(8)))               # тот же, но пора обновить ленту
    assert len(neo.sent) == 3


def test_repeat_refresh_zero_writes_every_message(params):
    node, neo = make_driver(params, repeat_refresh_s=0)
    data = random_int8(9)
    for _ in range(3):
        node.data_callback(frame_msg(data))
    assert len(neo.sent) == 3
