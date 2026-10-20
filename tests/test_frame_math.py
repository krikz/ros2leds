"""Логика кадра драйвера: совместимость с прежним драйвером и pi5neo, ограничитель тока."""

import array
import random

from led_matrix_driver import frame_math

N = 381


def pi5neo_reference_grb(values):
    """Алгоритм pi5neo 1.1.0 (update_strip, EPixelType.GRB) один в один."""
    out = []
    for i in range(0, len(values), 3):
        r, g, b = values[i], values[i + 1], values[i + 2]
        for byte in (g, r, b):
            out.extend(0xF8 if (byte & (1 << (7 - k))) else 0xC0 for k in range(8))
    return out


def old_driver_values(int8_data, brightness):
    """Формула прежнего data_callback: ((v + 128) * brightness) // 255."""
    return [((v + 128) * brightness) // 255 for v in int8_data]


def random_int8(seed, n=N * 3):
    rng = random.Random(seed)
    return [rng.randint(-128, 127) for _ in range(n)]


def test_encoding_matches_pi5neo_byte_for_byte():
    rng = random.Random(1)
    values = bytes(rng.randrange(256) for _ in range(N * 3))
    assert list(frame_math.encode_grb(values)) == pi5neo_reference_grb(values)
    assert len(frame_math.encode_grb(values)) == N * 24


def test_brightness_matches_old_driver_formula():
    data = random_int8(2)
    for brightness in (0, 1, 25, 128, 255):
        got = frame_math.apply_brightness(frame_math.to_unsigned(array.array('b', data)), brightness)
        assert list(got) == old_driver_values(data, brightness)


def test_to_unsigned_accepts_every_container_rclpy_may_hand_over():
    data = [-128, -1, 0, 1, 127]
    expected = [0, 127, 128, 129, 255]
    assert list(frame_math.to_unsigned(array.array('b', data))) == expected
    assert list(frame_math.to_unsigned(data)) == expected
    assert list(frame_math.to_unsigned(bytes(v & 0xFF for v in data))) == expected


def test_estimate_full_white_is_datasheet_number():
    white = bytes([255]) * (N * 3)
    assert frame_math.estimate_current_ma(white, N, 20.0, 1.0) == N * 1.0 + N * 3 * 20.0


def test_limiter_leaves_frames_under_budget_untouched():
    frame = bytes([25]) * (N * 3)         # всё белое при яркости 25: ~2,6 А
    out, scale = frame_math.limit_current(frame, N, 4500.0, 20.0, 1.0)
    assert scale == 1.0 and out == frame


def test_limiter_keeps_full_white_under_budget():
    white = bytes([255]) * (N * 3)        # 23,2 А по паспорту
    out, scale = frame_math.limit_current(white, N, 4500.0, 20.0, 1.0)
    assert 0 < scale < 1
    assert frame_math.estimate_current_ma(out, N, 20.0, 1.0) <= 4500.0
    # масштаб один на все каналы: цвет не плывёт
    assert len(set(out)) == 1


def test_limiter_preserves_colour_ratios():
    rng = random.Random(3)
    frame = bytes(rng.randrange(256) for _ in range(N * 3))
    out, scale = frame_math.limit_current(frame, N, 3000.0, 20.0, 1.0)
    assert scale < 1
    assert frame_math.estimate_current_ma(out, N, 20.0, 1.0) <= 3000.0
    for a, b in zip(frame, out):
        assert b == int(a * scale)


def test_limiter_disabled_by_non_positive_budget():
    white = bytes([255]) * (N * 3)
    assert frame_math.limit_current(white, N, 0, 20.0, 1.0) == (white, 1.0)
