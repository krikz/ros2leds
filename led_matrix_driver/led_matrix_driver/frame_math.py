"""Чистая логика кадра драйвера: яркость, оценка тока, ограничитель, SPI-код.

Без ROS и без железа — тестируется как обычный модуль. Все операции над
кадром идут через ``bytes.translate`` и срезы, то есть в C: на Vision Pi
питоновский цикл ``pi5neo.update_strip()`` стоил 2,56 мс на 381 светодиод.

Формат входа — то, что композитор кладёт в ``Int8MultiArray``: байт
канала минус 128. Беззнаковое значение канала восстанавливается как
``сырой_байт ^ 0x80`` (то же самое, что ``v + 128`` для int8 ``v``).
"""

# SPI-байт на один бит WS2812 при 8 SPI-битах на бит (как в pi5neo).
SPI_BIT_HIGH = 0xF8
SPI_BIT_LOW = 0xC0

_XOR_80 = bytes(v ^ 0x80 for v in range(256))
_SPI_LUT = [
    bytes(SPI_BIT_HIGH if (v >> (7 - i)) & 1 else SPI_BIT_LOW for i in range(8))
    for v in range(256)
]
_brightness_tables = {}


def to_unsigned(data):
    """Данные Int8MultiArray -> bytes с каналами 0..255."""
    if isinstance(data, (bytes, bytearray)):
        raw = bytes(data)
    elif hasattr(data, 'tobytes'):
        raw = data.tobytes()          # array.array('b') из rclpy
    else:
        raw = bytes(v & 0xFF for v in data)
    return raw.translate(_XOR_80)


def apply_brightness(values, brightness):
    """``v * brightness // 255`` на каждый канал — формула прежнего драйвера."""
    table = _brightness_tables.get(brightness)
    if table is None:
        table = bytes((v * brightness) // 255 for v in range(256))
        _brightness_tables[brightness] = table
    return values.translate(table)


def estimate_current_ma(values, num_leds, ma_per_channel, idle_ma_per_led):
    """Ток ленты по паспортной модели WS2812B: покой + доля ШИМ каждого канала."""
    return num_leds * idle_ma_per_led + sum(values) / 255.0 * ma_per_channel


def limit_current(values, num_leds, max_current_ma, ma_per_channel, idle_ma_per_led):
    """Если кадр тянет больше ``max_current_ma``, гасит все каналы одним коэффициентом.

    Возвращает ``(values, scale)``; ``scale == 1.0`` — кадр не тронут.
    Округление вниз гарантирует, что результат не превышает лимит.
    ``max_current_ma <= 0`` выключает ограничитель.
    """
    if max_current_ma <= 0:
        return values, 1.0
    idle = num_leds * idle_ma_per_led
    dynamic = sum(values) / 255.0 * ma_per_channel
    if idle + dynamic <= max_current_ma or dynamic <= 0:
        return values, 1.0
    scale = max(0.0, (max_current_ma - idle) / dynamic)
    table = bytes(int(v * scale) for v in range(256))
    return values.translate(table), scale


def encode_grb(values):
    """RGB-кадр -> SPI-поток WS2812 в порядке GRB, 8 SPI-байт на бит.

    Байт в байт то же, что ``Pi5Neo.update_strip()`` кладёт в ``raw_data``
    для ``EPixelType.GRB``.
    """
    grb = bytearray(len(values))
    grb[0::3] = values[1::3]
    grb[1::3] = values[0::3]
    grb[2::3] = values[2::3]
    return b''.join(map(_SPI_LUT.__getitem__, grb))
