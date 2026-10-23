#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Int8MultiArray
import time

from led_matrix_driver import frame_math

# Импортируем библиотеку для управления матрицей
try:
    from pi5neo import Pi5Neo
    PI5NEO_AVAILABLE = True
except ImportError:
    PI5NEO_AVAILABLE = False
    print("Warning: pi5neo library not found. Running in simulation mode.")


class LEDMatrixSimple(Node):
    def __init__(self):
        super().__init__('led_matrix_simple')

        # Числовые параметры тока принимают и int, и float из YAML: опечатка
        # «4500» вместо «4500.0» не должна гасить ленту при старте.
        number = ParameterDescriptor(dynamic_typing=True)

        # Объявляем параметры
        self.declare_parameters(
            namespace='',
            parameters=[
                ('num_leds', 253),
                ('spi_speed_khz', 1000),
                ('spi_device', '/dev/spidev0.0'),
                ('input_topic', 'led_matrix/data'),
                ('brightness', 25),  # Яркость 0-255 (25 = ~10%)
                # Ограничитель тока: бак 5 В × 30 Вт = 6 А, лимит с запасом 25%.
                # Паспорт WS2812B: ~20 мА на канал при 255, ~1 мА покоя на LED.
                ('max_current_ma', 4500.0, number),  # <= 0 выключает ограничитель
                ('ma_per_channel', 20.0, number),
                ('idle_ma_per_led', 1.0, number),
                # Пауза между записями для защёлки WS2812 (>= 280 мкс низкого уровня).
                # Раньше вместо неё после КАЖДОЙ записи был сон 100 мс внутри
                # pi5neo.update_strip() — потолок ~9 кадров в секунду на всю ленту.
                ('min_frame_interval_s', 0.0003, number),
                # Каждое сообщение — вся лента, нужна только последняя.
                ('queue_depth', 1),
            ]
        )

        # Получаем параметры
        self.num_leds = self.get_parameter('num_leds').value
        self.spi_speed_khz = self.get_parameter('spi_speed_khz').value
        self.spi_device = self.get_parameter('spi_device').value
        self.input_topic = self.get_parameter('input_topic').value
        self.brightness = self.get_parameter('brightness').value
        self.max_current_ma = float(self.get_parameter('max_current_ma').value)
        self.ma_per_channel = float(self.get_parameter('ma_per_channel').value)
        self.idle_ma_per_led = float(self.get_parameter('idle_ma_per_led').value)
        self.min_frame_interval_s = float(self.get_parameter('min_frame_interval_s').value)
        self.queue_depth = max(1, int(self.get_parameter('queue_depth').value))

        self._last_write = 0.0
        self._limited_frames = 0
        # Не 0.0: time.monotonic() на Linux - секунды с загрузки машины, и при 0.0
        # первое предупреждение глотается, пока аптайм < 60 с (CI-раннер, Pi после
        # старта). -inf = «ещё ни разу не писали», первый warn идёт сразу.
        self._limit_log_at = float('-inf')
        self._min_scale = 1.0

        # Проверяем корректность параметров
        if self.num_leds <= 0:
            self.get_logger().error("Parameter 'num_leds' must be positive")
            raise ValueError("Parameter 'num_leds' must be positive")

        if self.brightness < 0 or self.brightness > 255:
            self.get_logger().error("Parameter 'brightness' must be between 0 and 255")
            raise ValueError("Parameter 'brightness' must be between 0 and 255")

        # Инициализируем матрицу, если доступна библиотека pi5neo
        self.matrix_initialized = False
        if PI5NEO_AVAILABLE:
            try:
                self.neo = Pi5Neo(self.spi_device, self.num_leds, self.spi_speed_khz)
                self.matrix_initialized = True
                self.get_logger().info(f"LED matrix initialized with {self.spi_speed_khz} kHz SPI speed")
                self.get_logger().info(f"Total LEDs: {self.num_leds}")
                self.get_logger().info(f"Brightness: {self.brightness}/255 ({int(self.brightness/255*100)}%)")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize LED matrix: {str(e)}")
                self.matrix_initialized = False
        else:
            self.get_logger().warn("Running in simulation mode (pi5neo not available)")

        # Подписываемся на топик с данными
        self.data_subscription = self.create_subscription(
            Int8MultiArray,
            self.input_topic,
            self.data_callback,
            self.queue_depth)

        self.get_logger().info(f"Subscribed to topic: {self.input_topic} (queue depth {self.queue_depth})")
        if self.max_current_ma > 0:
            self.get_logger().info(
                f"Current limit: {self.max_current_ma:.0f} mA "
                f"({self.ma_per_channel:g} mA/channel, {self.idle_ma_per_led:g} mA idle per LED)"
            )
        self.get_logger().info("LED Matrix Simple Driver is ready")

    def data_callback(self, msg):
        """Обрабатывает входящие данные для отображения на матрице."""
        if not self.matrix_initialized:
            self.get_logger().warn("Matrix not initialized, skipping data")
            return

        # Проверяем длину данных
        expected_length = self.num_leds * 3
        if len(msg.data) != expected_length:
            self.get_logger().warn(f"Received data with incorrect length: {len(msg.data)} (expected {expected_length})")
            return

        try:
            values = frame_math.apply_brightness(frame_math.to_unsigned(msg.data), self.brightness)
            values, scale = frame_math.limit_current(
                values, self.num_leds, self.max_current_ma, self.ma_per_channel, self.idle_ma_per_led
            )
            if scale < 1.0:
                self._note_limited(scale)
            self._write_frame(values)
        except Exception as e:
            self.get_logger().error(f"Error displaying  {str(e)}")

    def _write_frame(self, values):
        """Пишет кадр в ленту без сна pi5neo.update_strip() по умолчанию (100 мс)."""
        wait = self.min_frame_interval_s - (time.monotonic() - self._last_write)
        if wait > 0:
            time.sleep(wait)
        neo = self.neo
        fast = (
            hasattr(neo, 'raw_data') and hasattr(neo, 'send_spi_data')
            and getattr(neo, 'bytes_per_led', 24) == 24
            and getattr(getattr(neo, 'pixel_type', None), 'name', 'GRB') == 'GRB'
        )
        if fast:
            # Тот же поток байт, что собрал бы update_strip(), но без
            # питоновского цикла по LEDColor и без сна после передачи.
            neo.raw_data = list(frame_math.encode_grb(values))
            neo.send_spi_data()
        else:
            for i in range(self.num_leds):
                idx = i * 3
                neo.set_led_color(i, values[idx], values[idx + 1], values[idx + 2])
            neo.update_strip(sleep_duration=None)
        self._last_write = time.monotonic()

    def _note_limited(self, scale):
        """Считает кадры, срезанные ограничителем; в лог — не чаще раза в минуту."""
        self._limited_frames += 1
        self._min_scale = min(self._min_scale, scale)
        now = time.monotonic()
        if now - self._limit_log_at < 60.0:
            return
        self.get_logger().warn(
            f"Current limiter: {self._limited_frames} frame(s) scaled down to stay under "
            f"{self.max_current_ma:.0f} mA (deepest scale {self._min_scale:.2f})"
        )
        self._limit_log_at = now
        self._limited_frames = 0
        self._min_scale = 1.0

    def clear_matrix(self, retries: int = 3):
        """Очищает матрицу (гасит все LED), устойчиво к сбоям SPI.

        Если SPI-запись падает во время очистки, LED остаются в последнем
        состоянии (горят). Поэтому:
          - clear_strip() и update_strip() выполняются в отдельных try,
            чтобы сбой одной операции не блокировал другую;
          - при ошибке update_strip() пробуем переоткрыть SPI-устройство
            и повторить очистку до ``retries`` раз;
          - если всё ещё не получилось — форсируем очистку напрямую
            через fill_strip(0,0,0,0) + update_strip().
        """
        if not self.matrix_initialized:
            self.get_logger().warn("Matrix not initialized, cannot clear")
            return

        for attempt in range(1, retries + 1):
            try:
                self.neo.clear_strip()
            except Exception as e:
                self.get_logger().warn(f"clear_strip failed (attempt {attempt}/{retries}): {str(e)}")

            try:
                self.neo.update_strip()
                self.get_logger().info("Matrix cleared on shutdown")
                return
            except Exception as e:
                self.get_logger().warn(
                    f"update_strip failed (attempt {attempt}/{retries}): {str(e)}"
                )
                # SPI-устройство могло пропасть (например, удалено при остановке):
                # пробуем переоткрыть его перед следующей попыткой.
                try:
                    self.neo.close()
                except Exception:
                    pass
                try:
                    self.neo.open_spi_device(self.spi_device)
                except Exception as e:
                    self.get_logger().warn(f"Failed to reopen SPI device: {str(e)}")

        # Финальная попытка: force-очистка через fill_strip в обход clear_strip
        try:
            self.neo.fill_strip(0, 0, 0, 0)
            self.neo.update_strip()
            self.get_logger().warn("Matrix cleared via forced fill_strip")
        except Exception as e:
            self.get_logger().error(f"Error clearing matrix on shutdown: {str(e)}")

    def destroy_node(self):
        """Очистка при завершении работы."""
        if self.matrix_initialized:
            self.clear_matrix()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    led_matrix_simple = LEDMatrixSimple()

    try:
        rclpy.spin(led_matrix_simple)
    except KeyboardInterrupt:
        pass
    finally:
        led_matrix_simple.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
