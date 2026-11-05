#!/usr/bin/env python3
"""CI smoke test: one image into a logical group -> one full strip out.

Run with the compositor already up. Publishes a solid red 25x5 frame to the
``main_display`` group on ``panel_image`` and waits for the compositor to
publish the whole strip on ``led_matrix/data``. Exits non-zero on timeout or
on a strip of the wrong size.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray

NUM_LEDS = 381          # packaged config: 5 x (5x5) + 4 x (8x8)
GROUP, WIDTH, HEIGHT = 'main_display', 25, 5
TIMEOUT_S = 15.0


def main():
    rclpy.init()
    node = Node('ci_smoke_compositor')
    strips = []
    node.create_subscription(Int8MultiArray, 'led_matrix/data', lambda m: strips.append(m), 10)
    pub = node.create_publisher(Image, 'panel_image', 10)

    msg = Image()
    msg.header.frame_id = GROUP
    msg.width, msg.height = WIDTH, HEIGHT
    msg.encoding = 'rgb8'
    msg.step = WIDTH * 3
    msg.data = bytes([255, 0, 0] * WIDTH * HEIGHT)

    deadline = time.monotonic() + TIMEOUT_S
    lit = None
    while time.monotonic() < deadline and lit is None:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
        for strip in strips:
            # Wire format: channel byte minus 128, so 255 arrives as 127.
            if sum(1 for v in strip.data if v == 127) > 0:
                lit = strip
                break

    node.destroy_node()
    rclpy.shutdown()
    if lit is None:
        print(f'FAIL: no lit strip on led_matrix/data within {TIMEOUT_S:.0f} s '
              f'({len(strips)} strip(s) received)')
        return 1
    red = sum(1 for i in range(0, len(lit.data), 3) if lit.data[i] == 127)
    print(f'strip length: {len(lit.data)} bytes, LEDs with red=255: {red}')
    if len(lit.data) != NUM_LEDS * 3:
        print(f'FAIL: expected {NUM_LEDS * 3} bytes')
        return 1
    if red != WIDTH * HEIGHT:
        print(f'FAIL: expected {WIDTH * HEIGHT} red LEDs (the whole main_display group)')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
