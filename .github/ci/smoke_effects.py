"""CI smoke test of the installed led_matrix_effects package: layout from the packaged compositor config, one frame.

Usage: python3 smoke_effects.py <led_matrix_compositor.yaml>
"""
import sys

import led_matrix_effects
from led_matrix_effects import Canvas, Layout, Rainbow, Strip

canvas = Canvas(Layout.from_compositor(sys.argv[1]))
frame = Rainbow(canvas)(0.5)
strip = Strip(canvas.layout.n_leds)
print(f'led_matrix_effects from {led_matrix_effects.__file__}')
print(f'{canvas.layout.n_leds} LEDs, panels {list(canvas.panels)}, canvas {canvas.width}x{canvas.height}, '
      f'rainbow frame {strip.frame_ma(frame):.0f} mA')
assert canvas.layout.n_leds == 381, canvas.layout.n_leds
assert '/install/' in led_matrix_effects.__file__.replace('\\', '/'), 'not the installed package'
