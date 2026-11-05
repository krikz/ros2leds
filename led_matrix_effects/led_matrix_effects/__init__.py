"""LED matrix effects: pure numpy pictures for the panels of ros2leds.

* :mod:`~led_matrix_effects.layout` — :class:`Layout` (panels, sizes, canvas places; from the compositor YAML)
  and :class:`Frame` (one picture of all panels);
* :mod:`~led_matrix_effects.canvas` — :class:`Canvas`, every LED with its canvas coordinates;
* :mod:`~led_matrix_effects.backgrounds` — rainbow, fire, matrix rain, and :func:`under`;
* :mod:`~led_matrix_effects.pulses` — slash, glitch, boot, power-down;
* :mod:`~led_matrix_effects.signals` — beacons, flashers, police / roadworks patterns, fire, ECG;
* :mod:`~led_matrix_effects.lamps` — tail light, turn signal, holiday and music themes for 8x8 round lamps;
* :mod:`~led_matrix_effects.power` — :class:`Strip`: levels -> wire bytes, strip current by the driver's model;
* :mod:`~led_matrix_effects.preview` — PIL previews (needs Pillow).

No ROS import anywhere: effects run and are tested on any machine with numpy.
"""
from .backgrounds import Fire, MatrixRain, Rainbow, under
from .canvas import Canvas
from .color import BLACK, clamp, col, hue
from .layout import Frame, Layout, read_groups
from .power import Strip
from .pulses import Boot, Glitch, PowerDown, Slash

__all__ = ['BLACK', 'Boot', 'Canvas', 'Fire', 'Frame', 'Glitch', 'Layout', 'MatrixRain', 'PowerDown', 'Rainbow',
           'Slash', 'Strip', 'clamp', 'col', 'hue', 'read_groups', 'under']
