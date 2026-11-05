"""Levels -> wire bytes, and the strip current of a frame by the driver's own model.

The ``led_matrix_driver`` sends every channel as ``v * brightness // 255`` and estimates the strip current as
``idle_ma_per_led * num_leds + sum(channels) / 255 * ma_per_channel``. :class:`Strip` holds the same four
parameters (same names as in ``led_matrix_driver.yaml``), so an application can check a scene against its current
budget before it reaches the driver's limiter.
"""
from __future__ import annotations

from pathlib import Path
from typing import Iterable, Mapping, Union

import numpy as np


class Strip:
    """The driver as an effect sees it: how many levels it shows and how much current a frame draws.

    At brightness ``b`` a level ``L`` in ``0..b`` survives the driver exactly: :meth:`to_wire` sends
    ``ceil(L / b * 255)`` and the driver shows ``that * b // 255 == L``. So ``max_level == brightness``."""

    def __init__(self, num_leds: int, brightness: int = 25, ma_per_channel: float = 20.0,
                 idle_ma_per_led: float = 1.0) -> None:
        self.num_leds = int(num_leds)
        self.brightness = int(brightness)
        self.ma_per_channel = float(ma_per_channel)
        self.idle_ma_per_led = float(idle_ma_per_led)
        #: brightest level an effect should use
        self.max_level = float(self.brightness)
        #: current of the dark strip, mA
        self.idle_ma = self.num_leds * self.idle_ma_per_led

    @classmethod
    def from_driver(cls, config: Union[str, Path, Mapping]) -> 'Strip':
        """From ``led_matrix_driver.yaml`` (a path or the parsed mapping, whole file or its ``ros__parameters``).

        Missing parameters take the driver's defaults."""
        if not isinstance(config, Mapping):
            from .layout import load_yaml

            config = load_yaml(config)
        if 'num_leds' not in config and 'ros__parameters' not in config and len(config) == 1:
            config = next(iter(config.values()))        # {node_name: {ros__parameters: ...}}
        config = config.get('ros__parameters', config)
        return cls(config['num_leds'], brightness=config.get('brightness', 25),
                   ma_per_channel=config.get('ma_per_channel', 20.0),
                   idle_ma_per_led=config.get('idle_ma_per_led', 1.0))

    def to_wire(self, levels) -> np.ndarray:
        """Levels ``0..max_level`` -> channel bytes for the driver (``ceil``, so that every level comes back)."""
        v = np.clip(np.asarray(levels, dtype=float), 0.0, self.max_level) / self.max_level * 255.0
        return np.ceil(v - 1e-6).astype(np.uint8)

    def wire_ma(self, wires: Iterable[np.ndarray], brightness: int = None) -> float:
        """Strip current, mA, of panels already in wire bytes (all LEDs of the strip that are not listed are dark)."""
        b = self.brightness if brightness is None else brightness
        lit = sum(int((w.astype(np.int64) * b // 255).sum()) for w in wires)
        return self.idle_ma + lit / 255.0 * self.ma_per_channel

    def frame_ma(self, frame, brightness: int = None) -> float:
        """Strip current, mA, of a frame in levels (anything with ``panels()``)."""
        return self.wire_ma([self.to_wire(a) for a in frame.panels().values()], brightness)


__all__ = ['Strip']
