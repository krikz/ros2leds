"""Colours in LED levels and small numeric helpers shared by the effects.

Effects draw in *levels*, not in 0..255 bytes: at driver brightness ``b`` a wire byte ``v`` lights the LED at
``v * b // 255``, so only ``b + 1`` shades exist (26 at the default brightness 25). Designing in levels keeps every
shade an effect picks distinguishable on the strip; :class:`led_matrix_effects.power.WireModel` turns levels into
wire bytes.
"""
from __future__ import annotations

import numpy as np


def col(r: float, g: float, b: float) -> np.ndarray:
    """An RGB colour in levels as a float array."""
    return np.array((r, g, b), dtype=float)


BLACK = col(0, 0, 0)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


_HUE_AT = np.array((3.0, 2.0, 4.0))


def hue(h: np.ndarray) -> np.ndarray:
    """Hue 0..1 (any real, wraps) -> RGB 0..1 at full saturation, element-wise; shape ``h.shape + (3,)``."""
    d = np.abs((h % 1.0)[..., None] * 6.0 - _HUE_AT)
    rgb = 2.0 - d
    rgb[..., 0] = d[..., 0] - 1.0
    return np.clip(rgb, 0.0, 1.0)


__all__ = ['BLACK', 'clamp', 'col', 'hue']
