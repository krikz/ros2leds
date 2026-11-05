"""Panel size and colour scale for the per-panel pictures (``signals``, ``lamps``).

**Size.** Pictures were designed for an 8x8 round lamp (:data:`LAMP`). Procedural ones (beacons, flashers, rings,
bars, disco) are computed on a grid of any ``size = (width, height)``: :func:`lamp_grid` gives the distance of
every LED from the panel centre in *lamp units* — the panel is stretched to 8x8, so a lens fills a square panel and
becomes an ellipse on a long one. Sprites (holiday pictures) are drawn at 8x8 and enlarged by an integer factor,
nearest neighbour, centred (:func:`fit`); a panel smaller than the sprite raises :class:`SpriteTooSmall` instead of
silently cutting it.

**Colour scale.** Colours are written in levels of :data:`FULL` = 25 (the driver's default brightness 25 shows 26
shades). ``levels`` is the brightest level of *your* strip — usually the driver's brightness, ``Strip.max_level``;
every picture is multiplied by ``levels / FULL``. At the defaults (8x8, 25 levels) nothing is resized or rescaled:
the output is byte for byte the one the pictures were designed with.
"""
from __future__ import annotations

from functools import lru_cache
from typing import Tuple

import numpy as np

#: The size the pictures were designed for: an 8x8 round lamp.
LAMP: Tuple[int, int] = (8, 8)
#: Levels the colours are written in (brightest channel = 25).
FULL = 25.0


class SpriteTooSmall(ValueError):
    """A sprite does not fit the panel: shrinking it would destroy the picture."""


def _wh(size) -> Tuple[int, int]:
    w, h = int(size[0]), int(size[1])
    if w < 1 or h < 1:
        raise ValueError(f'panel size must be positive, got {size!r}')
    return w, h


@lru_cache(maxsize=64)
def _grid(w: int, h: int):
    yy, xx = np.mgrid[0:h, 0:w]
    sx, sy = w / LAMP[0], h / LAMP[1]
    cx, cy = (w - 1) / 2, (h - 1) / 2
    d = np.hypot((xx - cx) / sx, (yy - cy) / sy)
    a = np.arctan2((yy - cy) / sy, (xx - cx) / sx)
    for arr in (d, a):
        arr.flags.writeable = False
    return d, a


def lamp_grid(size=LAMP) -> Tuple[np.ndarray, np.ndarray]:
    """``(distance, angle)`` of every LED of a ``(width, height)`` panel from its centre, in lamp units.

    Distance 3.95 is the rim of the lens on any panel; the arrays are shared, read-only, shape ``(height, width)``."""
    return _grid(*_wh(size))


def fit(img: np.ndarray, size=LAMP) -> np.ndarray:
    """An ``(8, 8, 3)`` sprite on a ``(width, height)`` panel: enlarged by the largest integer factor that fits
    (nearest neighbour), centred. Raises :class:`SpriteTooSmall` when the panel is smaller than the sprite."""
    w, h = _wh(size)
    sh, sw = img.shape[:2]
    if (w, h) == (sw, sh):
        return img
    if w < sw or h < sh:
        raise SpriteTooSmall(f'a {sw}x{sh} sprite does not fit a {w}x{h} panel (shrinking would destroy it)')
    k = min(w // sw, h // sh)
    big = np.repeat(np.repeat(img, k, axis=0), k, axis=1)
    out = np.zeros((h, w) + img.shape[2:], dtype=img.dtype)
    y0, x0 = (h - sh * k) // 2, (w - sw * k) // 2
    out[y0:y0 + sh * k, x0:x0 + sw * k] = big
    return out


def scaled(out, levels: float = FULL):
    """A picture (array or tuple of arrays) in levels of :data:`FULL` -> in levels of ``levels``."""
    if levels == FULL:
        return out
    k = levels / FULL
    if isinstance(out, tuple):
        return tuple(o * k for o in out)
    return out * k


__all__ = ['FULL', 'LAMP', 'SpriteTooSmall', 'fit', 'lamp_grid', 'scaled']
